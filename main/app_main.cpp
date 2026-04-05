#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <netinet/in.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <unistd.h>

#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "lwip/inet.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <esp_matter.h>
#include <platform/PlatformManager.h>
#include <setup_payload/OnboardingCodesUtil.h>

#define APP_WIFI_SSID            "ESP32C6-LED-Setup"
#define APP_WIFI_PASS            "esp32c6led"
#define APP_WIFI_CHANNEL         6
#define APP_WIFI_MAX_STA_CONN    4

#define APP_LED_GPIO             17
#define APP_LED_MAX_PIXELS       120
#define APP_LED_DEFAULT_COUNT    8
#define APP_LED_DEFAULT_RED      255
#define APP_LED_DEFAULT_GREEN    96
#define APP_LED_DEFAULT_BLUE     32
#define APP_LED_DEFAULT_BRIGHT   96
#define APP_LED_DEFAULT_POWER    0
#define APP_NVS_NAMESPACE        "led_cfg"
#define APP_POST_BODY_LIMIT      1024
#define APP_OTA_CHUNK_SIZE       4096
#define APP_RMT_RESOLUTION_HZ    (10 * 1000 * 1000)
#define APP_QR_CODE_MAX          128
#define APP_MANUAL_CODE_MAX      32
#define APP_QR_URL_MAX           256

static const char *TAG = "matter_led";
static constexpr auto kCommissioningTimeoutSeconds = 300;
static constexpr uint16_t kDefaultColorTempMireds = 0x00fa;
static constexpr uint16_t kDefaultCurrentX = 0x616b;
static constexpr uint16_t kDefaultCurrentY = 0x607d;
static constexpr size_t kEffectParamSlotCount = 5;

using namespace esp_matter;
using namespace chip::app::Clusters;

typedef enum {
    LED_EFFECT_SOLID = 0,
    LED_EFFECT_GLOW,
    LED_EFFECT_RAINBOW,
    LED_EFFECT_CHASE,
    LED_EFFECT_SPARKLE,
    LED_EFFECT_WAVE,
    LED_EFFECT_COUNT,
} led_effect_t;

typedef struct {
    const char *label;
    uint8_t min_value;
    uint8_t max_value;
    uint8_t default_value;
} effect_param_spec_t;

typedef struct {
    uint8_t param_count;
    effect_param_spec_t params[kEffectParamSlotCount];
} effect_spec_t;

typedef struct {
    uint8_t values[kEffectParamSlotCount];
} effect_params_t;

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} effect_color_t;

typedef struct {
    uint16_t count;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t brightness;
    bool power;
    uint8_t effect;
    effect_params_t effect_profiles[LED_EFFECT_COUNT];
    effect_color_t effect_colors[LED_EFFECT_COUNT];
} led_state_t;

static const effect_spec_t kEffectSpecs[LED_EFFECT_COUNT] = {
    {0, {{"", 0, 0, 0}, {"", 0, 0, 0}, {"", 0, 0, 0}, {"", 0, 0, 0}, {"", 0, 0, 0}}},
    {3, {{"Pulse Speed", 1, 255, 140}, {"Glow Floor", 0, 255, 72}, {"Pulse Depth", 0, 255, 180}, {"", 0, 0, 0}, {"", 0, 0, 0}}},
    {5, {{"Drift Speed", 1, 255, 120}, {"Rainbow Length", 1, 255, 96}, {"Color Blend", 0, 255, 220}, {"Start Offset", 0, 255, 0}, {"Contrast", 0, 255, 96}}},
    {3, {{"Chase Speed", 1, 255, 175}, {"Tail Length", 1, 255, 90}, {"Tail Sharpness", 0, 255, 170}, {"", 0, 0, 0}, {"", 0, 0, 0}}},
    {3, {{"Spark Density", 1, 255, 180}, {"Base Glow", 0, 255, 60}, {"Twinkle Speed", 1, 255, 170}, {"", 0, 0, 0}, {"", 0, 0, 0}}},
    {3, {{"Wave Speed", 1, 255, 110}, {"Wavelength", 1, 255, 110}, {"Wave Depth", 0, 255, 190}, {"", 0, 0, 0}, {"", 0, 0, 0}}},
};

static led_strip_handle_t s_led_strip = nullptr;
static SemaphoreHandle_t s_state_mutex = nullptr;
static SemaphoreHandle_t s_ota_mutex = nullptr;
static httpd_handle_t s_http_server = nullptr;
static esp_netif_t *s_ap_netif = nullptr;
static led_state_t s_led_state = {
    APP_LED_DEFAULT_COUNT,
    APP_LED_DEFAULT_RED,
    APP_LED_DEFAULT_GREEN,
    APP_LED_DEFAULT_BLUE,
    APP_LED_DEFAULT_BRIGHT,
    APP_LED_DEFAULT_POWER != 0,
    LED_EFFECT_SOLID,
};
static uint16_t s_light_endpoint_id = 0;
static uint8_t s_matter_hue = 0;
static uint8_t s_matter_saturation = 0;
static uint16_t s_matter_x = kDefaultCurrentX;
static uint16_t s_matter_y = kDefaultCurrentY;
static uint16_t s_matter_temp_mireds = kDefaultColorTempMireds;
static bool s_syncing_matter = false;
static bool s_network_handlers_registered = false;
static char s_ap_ip[16] = "192.168.4.1";
static char s_sta_ip[16] = "";
static char s_ap_ssid[33] = APP_WIFI_SSID;
static char s_ap_password[65] = APP_WIFI_PASS;
static char s_runtime_ap_ssid[33] = APP_WIFI_SSID;
static char s_runtime_ap_password[65] = APP_WIFI_PASS;
static char s_matter_qr_code[APP_QR_CODE_MAX] = "";
static char s_matter_manual_code[APP_MANUAL_CODE_MAX] = "";
static char s_matter_qr_url[APP_QR_URL_MAX] = "";

static constexpr char INDEX_HTML[] = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-C6 Matter LED</title>
<style>
:root{--bg:#08111f;--bg2:#0f1d35;--card:#101b2fcc;--line:#2f4a7d;--text:#eef4ff;--muted:#9db2d7;--accent:#6ee7ff;--accent2:#ffaf45;--good:#8ef3b0;--warn:#ffd36e}
*{box-sizing:border-box}body{margin:0;font-family:Verdana,Segoe UI,sans-serif;color:var(--text);background:radial-gradient(circle at top left,#173057 0,#08111f 45%),linear-gradient(135deg,#08111f,#112748 60%,#1d4261);min-height:100vh}
.wrap{max-width:1100px;margin:0 auto;padding:24px}.hero{padding:24px 0 16px}.hero h1{margin:0;font-size:clamp(2rem,5vw,3.6rem);letter-spacing:.04em;text-transform:uppercase}.hero p{margin:12px 0 0;color:var(--muted);max-width:62rem;line-height:1.6}
.grid{display:grid;gap:18px;grid-template-columns:repeat(auto-fit,minmax(300px,1fr))}.card{background:var(--card);backdrop-filter:blur(12px);border:1px solid var(--line);border-radius:22px;padding:20px;box-shadow:0 20px 50px rgba(0,0,0,.24)}
.label{display:flex;justify-content:space-between;align-items:center;color:var(--muted);font-size:.95rem;margin-bottom:10px}.value{color:var(--text);font-weight:700}.swatch{height:128px;border-radius:18px;border:1px solid rgba(255,255,255,.15);background:#ff6020;box-shadow:inset 0 0 50px rgba(255,255,255,.18),0 0 24px rgba(255,120,80,.35);transition:all .18s ease}
input[type=range],input[type=number],input[type=color],input[type=text],input[type=password]{width:100%}input[type=range]{accent-color:var(--accent)}input[type=number],input[type=text],input[type=password]{background:#091223;border:1px solid var(--line);color:var(--text);border-radius:14px;padding:12px 14px;font-size:1rem}input[type=color]{height:54px;background:transparent;border:none;padding:0}
.row{display:grid;gap:14px;margin-top:16px}.chips{display:flex;flex-wrap:wrap;gap:10px;margin-top:14px}.chip{padding:10px 14px;border-radius:999px;background:#0b1730;border:1px solid var(--line);color:var(--muted);font-size:.92rem}
.actions{display:flex;gap:12px;flex-wrap:wrap;margin-top:18px}.btn{border:none;border-radius:16px;padding:14px 18px;font-weight:700;cursor:pointer;transition:transform .14s ease,opacity .14s ease}.btn:hover{transform:translateY(-1px)}.btn:disabled{opacity:.45;cursor:not-allowed;transform:none}.btn-primary{background:linear-gradient(135deg,var(--accent),var(--accent2));color:#07111e}.btn-secondary{background:#0b1730;color:var(--text);border:1px solid var(--line)}.btn-danger{background:#35131a;color:#ffd4da;border:1px solid #7b2a3a}
.toggle{display:flex;align-items:center;justify-content:space-between;background:#0b1730;border:1px solid var(--line);border-radius:16px;padding:14px 16px}.toggle input{width:22px;height:22px}
.status{margin-top:14px;min-height:24px;color:var(--good);font-weight:700}.footer{margin-top:18px;color:var(--muted);font-size:.92rem;line-height:1.6}.pairing{margin-top:16px;padding:16px;border-radius:18px;background:#091223;border:1px solid var(--line)}.pairing h2{margin:0 0 8px;font-size:1rem}.pairing p{margin:8px 0;color:var(--muted);line-height:1.5}.pairing code{display:block;padding:10px 12px;background:#050c17;border-radius:12px;color:var(--text);overflow:auto}.link{color:var(--accent);word-break:break-all}.stack{display:grid;gap:16px}
.tabs,.effect-tabs{display:flex;flex-wrap:wrap;gap:10px}.tab-btn{border:none;border-radius:999px;padding:12px 18px;font-weight:700;cursor:pointer;background:#0b1730;color:var(--muted);border:1px solid var(--line)}.tab-btn.active{background:linear-gradient(135deg,var(--accent),var(--accent2));color:#07111e}.panel{display:none;margin-top:18px}.panel.active{display:block}.kv{display:grid;gap:10px}.kv-line{display:flex;justify-content:space-between;gap:12px;padding:12px 14px;border-radius:14px;background:#091223;border:1px solid var(--line)}.kv-line span:first-child{color:var(--muted)}.kv-line span:last-child{text-align:right;word-break:break-word}
input[type=file]{width:100%;padding:12px 14px;background:#091223;border:1px dashed var(--line);color:var(--text);border-radius:14px}
@media (max-width:640px){.wrap{padding:18px}.card{padding:16px}.actions{flex-direction:column}.btn{width:100%}.kv-line{flex-direction:column}}
</style>
</head>
<body>
<div class="wrap">
<section class="hero">
<h1>ESP32-C6 LED Lab</h1>
<p>Manage Matter status, Wi-Fi setup, firmware actions, and WS2812B control from one page with clear tabs so risky actions stay separate from daily lighting control.</p>
</section>
<div class="tabs">
<button class="tab-btn active" data-main-tab="overview">Overview</button>
<button class="tab-btn" data-main-tab="configuration">Configuration</button>
<button class="tab-btn" data-main-tab="control">Control</button>
</div>

<section class="panel active" id="overviewPanel">
<div class="grid">
<div class="card">
<h2>Matter State</h2>
<div class="kv">
<div class="kv-line"><span>Status</span><span id="overviewMatterStatus">Loading...</span></div>
<div class="kv-line"><span>Endpoint</span><span id="overviewMatterEndpoint">-</span></div>
<div class="kv-line"><span>Manual Code</span><span id="overviewManualCode">-</span></div>
<div class="kv-line"><span>QR Link</span><span><a class="link" id="overviewQrLink" href="#" target="_blank" rel="noopener">Unavailable</a></span></div>
</div>
<div class="footer">If the device is not yet in Apple Home, use the manual code or QR link while the commissioning window is open.</div>
</div>
<div class="card">
<h2>Wi-Fi State</h2>
<div class="kv">
<div class="kv-line"><span>Active AP SSID</span><span id="overviewApSsid">-</span></div>
<div class="kv-line"><span>AP Web UI</span><span><a class="link" id="overviewApUrl" href="#" target="_blank" rel="noopener">Unavailable</a></span></div>
<div class="kv-line"><span>Station Status</span><span id="overviewStaStatus">-</span></div>
<div class="kv-line"><span>LAN Web UI</span><span><a class="link" id="overviewLanUrl" href="#" target="_blank" rel="noopener">Unavailable</a></span></div>
<div class="kv-line"><span>Restart Needed</span><span id="overviewApRestart">-</span></div>
</div>
</div>
<div class="card">
<h2>Firmware Version</h2>
<div class="kv">
<div class="kv-line"><span>Current Version</span><span id="overviewFwVersion">-</span></div>
<div class="kv-line"><span>Running Slot</span><span id="overviewRunningPartition">-</span></div>
<div class="kv-line"><span>Next OTA Slot</span><span id="overviewNextPartition">-</span></div>
<div class="kv-line"><span>Revert Target</span><span id="overviewRevertTarget">-</span></div>
</div>
</div>
</div>
</section>

<section class="panel" id="configurationPanel">
<div class="grid">
<div class="card">
<h2>Device Configuration</h2>
<div class="row">
<div>
<div class="label"><span>LED Count</span><span class="value" id="configCountValue">0</span></div>
<input id="configCount" type="range" min="1" max="120" step="1" value="8">
</div>
<div>
<div class="label"><span>Exact Count</span><span class="value">Numeric input</span></div>
<input id="configCountNumber" type="number" min="1" max="120" value="8">
</div>
<div>
<div class="label"><span>AP SSID</span><span class="value">1-32 chars</span></div>
<input id="configApSsid" type="text" maxlength="32" value="">
</div>
<div>
<div class="label"><span>AP Password</span><span class="value">8-63 chars</span></div>
<input id="configApPassword" type="password" maxlength="63" value="">
</div>
</div>
<div class="actions">
<button class="btn btn-primary" id="saveConfigBtn">Save Configuration</button>
<button class="btn btn-secondary" id="rebootBtn">Reset</button>
</div>
<div class="status" id="configStatus"></div>
<div class="footer">New AP credentials are saved immediately but become active after a reset.</div>
</div>
<div class="card">
<h2>Firmware Actions</h2>
<div class="pairing">
<h2>Firmware Update</h2>
<p>Upload a new application binary from this project to install it into the inactive OTA slot. The device will reboot automatically after a successful update.</p>
<input id="otaFile" type="file" accept=".bin,application/octet-stream">
<div class="actions">
<button class="btn btn-secondary" id="otaBtn">Install OTA Update</button>
</div>
<p class="footer" id="otaStatus">Use <code>build/esp32c6_led_web.bin</code> after the first USB flash.</p>
</div>
<div class="actions">
<button class="btn btn-secondary" id="revertBtn">Revert To Previous Firmware</button>
<button class="btn btn-danger" id="factoryResetBtn">Factory Reset</button>
</div>
<div class="status" id="actionStatus"></div>
</div>
</div>
</section>

<section class="panel" id="controlPanel">
<div class="card">
<div class="row">
<div>
<div class="label"><span>Brightness</span><span class="value" id="brightnessValue">0</span></div>
<input id="controlBrightness" type="range" min="0" max="255" step="1" value="96">
</div>
<div>
<div class="label"><span>Color</span><span class="value" id="controlColorValue">#FF6020</span></div>
<input id="controlColor" type="color" value="#ff6020">
</div>
</div>
<div class="row">
<div id="effectColorRow" style="display:none">
<div class="label"><span id="effectColorLabel">Effect Color</span><span class="value" id="effectColorValue">#FFFFFF</span></div>
<input id="effectColor" type="color" value="#ffffff">
</div>
</div>
<div class="effect-tabs">
<button class="tab-btn active" data-effect-tab="solid">Solid</button>
<button class="tab-btn" data-effect-tab="glow">Glow</button>
<button class="tab-btn" data-effect-tab="rainbow">Rainbow</button>
<button class="tab-btn" data-effect-tab="chase">Chase</button>
<button class="tab-btn" data-effect-tab="sparkle">Sparkle</button>
<button class="tab-btn" data-effect-tab="wave">Wave</button>
</div>
<div class="row">
<div id="effectParamRow0">
<div class="label"><span id="effectParamLabel0">Param 1</span><span class="value" id="effectParamValue0">0</span></div>
<input id="effectParamInput0" type="range" min="0" max="255" step="1" value="0">
</div>
<div id="effectParamRow1">
<div class="label"><span id="effectParamLabel1">Param 2</span><span class="value" id="effectParamValue1">0</span></div>
<input id="effectParamInput1" type="range" min="0" max="255" step="1" value="0">
</div>
<div id="effectParamRow2">
<div class="label"><span id="effectParamLabel2">Param 3</span><span class="value" id="effectParamValue2">0</span></div>
<input id="effectParamInput2" type="range" min="0" max="255" step="1" value="0">
</div>
<div id="effectParamRow3">
<div class="label"><span id="effectParamLabel3">Param 4</span><span class="value" id="effectParamValue3">0</span></div>
<input id="effectParamInput3" type="range" min="0" max="255" step="1" value="0">
</div>
<div id="effectParamRow4">
<div class="label"><span id="effectParamLabel4">Param 5</span><span class="value" id="effectParamValue4">0</span></div>
<input id="effectParamInput4" type="range" min="0" max="255" step="1" value="0">
</div>
</div>
<div class="actions">
<button class="btn btn-primary" id="saveControlBtn">Apply Control</button>
<button class="btn btn-secondary" id="reloadBtn">Reload Device State</button>
</div>
<div class="status" id="controlStatus"></div>
<div class="footer">Applying control turns the strip on when brightness is above zero. Set brightness to zero to keep it dark.</div>
</div>
</section>
</div>
<script>
const mainTabButtons = Array.from(document.querySelectorAll('[data-main-tab]'));
const panels = {overview:document.getElementById('overviewPanel'),configuration:document.getElementById('configurationPanel'),control:document.getElementById('controlPanel')};
const configCount = document.getElementById('configCount');
const configCountNumber = document.getElementById('configCountNumber');
const configCountValue = document.getElementById('configCountValue');
const configApSsid = document.getElementById('configApSsid');
const configApPassword = document.getElementById('configApPassword');
const controlBrightness = document.getElementById('controlBrightness');
const controlColor = document.getElementById('controlColor');
const controlColorValue = document.getElementById('controlColorValue');
const effectColorRow = document.getElementById('effectColorRow');
const effectColor = document.getElementById('effectColor');
const effectColorLabel = document.getElementById('effectColorLabel');
const effectColorValue = document.getElementById('effectColorValue');
const controlStatus = document.getElementById('controlStatus');
const configStatus = document.getElementById('configStatus');
const actionStatus = document.getElementById('actionStatus');
const brightnessValue = document.getElementById('brightnessValue');
const otaFile = document.getElementById('otaFile');
const otaBtn = document.getElementById('otaBtn');
const otaStatus = document.getElementById('otaStatus');
const overviewMatterStatus = document.getElementById('overviewMatterStatus');
const overviewMatterEndpoint = document.getElementById('overviewMatterEndpoint');
const overviewManualCode = document.getElementById('overviewManualCode');
const overviewQrLink = document.getElementById('overviewQrLink');
const overviewApSsid = document.getElementById('overviewApSsid');
const overviewApUrl = document.getElementById('overviewApUrl');
const overviewStaStatus = document.getElementById('overviewStaStatus');
const overviewLanUrl = document.getElementById('overviewLanUrl');
const overviewApRestart = document.getElementById('overviewApRestart');
const overviewFwVersion = document.getElementById('overviewFwVersion');
const overviewRunningPartition = document.getElementById('overviewRunningPartition');
const overviewNextPartition = document.getElementById('overviewNextPartition');
const overviewRevertTarget = document.getElementById('overviewRevertTarget');
const saveConfigBtn = document.getElementById('saveConfigBtn');
const saveControlBtn = document.getElementById('saveControlBtn');
const rebootBtn = document.getElementById('rebootBtn');
const revertBtn = document.getElementById('revertBtn');
const factoryResetBtn = document.getElementById('factoryResetBtn');
const effectTabButtons = Array.from(document.querySelectorAll('[data-effect-tab]'));
const effectParamRows = [0,1,2,3,4].map((index)=>({row:document.getElementById('effectParamRow'+index),label:document.getElementById('effectParamLabel'+index),value:document.getElementById('effectParamValue'+index),input:document.getElementById('effectParamInput'+index)}));
const EFFECT_META = {
solid:{params:[],colors:[]},
glow:{params:[{label:'Pulse Speed',min:1,max:255,defaultValue:140},{label:'Glow Floor',min:0,max:255,defaultValue:72},{label:'Pulse Depth',min:0,max:255,defaultValue:180}],colors:[]},
rainbow:{params:[{label:'Drift Speed',min:1,max:255,defaultValue:120},{label:'Rainbow Length',min:1,max:255,defaultValue:96},{label:'Color Blend',min:0,max:255,defaultValue:220},{label:'Start Offset',min:0,max:255,defaultValue:0},{label:'Contrast',min:0,max:255,defaultValue:96}],colors:[]},
chase:{params:[{label:'Chase Speed',min:1,max:255,defaultValue:175},{label:'Tail Length',min:1,max:255,defaultValue:90},{label:'Tail Sharpness',min:0,max:255,defaultValue:170}],colors:[]},
sparkle:{params:[{label:'Spark Density',min:1,max:255,defaultValue:180},{label:'Base Glow',min:0,max:255,defaultValue:60},{label:'Twinkle Speed',min:1,max:255,defaultValue:170}],colors:[{label:'Sparkle Color',defaultValue:'#FFFFFF'}]},
wave:{params:[{label:'Wave Speed',min:1,max:255,defaultValue:110},{label:'Wavelength',min:1,max:255,defaultValue:110},{label:'Wave Depth',min:0,max:255,defaultValue:190}],colors:[]}
};
let effectProfiles = {};
let effectColors = {};
let selectedEffect = 'solid';
function switchMainTab(name){mainTabButtons.forEach((button)=>button.classList.toggle('active',button.dataset.mainTab===name));Object.entries(panels).forEach(([panelName,panel])=>panel.classList.toggle('active',panelName===name))}
function buildDefaultEffectProfiles(){const profiles={};for(const [name,meta] of Object.entries(EFFECT_META)){profiles[name]=[0,0,0,0,0];meta.params.forEach((param,index)=>{profiles[name][index]=param.defaultValue})}return profiles}
function buildDefaultEffectColors(){const colors={};for(const [name,meta] of Object.entries(EFFECT_META)){colors[name]=(meta.colors&&meta.colors[0]?meta.colors[0].defaultValue:'#FFFFFF').toUpperCase()}return colors}
function normalizeEffectProfiles(rawProfiles){const profiles=buildDefaultEffectProfiles();for(const [name,values] of Object.entries(rawProfiles||{})){if(!profiles[name]||!Array.isArray(values))continue;values.forEach((value,index)=>{const meta=EFFECT_META[name].params[index];if(!meta)return;const parsed=Number(value);if(Number.isFinite(parsed)){profiles[name][index]=Math.max(meta.min,Math.min(meta.max,parsed))}})}return profiles}
function normalizeHexColor(value,fallback){return typeof value==='string'&&/^#[0-9a-fA-F]{6}$/.test(value)?value.toUpperCase():fallback}
function normalizeEffectColors(rawColors){const colors=buildDefaultEffectColors();for(const [name,value] of Object.entries(rawColors||{})){if(!(name in colors))continue;colors[name]=normalizeHexColor(value,colors[name])}return colors}
function syncConfigCount(v){configCount.value=v;configCountNumber.value=v;configCountValue.textContent=v}
function getSelectedEffectValues(){if(!effectProfiles[selectedEffect]){effectProfiles[selectedEffect]=buildDefaultEffectProfiles()[selectedEffect]||[0,0,0,0,0]}return effectProfiles[selectedEffect]}
function getSelectedEffectColor(){if(!(selectedEffect in effectColors)){effectColors[selectedEffect]=buildDefaultEffectColors()[selectedEffect]||'#FFFFFF'}return effectColors[selectedEffect]}
function renderEffectButtons(){effectTabButtons.forEach((button)=>button.classList.toggle('active',button.dataset.effectTab===selectedEffect))}
function syncEffectControls(){const meta=EFFECT_META[selectedEffect]||EFFECT_META.solid;const values=getSelectedEffectValues();effectParamRows.forEach((slot,index)=>{const spec=meta.params[index];if(!spec){slot.row.style.display='none';return}slot.row.style.display='block';slot.label.textContent=spec.label;slot.input.min=spec.min;slot.input.max=spec.max;slot.input.value=values[index];slot.value.textContent=values[index]});const colorSpec=(meta.colors||[])[0];if(!colorSpec){effectColorRow.style.display='none'}else{effectColorRow.style.display='block';effectColorLabel.textContent=colorSpec.label;effectColor.value=getSelectedEffectColor().toLowerCase();effectColorValue.textContent=getSelectedEffectColor().toUpperCase()}renderEffectButtons()}
function stashEffectControls(){const meta=EFFECT_META[selectedEffect]||EFFECT_META.solid;const values=getSelectedEffectValues();effectParamRows.forEach((slot,index)=>{if(!meta.params[index]){values[index]=0;return}values[index]=Number(slot.input.value);slot.value.textContent=slot.input.value});if((meta.colors||[])[0]){effectColors[selectedEffect]=normalizeHexColor(effectColor.value,getSelectedEffectColor())}}
function updateControlReadout(){brightnessValue.textContent=controlBrightness.value;controlColorValue.textContent=controlColor.value.toUpperCase();if(effectColorRow.style.display!=='none'){effectColorValue.textContent=effectColor.value.toUpperCase()}}
function setLink(linkEl,url,emptyLabel){if(url){linkEl.href=url;linkEl.textContent=url}else{linkEl.href='#';linkEl.textContent=emptyLabel||'Unavailable'}}
function refreshOverview(data){overviewMatterStatus.textContent=data.commissioned?'Commissioned':'Ready to pair';overviewMatterEndpoint.textContent=data.matter_endpoint;overviewManualCode.textContent=data.manual_code||'Unavailable';setLink(overviewQrLink,data.qr_url,'Unavailable');overviewApSsid.textContent=data.ap_ssid||'-';setLink(overviewApUrl,data.ap_url||(data.ap_ip?('http://'+data.ap_ip):''),'Unavailable');overviewStaStatus.textContent=data.sta_connected?'Connected':'Not connected';setLink(overviewLanUrl,data.lan_url||(data.sta_ip?('http://'+data.sta_ip):''),'Not connected');overviewApRestart.textContent=data.ap_restart_required?'Yes, reset to apply new AP config':'No';overviewFwVersion.textContent=data.fw_version||'unknown';overviewRunningPartition.textContent=data.running_partition||'-';overviewNextPartition.textContent=data.ota_target_partition||'-';overviewRevertTarget.textContent=data.revert_available?((data.revert_version||'unknown')+' @ '+(data.revert_partition||'')):'No previous firmware available'}
function setOtaBusy(busy){otaBtn.disabled=busy;otaFile.disabled=busy;otaBtn.textContent=busy?'Uploading OTA...':'Install OTA Update'}
function applyStateToUi(data){effectProfiles=normalizeEffectProfiles(data.effect_profiles);effectColors=normalizeEffectColors(data.effect_colors);selectedEffect=data.effect||'solid';syncConfigCount(data.count);configCount.max=data.max_leds;configCountNumber.max=data.max_leds;configApSsid.value=data.config_ap_ssid||data.ap_ssid||'';configApPassword.value=data.config_ap_password||'';controlBrightness.value=data.brightness;controlColor.value=data.color;revertBtn.disabled=!data.revert_available;refreshOverview(data);otaStatus.textContent='Next OTA slot: '+(data.ota_target_partition||'unknown')+'. Upload build/esp32c6_led_web.bin after the first USB flash.';syncEffectControls();updateControlReadout()}
async function loadState(){controlStatus.textContent='Loading device state...';const res=await fetch('/api/state');if(!res.ok)throw new Error('Failed to load state');const data=await res.json();applyStateToUi(data);controlStatus.textContent='Device state loaded';configStatus.textContent=data.ap_restart_required?'Saved AP config is waiting for a reset.':'Configuration loaded'}
async function saveControl(){controlStatus.textContent='Applying control...';stashEffectControls();const brightness=Number(controlBrightness.value);const payload={brightness:brightness,color:controlColor.value,effect:selectedEffect,effect_params:getSelectedEffectValues(),power:brightness>0};if((EFFECT_META[selectedEffect].colors||[])[0]){payload.effect_color=getSelectedEffectColor()}const res=await fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});if(!res.ok)throw new Error(await res.text()||'Failed to apply control');const data=await res.json();applyStateToUi(data);controlStatus.textContent='Control saved'}
async function saveConfig(){configStatus.textContent='Saving configuration...';const payload={count:Number(configCount.value),ap_ssid:configApSsid.value.trim(),ap_password:configApPassword.value};const res=await fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});if(!res.ok)throw new Error(await res.text()||'Failed to save configuration');const data=await res.json();applyStateToUi(data);configStatus.textContent=data.ap_restart_required?'Configuration saved. Press Reset to apply the new AP credentials.':'Configuration saved'}
async function uploadOta(){const file=otaFile.files&&otaFile.files[0];if(!file)throw new Error('Choose a firmware .bin file first');setOtaBusy(true);otaStatus.textContent='Uploading '+file.name+' ('+file.size+' bytes)...';const res=await fetch('/api/ota',{method:'POST',headers:{'Content-Type':'application/octet-stream','X-Filename':file.name},body:file});const text=await res.text();let data={message:text};try{data=JSON.parse(text)}catch(_){ }if(!res.ok)throw new Error(data.message||text||'OTA update failed');otaStatus.textContent=data.message||'Update installed. Device will restart.';actionStatus.textContent='OTA accepted. Reconnect after reboot.';setTimeout(()=>window.location.reload(),12000)}
async function postAction(url,statusEl,confirmText){if(confirmText&&!window.confirm(confirmText))return;statusEl.textContent='Sending command...';const res=await fetch(url,{method:'POST'});const text=await res.text();let data={message:text};try{data=JSON.parse(text)}catch(_){ }if(!res.ok)throw new Error(data.message||text||'Action failed');statusEl.textContent=data.message||'Command sent';setTimeout(()=>window.location.reload(),12000)}
mainTabButtons.forEach((button)=>button.addEventListener('click',()=>switchMainTab(button.dataset.mainTab)));
configCount.addEventListener('input',()=>syncConfigCount(configCount.value));
configCountNumber.addEventListener('input',()=>{const max=Number(configCount.max);let v=Number(configCountNumber.value||1);v=Math.max(1,Math.min(max,v));syncConfigCount(v)});
controlBrightness.addEventListener('input',updateControlReadout);controlColor.addEventListener('input',updateControlReadout);
effectTabButtons.forEach((button)=>button.addEventListener('click',()=>{stashEffectControls();selectedEffect=button.dataset.effectTab;syncEffectControls();updateControlReadout()}));
effectParamRows.forEach((slot)=>slot.input.addEventListener('input',()=>{stashEffectControls();updateControlReadout()}));
effectColor.addEventListener('input',()=>{effectColors[selectedEffect]=normalizeHexColor(effectColor.value,getSelectedEffectColor());updateControlReadout()});
saveControlBtn.addEventListener('click',()=>saveControl().catch(err=>controlStatus.textContent=err.message));
saveConfigBtn.addEventListener('click',()=>saveConfig().catch(err=>configStatus.textContent=err.message));
document.getElementById('reloadBtn').addEventListener('click',()=>loadState().catch(err=>controlStatus.textContent=err.message));
otaBtn.addEventListener('click',()=>uploadOta().catch(err=>{otaStatus.textContent=err.message;setOtaBusy(false)}));
rebootBtn.addEventListener('click',()=>postAction('/api/reboot',configStatus,'Reset the device now?').catch(err=>configStatus.textContent=err.message));
revertBtn.addEventListener('click',()=>postAction('/api/revert',actionStatus,'Revert to the previous firmware slot and reboot?').catch(err=>actionStatus.textContent=err.message));
factoryResetBtn.addEventListener('click',()=>postAction('/api/factory-reset',actionStatus,'Factory reset will erase Matter pairing, Wi-Fi AP config, and saved LED settings. Continue?').catch(err=>actionStatus.textContent=err.message));
effectProfiles=buildDefaultEffectProfiles();effectColors=buildDefaultEffectColors();syncEffectControls();loadState().catch(err=>{controlStatus.textContent=err.message;configStatus.textContent=err.message;updateControlReadout()});
</script>
</body>
</html>
)HTML";

static inline uint8_t clamp_u8(int value)
{
    return static_cast<uint8_t>(std::clamp(value, 0, 255));
}

static inline uint16_t clamp_u16(int value, int min_value, int max_value)
{
    return static_cast<uint16_t>(std::clamp(value, min_value, max_value));
}

static inline double normalized_u8(uint8_t value)
{
    return static_cast<double>(value) / 255.0;
}

static inline uint8_t effect_from_index(int value)
{
    return static_cast<uint8_t>(std::clamp(value, 0, static_cast<int>(LED_EFFECT_COUNT) - 1));
}

static const effect_spec_t &get_effect_spec(uint8_t effect)
{
    return kEffectSpecs[effect_from_index(effect)];
}

static const char *effect_to_name(uint8_t effect)
{
    switch (effect) {
    case LED_EFFECT_GLOW:
        return "glow";
    case LED_EFFECT_RAINBOW:
        return "rainbow";
    case LED_EFFECT_CHASE:
        return "chase";
    case LED_EFFECT_SPARKLE:
        return "sparkle";
    case LED_EFFECT_WAVE:
        return "wave";
    case LED_EFFECT_SOLID:
    default:
        return "solid";
    }
}

static uint8_t effect_from_name(const char *effect_name)
{
    if (!effect_name) {
        return LED_EFFECT_SOLID;
    }
    if (std::strcmp(effect_name, "glow") == 0) {
        return LED_EFFECT_GLOW;
    }
    if (std::strcmp(effect_name, "rainbow") == 0) {
        return LED_EFFECT_RAINBOW;
    }
    if (std::strcmp(effect_name, "chase") == 0) {
        return LED_EFFECT_CHASE;
    }
    if (std::strcmp(effect_name, "sparkle") == 0) {
        return LED_EFFECT_SPARKLE;
    }
    if (std::strcmp(effect_name, "wave") == 0) {
        return LED_EFFECT_WAVE;
    }
    return LED_EFFECT_SOLID;
}

static void copy_string_value(char *dest, size_t dest_size, const char *src)
{
    if (!dest || dest_size == 0) {
        return;
    }

    if (!src) {
        dest[0] = '\0';
        return;
    }

    size_t copy_len = std::min(dest_size - 1, std::strlen(src));
    std::memcpy(dest, src, copy_len);
    dest[copy_len] = '\0';
}

static bool matter_is_ready()
{
    return esp_matter::is_started() && s_light_endpoint_id != 0;
}

static void reset_effect_profiles_to_defaults(led_state_t *state)
{
    if (!state) {
        return;
    }

    for (uint8_t effect = 0; effect < LED_EFFECT_COUNT; ++effect) {
        const effect_spec_t &spec = get_effect_spec(effect);
        for (size_t index = 0; index < kEffectParamSlotCount; ++index) {
            state->effect_profiles[effect].values[index] = index < spec.param_count ? spec.params[index].default_value : 0;
        }
    }
}

static void reset_effect_colors_to_defaults(led_state_t *state)
{
    if (!state) {
        return;
    }

    for (uint8_t effect = 0; effect < LED_EFFECT_COUNT; ++effect) {
        state->effect_colors[effect].red = APP_LED_DEFAULT_RED;
        state->effect_colors[effect].green = APP_LED_DEFAULT_GREEN;
        state->effect_colors[effect].blue = APP_LED_DEFAULT_BLUE;
    }

    state->effect_colors[LED_EFFECT_SPARKLE].red = 255;
    state->effect_colors[LED_EFFECT_SPARKLE].green = 255;
    state->effect_colors[LED_EFFECT_SPARKLE].blue = 255;
}

static void clamp_effect_profile(uint8_t effect, effect_params_t *profile)
{
    if (!profile) {
        return;
    }

    const effect_spec_t &spec = get_effect_spec(effect);
    for (size_t index = 0; index < kEffectParamSlotCount; ++index) {
        if (index < spec.param_count) {
            profile->values[index] = static_cast<uint8_t>(
                std::clamp(static_cast<int>(profile->values[index]),
                           static_cast<int>(spec.params[index].min_value),
                           static_cast<int>(spec.params[index].max_value)));
        } else {
            profile->values[index] = 0;
        }
    }
}

static bool ap_config_restart_required()
{
    return std::strcmp(s_ap_ssid, s_runtime_ap_ssid) != 0 || std::strcmp(s_ap_password, s_runtime_ap_password) != 0;
}

static bool validate_ap_credentials(const char *ssid, const char *password)
{
    if (!ssid || !password) {
        return false;
    }

    size_t ssid_len = std::strlen(ssid);
    size_t pass_len = std::strlen(password);
    if (ssid_len == 0 || ssid_len > 32) {
        return false;
    }

    if (pass_len < 8 || pass_len > 63) {
        return false;
    }

    return true;
}

static void clamp_state(led_state_t *state)
{
    if (!state) {
        return;
    }
    state->count = clamp_u16(state->count, 1, APP_LED_MAX_PIXELS);
    state->effect = effect_from_index(state->effect);
    for (uint8_t effect = 0; effect < LED_EFFECT_COUNT; ++effect) {
        clamp_effect_profile(effect, &state->effect_profiles[effect]);
    }
}

static uint8_t brightness_to_matter_level(uint8_t brightness)
{
    return static_cast<uint8_t>((static_cast<uint32_t>(brightness) * 254 + 127) / 255);
}

static uint8_t matter_level_to_brightness(uint8_t level)
{
    return static_cast<uint8_t>((static_cast<uint32_t>(level) * 255 + 127) / 254);
}

static void rgb_to_matter_hs(uint8_t red, uint8_t green, uint8_t blue, uint8_t *matter_hue, uint8_t *matter_saturation)
{
    double rf = static_cast<double>(red) / 255.0;
    double gf = static_cast<double>(green) / 255.0;
    double bf = static_cast<double>(blue) / 255.0;
    double max_value = std::max({rf, gf, bf});
    double min_value = std::min({rf, gf, bf});
    double delta = max_value - min_value;
    double hue = 0.0;

    if (delta > 0.0) {
        if (max_value == rf) {
            hue = 60.0 * std::fmod(((gf - bf) / delta), 6.0);
        } else if (max_value == gf) {
            hue = 60.0 * (((bf - rf) / delta) + 2.0);
        } else {
            hue = 60.0 * (((rf - gf) / delta) + 4.0);
        }
    }

    if (hue < 0.0) {
        hue += 360.0;
    }

    double saturation = max_value <= 0.0 ? 0.0 : (delta / max_value);
    if (matter_hue) {
        *matter_hue = clamp_u8(static_cast<int>(std::lround((hue / 360.0) * 254.0)));
    }
    if (matter_saturation) {
        *matter_saturation = clamp_u8(static_cast<int>(std::lround(saturation * 254.0)));
    }
}

static void matter_hs_to_rgb(uint8_t matter_hue, uint8_t matter_saturation, uint8_t *red, uint8_t *green, uint8_t *blue)
{
    double hue = (static_cast<double>(matter_hue) / 254.0) * 360.0;
    double saturation = static_cast<double>(matter_saturation) / 254.0;
    double value = 1.0;
    double chroma = value * saturation;
    double x = chroma * (1.0 - std::fabs(std::fmod(hue / 60.0, 2.0) - 1.0));
    double match = value - chroma;
    double rf = 0.0;
    double gf = 0.0;
    double bf = 0.0;

    if (hue < 60.0) {
        rf = chroma;
        gf = x;
    } else if (hue < 120.0) {
        rf = x;
        gf = chroma;
    } else if (hue < 180.0) {
        gf = chroma;
        bf = x;
    } else if (hue < 240.0) {
        gf = x;
        bf = chroma;
    } else if (hue < 300.0) {
        rf = x;
        bf = chroma;
    } else {
        rf = chroma;
        bf = x;
    }

    if (red) {
        *red = clamp_u8(static_cast<int>(std::lround((rf + match) * 255.0)));
    }
    if (green) {
        *green = clamp_u8(static_cast<int>(std::lround((gf + match) * 255.0)));
    }
    if (blue) {
        *blue = clamp_u8(static_cast<int>(std::lround((bf + match) * 255.0)));
    }
}

static void matter_xy_to_rgb(uint16_t current_x, uint16_t current_y, uint8_t *red, uint8_t *green, uint8_t *blue)
{
    double x = static_cast<double>(current_x) / 65535.0;
    double y = static_cast<double>(current_y) / 65535.0;
    if (y <= 0.0001) {
        if (red) {
            *red = 255;
        }
        if (green) {
            *green = 255;
        }
        if (blue) {
            *blue = 255;
        }
        return;
    }

    double z = std::max(0.0, 1.0 - x - y);
    double luminance = 1.0;
    double X = (luminance / y) * x;
    double Y = luminance;
    double Z = (luminance / y) * z;

    double rf = X * 1.656492 - Y * 0.354851 - Z * 0.255038;
    double gf = -X * 0.707196 + Y * 1.655397 + Z * 0.036152;
    double bf = X * 0.051713 - Y * 0.121364 + Z * 1.011530;

    rf = std::max(0.0, rf);
    gf = std::max(0.0, gf);
    bf = std::max(0.0, bf);

    auto gamma_correct = [](double value) {
        if (value <= 0.0031308) {
            return 12.92 * value;
        }
        return 1.055 * std::pow(value, 1.0 / 2.4) - 0.055;
    };

    rf = gamma_correct(rf);
    gf = gamma_correct(gf);
    bf = gamma_correct(bf);

    double max_value = std::max({rf, gf, bf});
    if (max_value > 1.0) {
        rf /= max_value;
        gf /= max_value;
        bf /= max_value;
    }

    if (red) {
        *red = clamp_u8(static_cast<int>(std::lround(rf * 255.0)));
    }
    if (green) {
        *green = clamp_u8(static_cast<int>(std::lround(gf * 255.0)));
    }
    if (blue) {
        *blue = clamp_u8(static_cast<int>(std::lround(bf * 255.0)));
    }
}

static void color_temp_to_rgb(uint16_t mireds, uint8_t *red, uint8_t *green, uint8_t *blue)
{
    double kelvin = 1000000.0 / std::max<uint16_t>(mireds, 1);
    double temp = std::clamp(kelvin / 100.0, 10.0, 400.0);

    double rf;
    double gf;
    double bf;

    if (temp <= 66.0) {
        rf = 255.0;
        gf = 99.4708025861 * std::log(temp) - 161.1195681661;
        if (temp <= 19.0) {
            bf = 0.0;
        } else {
            bf = 138.5177312231 * std::log(temp - 10.0) - 305.0447927307;
        }
    } else {
        rf = 329.698727446 * std::pow(temp - 60.0, -0.1332047592);
        gf = 288.1221695283 * std::pow(temp - 60.0, -0.0755148492);
        bf = 255.0;
    }

    if (red) {
        *red = clamp_u8(static_cast<int>(std::lround(rf)));
    }
    if (green) {
        *green = clamp_u8(static_cast<int>(std::lround(gf)));
    }
    if (blue) {
        *blue = clamp_u8(static_cast<int>(std::lround(bf)));
    }
}

static void refresh_matter_hs_trackers_from_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    rgb_to_matter_hs(red, green, blue, &s_matter_hue, &s_matter_saturation);
}

static uint32_t pseudo_random_u32(uint32_t value)
{
    value ^= value >> 16;
    value *= 0x7feb352dU;
    value ^= value >> 15;
    value *= 0x846ca68bU;
    value ^= value >> 16;
    return value;
}

static uint8_t wheel_channel(uint8_t wheel_pos, uint8_t channel)
{
    if (wheel_pos < 85) {
        return channel == 0 ? static_cast<uint8_t>(255 - wheel_pos * 3)
                            : (channel == 1 ? static_cast<uint8_t>(wheel_pos * 3) : 0);
    }
    if (wheel_pos < 170) {
        wheel_pos = static_cast<uint8_t>(wheel_pos - 85);
        return channel == 1 ? static_cast<uint8_t>(255 - wheel_pos * 3)
                            : (channel == 2 ? static_cast<uint8_t>(wheel_pos * 3) : 0);
    }
    wheel_pos = static_cast<uint8_t>(wheel_pos - 170);
    return channel == 2 ? static_cast<uint8_t>(255 - wheel_pos * 3)
                        : (channel == 0 ? static_cast<uint8_t>(wheel_pos * 3) : 0);
}

static uint8_t float_to_u8(double value)
{
    return clamp_u8(static_cast<int>(std::lround(std::clamp(value, 0.0, 255.0))));
}

static uint32_t effect_cycle_ms_from_value(uint8_t value, uint32_t slow_ms, uint32_t fast_ms)
{
    double speed = normalized_u8(std::max<uint8_t>(1, value));
    double interpolated = static_cast<double>(slow_ms) - (static_cast<double>(slow_ms - fast_ms) * speed);
    return std::max<uint32_t>(fast_ms, static_cast<uint32_t>(std::lround(interpolated)));
}

static void render_effect_pixel(const led_state_t *state, uint16_t index, uint32_t now_ms,
                                uint8_t *out_red, uint8_t *out_green, uint8_t *out_blue)
{
    if (!state || !out_red || !out_green || !out_blue || !state->power || state->brightness == 0 || index >= state->count) {
        if (out_red) {
            *out_red = 0;
        }
        if (out_green) {
            *out_green = 0;
        }
        if (out_blue) {
            *out_blue = 0;
        }
        return;
    }

    double brightness_scale = static_cast<double>(state->brightness) / 255.0;
    double red = static_cast<double>(state->red);
    double green = static_cast<double>(state->green);
    double blue = static_cast<double>(state->blue);
    uint16_t active_count = std::max<uint16_t>(state->count, 1);
    const effect_params_t &params = state->effect_profiles[state->effect];

    switch (state->effect) {
    case LED_EFFECT_GLOW: {
        double floor = 0.03 + normalized_u8(params.values[1]) * 0.62;
        double depth = 0.12 + normalized_u8(params.values[2]) * 0.88;
        uint32_t cycle = effect_cycle_ms_from_value(params.values[0], 3200U, 600U);
        double phase = (static_cast<double>(now_ms % cycle) / static_cast<double>(cycle)) * 2.0 * M_PI;
        double pulse = floor + (1.0 - floor) * (((std::sin(phase) + 1.0) * 0.5) * depth);
        brightness_scale *= pulse;
        break;
    }
    case LED_EFFECT_RAINBOW: {
        uint32_t cycle = effect_cycle_ms_from_value(params.values[0], 6000U, 450U);
        double length_leds = 2.0 + normalized_u8(params.values[1]) * std::max<double>(8.0, static_cast<double>(active_count) * 3.0);
        double travel_leds = (static_cast<double>(now_ms % cycle) / static_cast<double>(cycle)) * length_leds;
        double start_offset = normalized_u8(params.values[3]) * 255.0;
        double wheel_position = std::fmod((((static_cast<double>(index) + travel_leds) / length_leds) * 255.0) + start_offset, 256.0);
        if (wheel_position < 0.0) {
            wheel_position += 256.0;
        }
        uint8_t wheel_pos = static_cast<uint8_t>(wheel_position);
        double rainbow_mix = normalized_u8(params.values[2]);
        double contrast = normalized_u8(params.values[4]);
        double wave_phase = (((static_cast<double>(index) + travel_leds) / length_leds) * 2.0 * M_PI) +
                            (normalized_u8(params.values[3]) * 2.0 * M_PI);
        double rainbow_wave = 0.5 + 0.5 * std::sin(wave_phase);
        double contrast_scale = (1.0 - contrast) + contrast * rainbow_wave;
        red = red * (1.0 - rainbow_mix) + wheel_channel(wheel_pos, 0) * rainbow_mix;
        green = green * (1.0 - rainbow_mix) + wheel_channel(wheel_pos, 1) * rainbow_mix;
        blue = blue * (1.0 - rainbow_mix) + wheel_channel(wheel_pos, 2) * rainbow_mix;
        brightness_scale *= 0.3 + contrast_scale * 0.7;
        break;
    }
    case LED_EFFECT_CHASE: {
        uint32_t interval = effect_cycle_ms_from_value(params.values[0], 260U, 35U);
        uint32_t step = now_ms / interval;
        uint16_t head = static_cast<uint16_t>(step % active_count);
        uint16_t distance = static_cast<uint16_t>((index + active_count - head) % active_count);
        uint8_t tail_len = static_cast<uint8_t>(1 + std::lround(normalized_u8(params.values[1]) * 12.0));
        double sharpness = 0.5 + normalized_u8(params.values[2]) * 3.5;
        double trail = 0.0;
        if (distance < tail_len) {
            double falloff = 1.0 - (static_cast<double>(distance) / std::max<uint8_t>(1, tail_len));
            trail = std::pow(falloff, sharpness);
        }
        brightness_scale *= trail;
        break;
    }
    case LED_EFFECT_SPARKLE: {
        double density = normalized_u8(params.values[0]);
        double base = normalized_u8(params.values[1]) * 0.35;
        uint32_t interval = effect_cycle_ms_from_value(params.values[2], 180U, 25U);
        uint32_t phase = now_ms / interval;
        uint32_t random = pseudo_random_u32(index * 2654435761U + phase * 2246822519U);
        uint32_t sparkle_mask = std::max<uint32_t>(1, 127U - static_cast<uint32_t>(std::lround(density * 118.0)));
        if ((random & sparkle_mask) == 0) {
            red = static_cast<double>(state->effect_colors[LED_EFFECT_SPARKLE].red);
            green = static_cast<double>(state->effect_colors[LED_EFFECT_SPARKLE].green);
            blue = static_cast<double>(state->effect_colors[LED_EFFECT_SPARKLE].blue);
            brightness_scale *= 1.0;
        } else {
            brightness_scale *= base;
        }
        break;
    }
    case LED_EFFECT_WAVE: {
        uint32_t cycle = effect_cycle_ms_from_value(params.values[0], 3600U, 550U);
        double phase = (static_cast<double>(now_ms % cycle) / static_cast<double>(cycle)) * 2.0 * M_PI;
        double wavelength = 0.7 + normalized_u8(params.values[1]) * 6.3;
        double position = (static_cast<double>(index) / active_count) * 2.0 * M_PI * wavelength;
        double depth = normalized_u8(params.values[2]);
        double floor = 0.05 + (1.0 - depth) * 0.55;
        double wave = floor + (1.0 - floor) * ((std::sin(phase - position) + 1.0) * 0.5);
        brightness_scale *= wave;
        break;
    }
    case LED_EFFECT_SOLID:
    default:
        break;
    }

    *out_red = float_to_u8(red * brightness_scale);
    *out_green = float_to_u8(green * brightness_scale);
    *out_blue = float_to_u8(blue * brightness_scale);
}

static esp_err_t apply_led_state_locked(const led_state_t *state)
{
    uint32_t now_ms = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);

    for (uint16_t i = 0; i < APP_LED_MAX_PIXELS; ++i) {
        uint8_t red = 0;
        uint8_t green = 0;
        uint8_t blue = 0;
        render_effect_pixel(state, i, now_ms, &red, &green, &blue);
        ESP_RETURN_ON_ERROR(led_strip_set_pixel(s_led_strip, i, red, green, blue), TAG, "set pixel failed");
    }

    return led_strip_refresh(s_led_strip);
}

static esp_err_t save_state_to_nvs(const led_state_t *state)
{
    esp_err_t ret = ESP_OK;
    nvs_handle_t nvs_handle = 0;
    char key[16];
    ESP_RETURN_ON_ERROR(nvs_open(APP_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle), TAG, "nvs_open failed");
    ESP_GOTO_ON_ERROR(nvs_set_u16(nvs_handle, "count", state->count), cleanup, TAG, "save count failed");
    ESP_GOTO_ON_ERROR(nvs_set_u8(nvs_handle, "red", state->red), cleanup, TAG, "save red failed");
    ESP_GOTO_ON_ERROR(nvs_set_u8(nvs_handle, "green", state->green), cleanup, TAG, "save green failed");
    ESP_GOTO_ON_ERROR(nvs_set_u8(nvs_handle, "blue", state->blue), cleanup, TAG, "save blue failed");
    ESP_GOTO_ON_ERROR(nvs_set_u8(nvs_handle, "bright", state->brightness), cleanup, TAG, "save brightness failed");
    ESP_GOTO_ON_ERROR(nvs_set_u8(nvs_handle, "power", state->power ? 1 : 0), cleanup, TAG, "save power failed");
    ESP_GOTO_ON_ERROR(nvs_set_u8(nvs_handle, "effect", state->effect), cleanup, TAG, "save effect failed");
    ESP_GOTO_ON_ERROR(nvs_set_str(nvs_handle, "ap_ssid", s_ap_ssid), cleanup, TAG, "save ap ssid failed");
    ESP_GOTO_ON_ERROR(nvs_set_str(nvs_handle, "ap_pass", s_ap_password), cleanup, TAG, "save ap password failed");
    for (uint8_t effect = 0; effect < LED_EFFECT_COUNT; ++effect) {
        for (size_t index = 0; index < kEffectParamSlotCount; ++index) {
            std::snprintf(key, sizeof(key), "e%u_p%u", effect, static_cast<unsigned>(index));
            ESP_GOTO_ON_ERROR(nvs_set_u8(nvs_handle, key, state->effect_profiles[effect].values[index]),
                              cleanup, TAG, "save effect profile failed");
        }
        std::snprintf(key, sizeof(key), "e%u_cr", effect);
        ESP_GOTO_ON_ERROR(nvs_set_u8(nvs_handle, key, state->effect_colors[effect].red), cleanup, TAG,
                          "save effect color red failed");
        std::snprintf(key, sizeof(key), "e%u_cg", effect);
        ESP_GOTO_ON_ERROR(nvs_set_u8(nvs_handle, key, state->effect_colors[effect].green), cleanup, TAG,
                          "save effect color green failed");
        std::snprintf(key, sizeof(key), "e%u_cb", effect);
        ESP_GOTO_ON_ERROR(nvs_set_u8(nvs_handle, key, state->effect_colors[effect].blue), cleanup, TAG,
                          "save effect color blue failed");
    }
    ESP_GOTO_ON_ERROR(nvs_commit(nvs_handle), cleanup, TAG, "nvs_commit failed");

cleanup:
    nvs_close(nvs_handle);
    return ret;
}

static void load_state_from_nvs()
{
    nvs_handle_t nvs_handle = 0;
    char key[16];
    if (nvs_open(APP_NVS_NAMESPACE, NVS_READONLY, &nvs_handle) != ESP_OK) {
        clamp_state(&s_led_state);
        refresh_matter_hs_trackers_from_rgb(s_led_state.red, s_led_state.green, s_led_state.blue);
        return;
    }

    uint16_t count = s_led_state.count;
    uint8_t red = s_led_state.red;
    uint8_t green = s_led_state.green;
    uint8_t blue = s_led_state.blue;
    uint8_t brightness = s_led_state.brightness;
    uint8_t power = s_led_state.power ? 1 : 0;
    uint8_t effect = s_led_state.effect;
    size_t ssid_len = sizeof(s_ap_ssid);
    size_t pass_len = sizeof(s_ap_password);

    nvs_get_u16(nvs_handle, "count", &count);
    nvs_get_u8(nvs_handle, "red", &red);
    nvs_get_u8(nvs_handle, "green", &green);
    nvs_get_u8(nvs_handle, "blue", &blue);
    nvs_get_u8(nvs_handle, "bright", &brightness);
    nvs_get_u8(nvs_handle, "power", &power);
    nvs_get_u8(nvs_handle, "effect", &effect);
    nvs_get_str(nvs_handle, "ap_ssid", s_ap_ssid, &ssid_len);
    nvs_get_str(nvs_handle, "ap_pass", s_ap_password, &pass_len);
    for (uint8_t loaded_effect = 0; loaded_effect < LED_EFFECT_COUNT; ++loaded_effect) {
        for (size_t index = 0; index < kEffectParamSlotCount; ++index) {
            std::snprintf(key, sizeof(key), "e%u_p%u", loaded_effect, static_cast<unsigned>(index));
            nvs_get_u8(nvs_handle, key, &s_led_state.effect_profiles[loaded_effect].values[index]);
        }
        std::snprintf(key, sizeof(key), "e%u_cr", loaded_effect);
        nvs_get_u8(nvs_handle, key, &s_led_state.effect_colors[loaded_effect].red);
        std::snprintf(key, sizeof(key), "e%u_cg", loaded_effect);
        nvs_get_u8(nvs_handle, key, &s_led_state.effect_colors[loaded_effect].green);
        std::snprintf(key, sizeof(key), "e%u_cb", loaded_effect);
        nvs_get_u8(nvs_handle, key, &s_led_state.effect_colors[loaded_effect].blue);
    }
    nvs_close(nvs_handle);

    s_led_state.count = count;
    s_led_state.red = red;
    s_led_state.green = green;
    s_led_state.blue = blue;
    s_led_state.brightness = brightness;
    s_led_state.power = power != 0;
    s_led_state.effect = effect;
    clamp_state(&s_led_state);
    refresh_matter_hs_trackers_from_rgb(s_led_state.red, s_led_state.green, s_led_state.blue);
}

static void apply_startup_power_policy()
{
    // Always boot dark to avoid lighting the strip during power-up.
    s_led_state.power = false;
}

static bool parse_hex_color(const char *color, uint8_t *red, uint8_t *green, uint8_t *blue)
{
    if (!color || std::strlen(color) != 7 || color[0] != '#') {
        return false;
    }

    unsigned int parsed_red = 0;
    unsigned int parsed_green = 0;
    unsigned int parsed_blue = 0;
    if (std::sscanf(color + 1, "%02x%02x%02x", &parsed_red, &parsed_green, &parsed_blue) != 3) {
        return false;
    }

    *red = static_cast<uint8_t>(parsed_red);
    *green = static_cast<uint8_t>(parsed_green);
    *blue = static_cast<uint8_t>(parsed_blue);
    return true;
}

static void format_hex_color(uint8_t red, uint8_t green, uint8_t blue, char *buffer, size_t buffer_len)
{
    if (!buffer || buffer_len == 0) {
        return;
    }

    std::snprintf(buffer, buffer_len, "#%02X%02X%02X", red, green, blue);
}

static void update_ip_string_from_netif(const char *if_key, char *output, size_t output_len)
{
    if (!output || output_len == 0) {
        return;
    }

    output[0] = '\0';
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey(if_key);
    if (!netif) {
        return;
    }

    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK) {
        return;
    }
    if (ip_info.ip.addr == 0) {
        return;
    }

    std::snprintf(output, output_len, IPSTR, IP2STR(&ip_info.ip));
}

static void build_http_url(const char *ip, char *output, size_t output_len)
{
    if (!output || output_len == 0) {
        return;
    }

    output[0] = '\0';
    if (!ip || ip[0] == '\0') {
        return;
    }

    std::snprintf(output, output_len, "http://%s", ip);
}

static void refresh_ip_strings()
{
    char ap_ip[sizeof(s_ap_ip)] = "";
    char sta_ip[sizeof(s_sta_ip)] = "";

    update_ip_string_from_netif("WIFI_AP_DEF", ap_ip, sizeof(ap_ip));
    update_ip_string_from_netif("WIFI_STA_DEF", sta_ip, sizeof(sta_ip));

    if (ap_ip[0] != '\0') {
        std::snprintf(s_ap_ip, sizeof(s_ap_ip), "%s", ap_ip);
    }
    if (sta_ip[0] != '\0') {
        std::snprintf(s_sta_ip, sizeof(s_sta_ip), "%s", sta_ip);
    } else {
        s_sta_ip[0] = '\0';
    }
}

static bool matter_is_commissioned()
{
    if (!matter_is_ready()) {
        return false;
    }
    esp_matter::lock::ScopedChipStackLock lock(portMAX_DELAY);
    return chip::Server::GetInstance().GetFabricTable().FabricCount() > 0;
}

static void refresh_matter_onboarding_data()
{
    if (!matter_is_ready()) {
        s_matter_qr_code[0] = '\0';
        s_matter_manual_code[0] = '\0';
        s_matter_qr_url[0] = '\0';
        return;
    }

    chip::MutableCharSpan qr_span(s_matter_qr_code, sizeof(s_matter_qr_code));
    chip::MutableCharSpan manual_span(s_matter_manual_code, sizeof(s_matter_manual_code));
    auto rendezvous = chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE);

    {
        esp_matter::lock::ScopedChipStackLock lock(portMAX_DELAY);
        if (GetQRCode(qr_span, rendezvous) == CHIP_NO_ERROR) {
            s_matter_qr_code[qr_span.size()] = '\0';
        } else {
            s_matter_qr_code[0] = '\0';
        }

        if (GetManualPairingCode(manual_span, rendezvous) == CHIP_NO_ERROR) {
            s_matter_manual_code[manual_span.size()] = '\0';
        } else {
            s_matter_manual_code[0] = '\0';
        }
    }

    if (s_matter_qr_code[0] != '\0') {
        if (GetQRCodeUrl(s_matter_qr_url, sizeof(s_matter_qr_url),
                         chip::CharSpan(s_matter_qr_code, std::strlen(s_matter_qr_code))) != CHIP_NO_ERROR) {
            s_matter_qr_url[0] = '\0';
        }
    } else {
        s_matter_qr_url[0] = '\0';
    }
}

static const esp_partition_t *get_revert_partition(esp_app_desc_t *app_desc)
{
    const esp_partition_t *running_partition = esp_ota_get_running_partition();
    const esp_partition_t *candidate_partition = esp_ota_get_next_update_partition(nullptr);
    if (!candidate_partition || candidate_partition == running_partition) {
        return nullptr;
    }

    esp_app_desc_t description = {};
    if (esp_ota_get_partition_description(candidate_partition, &description) != ESP_OK) {
        return nullptr;
    }

    if (app_desc) {
        *app_desc = description;
    }
    return candidate_partition;
}

static void sync_matter_state_work_handler(intptr_t arg)
{
    (void) arg;
    if (!matter_is_ready()) {
        return;
    }

    led_state_t snapshot;
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    snapshot = s_led_state;
    xSemaphoreGive(s_state_mutex);

    uint8_t hue = 0;
    uint8_t saturation = 0;
    rgb_to_matter_hs(snapshot.red, snapshot.green, snapshot.blue, &hue, &saturation);

    esp_err_t err = ESP_OK;
    s_syncing_matter = true;

    esp_matter_attr_val_t on_off_value = esp_matter_bool(snapshot.power);
    esp_matter_attr_val_t level_value = esp_matter_nullable_uint8(brightness_to_matter_level(snapshot.brightness));
    esp_matter_attr_val_t hue_value = esp_matter_uint8(hue);
    esp_matter_attr_val_t saturation_value = esp_matter_uint8(saturation);

    err = attribute::update(s_light_endpoint_id, OnOff::Id, OnOff::Attributes::OnOff::Id, &on_off_value);
    if (err == ESP_OK) {
        err = attribute::update(s_light_endpoint_id, LevelControl::Id, LevelControl::Attributes::CurrentLevel::Id,
                                &level_value);
    }
    if (err == ESP_OK) {
        err = attribute::update(s_light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentHue::Id,
                                &hue_value);
    }
    if (err == ESP_OK) {
        err = attribute::update(s_light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentSaturation::Id,
                                &saturation_value);
    }

    s_syncing_matter = false;
    if (err == ESP_OK) {
        s_matter_hue = hue;
        s_matter_saturation = saturation;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Matter attribute sync failed: %s", esp_err_to_name(err));
    }
}

static esp_err_t sync_matter_state_from_led_state()
{
    if (!matter_is_ready()) {
        return ESP_OK;
    }

    CHIP_ERROR schedule_err = chip::DeviceLayer::PlatformMgr().ScheduleWork(sync_matter_state_work_handler, 0);
    if (schedule_err != CHIP_NO_ERROR) {
        ESP_LOGW(TAG, "Failed to schedule Matter sync: %" CHIP_ERROR_FORMAT, schedule_err.Format());
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t send_state_json(httpd_req_t *req)
{
    led_state_t snapshot;
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    snapshot = s_led_state;
    xSemaphoreGive(s_state_mutex);

    refresh_ip_strings();
    refresh_matter_onboarding_data();
    const esp_app_desc_t *app_desc = esp_app_get_description();
    const esp_partition_t *running_partition = esp_ota_get_running_partition();
    const esp_partition_t *ota_target_partition = esp_ota_get_next_update_partition(nullptr);
    esp_app_desc_t revert_desc = {};
    const esp_partition_t *revert_partition = get_revert_partition(&revert_desc);
    char ap_url[32] = "";
    char lan_url[32] = "";
    build_http_url(s_ap_ip, ap_url, sizeof(ap_url));
    build_http_url(s_sta_ip, lan_url, sizeof(lan_url));

    cJSON *root = cJSON_CreateObject();
    if (!root) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "Failed to build response");
    }

    char color_hex[8];
    format_hex_color(snapshot.red, snapshot.green, snapshot.blue, color_hex, sizeof(color_hex));

    cJSON_AddNumberToObject(root, "count", snapshot.count);
    cJSON_AddNumberToObject(root, "brightness", snapshot.brightness);
    cJSON_AddStringToObject(root, "color", color_hex);
    cJSON_AddStringToObject(root, "effect", effect_to_name(snapshot.effect));
    cJSON *effect_profiles = cJSON_AddObjectToObject(root, "effect_profiles");
    for (uint8_t effect = 0; effect < LED_EFFECT_COUNT; ++effect) {
        cJSON *profile = cJSON_AddArrayToObject(effect_profiles, effect_to_name(effect));
        for (size_t index = 0; index < kEffectParamSlotCount; ++index) {
            cJSON_AddItemToArray(profile, cJSON_CreateNumber(snapshot.effect_profiles[effect].values[index]));
        }
    }
    cJSON *effect_colors = cJSON_AddObjectToObject(root, "effect_colors");
    for (uint8_t effect = 0; effect < LED_EFFECT_COUNT; ++effect) {
        char effect_color_hex[8];
        format_hex_color(snapshot.effect_colors[effect].red, snapshot.effect_colors[effect].green,
                         snapshot.effect_colors[effect].blue, effect_color_hex, sizeof(effect_color_hex));
        cJSON_AddStringToObject(effect_colors, effect_to_name(effect), effect_color_hex);
    }
    cJSON_AddBoolToObject(root, "power", snapshot.power);
    cJSON_AddNumberToObject(root, "max_leds", APP_LED_MAX_PIXELS);
    cJSON_AddNumberToObject(root, "gpio", APP_LED_GPIO);
    cJSON_AddStringToObject(root, "ap_ssid", s_runtime_ap_ssid);
    cJSON_AddStringToObject(root, "ap_ip", s_ap_ip);
    cJSON_AddStringToObject(root, "ap_url", ap_url);
    cJSON_AddStringToObject(root, "sta_ip", s_sta_ip);
    cJSON_AddStringToObject(root, "lan_url", lan_url);
    cJSON_AddBoolToObject(root, "sta_connected", s_sta_ip[0] != '\0');
    cJSON_AddStringToObject(root, "config_ap_ssid", s_ap_ssid);
    cJSON_AddStringToObject(root, "config_ap_password", s_ap_password);
    cJSON_AddBoolToObject(root, "ap_restart_required", ap_config_restart_required());
    cJSON_AddBoolToObject(root, "matter_ready", matter_is_ready());
    cJSON_AddBoolToObject(root, "commissioned", matter_is_commissioned());
    cJSON_AddNumberToObject(root, "matter_endpoint", s_light_endpoint_id);
    cJSON_AddStringToObject(root, "fw_version", app_desc ? app_desc->version : "unknown");
    cJSON_AddStringToObject(root, "running_partition", running_partition ? running_partition->label : "");
    cJSON_AddStringToObject(root, "ota_target_partition", ota_target_partition ? ota_target_partition->label : "");
    cJSON_AddBoolToObject(root, "revert_available", revert_partition != nullptr);
    cJSON_AddStringToObject(root, "revert_partition", revert_partition ? revert_partition->label : "");
    cJSON_AddStringToObject(root, "revert_version", revert_partition ? revert_desc.version : "");
    cJSON_AddStringToObject(root, "manual_code", s_matter_manual_code);
    cJSON_AddStringToObject(root, "qr_code", s_matter_qr_code);
    cJSON_AddStringToObject(root, "qr_url", s_matter_qr_url);

    char *response = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!response) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "Failed to encode response");
    }

    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_sendstr(req, response);
    free(response);
    return err;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t captive_redirect_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, nullptr, 0);
    return ESP_OK;
}

static esp_err_t not_found_handler(httpd_req_t *req, httpd_err_code_t err)
{
    (void) err;
    return captive_redirect_handler(req);
}

static esp_err_t state_get_handler(httpd_req_t *req)
{
    return send_state_json(req);
}

static char *read_request_body(httpd_req_t *req)
{
    if (!req || req->content_len <= 0 || req->content_len >= APP_POST_BODY_LIMIT) {
        return nullptr;
    }

    char *body = static_cast<char *>(malloc(req->content_len + 1));
    if (!body) {
        return nullptr;
    }

    int remaining = req->content_len;
    int offset = 0;
    while (remaining > 0) {
        int received = httpd_req_recv(req, body + offset, remaining);
        if (received <= 0) {
            free(body);
            return nullptr;
        }
        offset += received;
        remaining -= received;
    }
    body[offset] = '\0';
    return body;
}

static esp_err_t control_post_handler(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len >= APP_POST_BODY_LIMIT) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "Invalid request body");
    }

    char *body = read_request_body(req);
    if (!body) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "Failed to read request body");
    }
    cJSON *root = cJSON_Parse(body);
    free(body);
    if (!root) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "Invalid JSON");
    }

    led_state_t updated;
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    updated = s_led_state;
    xSemaphoreGive(s_state_mutex);

    cJSON *count = cJSON_GetObjectItemCaseSensitive(root, "count");
    cJSON *brightness = cJSON_GetObjectItemCaseSensitive(root, "brightness");
    cJSON *color = cJSON_GetObjectItemCaseSensitive(root, "color");
    cJSON *effect = cJSON_GetObjectItemCaseSensitive(root, "effect");
    cJSON *effect_params = cJSON_GetObjectItemCaseSensitive(root, "effect_params");
    cJSON *effect_color = cJSON_GetObjectItemCaseSensitive(root, "effect_color");
    cJSON *power = cJSON_GetObjectItemCaseSensitive(root, "power");

    if ((count && !cJSON_IsNumber(count)) || !cJSON_IsNumber(brightness) || !cJSON_IsString(color) ||
        !cJSON_IsString(effect) || !cJSON_IsArray(effect_params) ||
        !(cJSON_IsString(effect_color) || effect_color == nullptr) ||
        !(cJSON_IsBool(power) || power == nullptr)) {
        cJSON_Delete(root);
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "Missing fields");
    }

    if (cJSON_IsNumber(count)) {
        updated.count = static_cast<uint16_t>(count->valuedouble);
    }
    updated.brightness = clamp_u8(static_cast<int>(brightness->valuedouble));
    if (!parse_hex_color(color->valuestring, &updated.red, &updated.green, &updated.blue)) {
        cJSON_Delete(root);
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "Invalid color");
    }
    updated.effect = effect_from_name(effect->valuestring);
    for (size_t index = 0; index < kEffectParamSlotCount; ++index) {
        cJSON *item = cJSON_GetArrayItem(effect_params, index);
        if (cJSON_IsNumber(item)) {
            updated.effect_profiles[updated.effect].values[index] = clamp_u8(static_cast<int>(item->valuedouble));
        }
    }
    if (effect_color &&
        !parse_hex_color(effect_color->valuestring, &updated.effect_colors[updated.effect].red,
                         &updated.effect_colors[updated.effect].green, &updated.effect_colors[updated.effect].blue)) {
        cJSON_Delete(root);
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "Invalid effect color");
    }
    clamp_effect_profile(updated.effect, &updated.effect_profiles[updated.effect]);
    if (power) {
        updated.power = cJSON_IsTrue(power);
    } else {
        updated.power = updated.brightness > 0;
    }
    clamp_state(&updated);
    cJSON_Delete(root);

    esp_err_t err = ESP_OK;
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_led_state = updated;
    refresh_matter_hs_trackers_from_rgb(s_led_state.red, s_led_state.green, s_led_state.blue);
    err = apply_led_state_locked(&s_led_state);
    if (err == ESP_OK) {
        err = save_state_to_nvs(&s_led_state);
    }
    xSemaphoreGive(s_state_mutex);

    if (err != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "Failed to update LED state");
    }

    sync_matter_state_from_led_state();

    return send_state_json(req);
}

static esp_err_t config_post_handler(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len >= APP_POST_BODY_LIMIT) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "Invalid request body");
    }

    char *body = read_request_body(req);
    if (!body) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "Failed to read request body");
    }
    cJSON *root = cJSON_Parse(body);
    free(body);
    if (!root) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "Invalid JSON");
    }

    cJSON *count = cJSON_GetObjectItemCaseSensitive(root, "count");
    cJSON *ap_ssid = cJSON_GetObjectItemCaseSensitive(root, "ap_ssid");
    cJSON *ap_password = cJSON_GetObjectItemCaseSensitive(root, "ap_password");
    if (!cJSON_IsNumber(count) || !cJSON_IsString(ap_ssid) || !cJSON_IsString(ap_password)) {
        cJSON_Delete(root);
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "Missing configuration fields");
    }

    if (!validate_ap_credentials(ap_ssid->valuestring, ap_password->valuestring)) {
        cJSON_Delete(root);
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "AP SSID must be 1-32 chars and password must be 8-63 chars");
    }

    esp_err_t err = ESP_OK;
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_led_state.count = static_cast<uint16_t>(count->valuedouble);
    clamp_state(&s_led_state);
    copy_string_value(s_ap_ssid, sizeof(s_ap_ssid), ap_ssid->valuestring);
    copy_string_value(s_ap_password, sizeof(s_ap_password), ap_password->valuestring);
    err = apply_led_state_locked(&s_led_state);
    if (err == ESP_OK) {
        err = save_state_to_nvs(&s_led_state);
    }
    xSemaphoreGive(s_state_mutex);
    cJSON_Delete(root);

    if (err != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "Failed to save configuration");
    }

    return send_state_json(req);
}

static esp_err_t state_post_handler(httpd_req_t *req)
{
    return control_post_handler(req);
}

static esp_err_t send_message_json(httpd_req_t *req, const char *message)
{
    cJSON *root = cJSON_CreateObject();
    if (!root) {
        return httpd_resp_sendstr(req, message ? message : "Request complete");
    }

    cJSON_AddStringToObject(root, "message", message ? message : "Request complete");
    char *response = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!response) {
        return httpd_resp_sendstr(req, message ? message : "Request complete");
    }

    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_sendstr(req, response);
    free(response);
    return err;
}

static esp_err_t erase_app_settings_namespace()
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(APP_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_erase_all(handle);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

static void reboot_task(void *arg)
{
    (void) arg;
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_restart();
}

static void factory_reset_task(void *arg)
{
    (void) arg;
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_err_t err = erase_app_settings_namespace();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase app settings namespace: %s", esp_err_to_name(err));
    }

    err = esp_matter::factory_reset();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Matter factory reset failed: %s", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }
    vTaskDelete(nullptr);
}

static esp_err_t ota_post_handler(httpd_req_t *req)
{
    if (!s_ota_mutex) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return send_message_json(req, "OTA subsystem is not ready");
    }

    if (xSemaphoreTake(s_ota_mutex, 0) != pdTRUE) {
        httpd_resp_set_status(req, "409 Conflict");
        return send_message_json(req, "Another OTA update is already in progress");
    }

    esp_err_t result = ESP_OK;
    esp_ota_handle_t ota_handle = 0;
    bool ota_started = false;
    char *buffer = nullptr;
    int remaining = 0;
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(nullptr);

    if (!update_partition) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        result = send_message_json(req, "No OTA target partition is available");
        goto cleanup;
    }

    if (req->content_len <= 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        result = send_message_json(req, "Firmware payload is empty");
        goto cleanup;
    }

    if (req->content_len > static_cast<int>(update_partition->size)) {
        httpd_resp_set_status(req, "413 Payload Too Large");
        result = send_message_json(req, "Firmware image is larger than the OTA partition");
        goto cleanup;
    }

    buffer = static_cast<char *>(malloc(APP_OTA_CHUNK_SIZE));
    if (!buffer) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        result = send_message_json(req, "Failed to allocate OTA buffer");
        goto cleanup;
    }

    ESP_LOGI(TAG, "Starting OTA upload to %s (%" PRIu32 " bytes), content length=%d",
             update_partition->label, update_partition->size, req->content_len);

    result = esp_ota_begin(update_partition, req->content_len, &ota_handle);
    if (result != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        result = send_message_json(req, "Failed to start OTA update");
        goto cleanup;
    }
    ota_started = true;

    remaining = req->content_len;
    while (remaining > 0) {
        int received = httpd_req_recv(req, buffer, std::min(remaining, APP_OTA_CHUNK_SIZE));
        if (received == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (received <= 0) {
            ESP_LOGE(TAG, "OTA upload receive failed: %d", received);
            httpd_resp_set_status(req, "400 Bad Request");
            result = send_message_json(req, "Failed to receive firmware data");
            goto cleanup;
        }

        result = esp_ota_write(ota_handle, buffer, received);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "OTA write failed: %s", esp_err_to_name(result));
            httpd_resp_set_status(req, "500 Internal Server Error");
            result = send_message_json(req, "Failed while writing the OTA image");
            goto cleanup;
        }

        remaining -= received;
    }

    result = esp_ota_end(ota_handle);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "OTA finalize failed: %s", esp_err_to_name(result));
        httpd_resp_set_status(req, "400 Bad Request");
        result = send_message_json(req, "Firmware validation failed");
        goto cleanup;
    }
    ota_started = false;

    result = esp_ota_set_boot_partition(update_partition);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select OTA partition: %s", esp_err_to_name(result));
        httpd_resp_set_status(req, "500 Internal Server Error");
        result = send_message_json(req, "Firmware was written but the boot slot could not be updated");
        goto cleanup;
    }

    ESP_LOGI(TAG, "OTA update stored in %s, reboot scheduled", update_partition->label);
    httpd_resp_set_status(req, "200 OK");
    result = send_message_json(req, "Firmware installed successfully. Rebooting into the new OTA slot.");
    xTaskCreate(reboot_task, "ota_reboot", 2048, nullptr, 4, nullptr);

cleanup:
    if (ota_started) {
        esp_ota_abort(ota_handle);
    }
    free(buffer);
    xSemaphoreGive(s_ota_mutex);
    return result;
}

static esp_err_t reboot_post_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "200 OK");
    esp_err_t err = send_message_json(req, "Device will reboot now.");
    xTaskCreate(reboot_task, "device_reboot", 2048, nullptr, 4, nullptr);
    return err;
}

static esp_err_t revert_post_handler(httpd_req_t *req)
{
    esp_app_desc_t revert_desc = {};
    const esp_partition_t *revert_partition = get_revert_partition(&revert_desc);
    if (!revert_partition) {
        httpd_resp_set_status(req, "409 Conflict");
        return send_message_json(req, "No previous firmware slot is available to revert to.");
    }

    esp_err_t err = esp_ota_set_boot_partition(revert_partition);
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return send_message_json(req, "Failed to select the previous firmware slot.");
    }

    char message[160];
    std::snprintf(message, sizeof(message), "Reverting to %s on %s. Device will reboot now.",
                  revert_desc.version, revert_partition->label);
    httpd_resp_set_status(req, "200 OK");
    err = send_message_json(req, message);
    xTaskCreate(reboot_task, "revert_reboot", 2048, nullptr, 4, nullptr);
    return err;
}

static esp_err_t factory_reset_post_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "200 OK");
    esp_err_t err = send_message_json(req, "Factory reset started. The device will erase settings and reboot.");
    xTaskCreate(factory_reset_task, "factory_reset", 3072, nullptr, 4, nullptr);
    return err;
}

static void effect_task(void *arg)
{
    (void) arg;
    while (true) {
        bool animated = false;
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        animated = s_led_state.power && s_led_state.effect != LED_EFFECT_SOLID;
        if (animated) {
            esp_err_t err = apply_led_state_locked(&s_led_state);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Effect render failed: %s", esp_err_to_name(err));
            }
        }
        xSemaphoreGive(s_state_mutex);
        vTaskDelay(pdMS_TO_TICKS(animated ? 40 : 250));
    }
}

static void start_webserver()
{
    if (s_http_server) {
        return;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 24;
    config.stack_size = 8192;
    config.recv_wait_timeout = 30;
    config.send_wait_timeout = 30;

    ESP_ERROR_CHECK(httpd_start(&s_http_server, &config));

    httpd_uri_t root = {};
    root.uri = "/";
    root.method = HTTP_GET;
    root.handler = root_get_handler;

    httpd_uri_t state_get = {};
    state_get.uri = "/api/state";
    state_get.method = HTTP_GET;
    state_get.handler = state_get_handler;

    httpd_uri_t state_post = {};
    state_post.uri = "/api/state";
    state_post.method = HTTP_POST;
    state_post.handler = state_post_handler;

    httpd_uri_t control_post = {};
    control_post.uri = "/api/control";
    control_post.method = HTTP_POST;
    control_post.handler = control_post_handler;

    httpd_uri_t config_post = {};
    config_post.uri = "/api/config";
    config_post.method = HTTP_POST;
    config_post.handler = config_post_handler;

    httpd_uri_t ota_post = {};
    ota_post.uri = "/api/ota";
    ota_post.method = HTTP_POST;
    ota_post.handler = ota_post_handler;

    httpd_uri_t reboot_post = {};
    reboot_post.uri = "/api/reboot";
    reboot_post.method = HTTP_POST;
    reboot_post.handler = reboot_post_handler;

    httpd_uri_t revert_post = {};
    revert_post.uri = "/api/revert";
    revert_post.method = HTTP_POST;
    revert_post.handler = revert_post_handler;

    httpd_uri_t factory_reset_post = {};
    factory_reset_post.uri = "/api/factory-reset";
    factory_reset_post.method = HTTP_POST;
    factory_reset_post.handler = factory_reset_post_handler;

    httpd_uri_t generate_204 = {};
    generate_204.uri = "/generate_204";
    generate_204.method = HTTP_GET;
    generate_204.handler = captive_redirect_handler;

    httpd_uri_t hotspot_detect = {};
    hotspot_detect.uri = "/hotspot-detect.html";
    hotspot_detect.method = HTTP_GET;
    hotspot_detect.handler = captive_redirect_handler;

    httpd_uri_t ncsi = {};
    ncsi.uri = "/ncsi.txt";
    ncsi.method = HTTP_GET;
    ncsi.handler = captive_redirect_handler;

    httpd_uri_t connecttest = {};
    connecttest.uri = "/connecttest.txt";
    connecttest.method = HTTP_GET;
    connecttest.handler = captive_redirect_handler;

    httpd_uri_t canonical = {};
    canonical.uri = "/canonical.html";
    canonical.method = HTTP_GET;
    canonical.handler = captive_redirect_handler;

    httpd_uri_t fwlink = {};
    fwlink.uri = "/fwlink";
    fwlink.method = HTTP_GET;
    fwlink.handler = captive_redirect_handler;

    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &root));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &state_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &state_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &control_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &config_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &ota_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &reboot_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &revert_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &factory_reset_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &generate_204));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &hotspot_detect));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &ncsi));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &connecttest));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &canonical));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &fwlink));
    ESP_ERROR_CHECK(httpd_register_err_handler(s_http_server, HTTPD_404_NOT_FOUND, not_found_handler));
}

static size_t dns_skip_name(const uint8_t *packet, size_t length, size_t offset)
{
    while (offset < length) {
        uint8_t label_len = packet[offset];
        if (label_len == 0) {
            return offset + 1;
        }
        if ((label_len & 0xC0) == 0xC0) {
            return offset + 2;
        }
        offset += label_len + 1;
    }
    return 0;
}

static size_t build_dns_response(const uint8_t *query, size_t query_len, uint8_t *response, size_t response_len)
{
    if (query_len < 12 || response_len < 12) {
        return 0;
    }

    uint16_t qdcount = (static_cast<uint16_t>(query[4]) << 8) | query[5];
    if (qdcount == 0) {
        return 0;
    }

    size_t question_end = dns_skip_name(query, query_len, 12);
    if (question_end == 0 || question_end + 4 > query_len) {
        return 0;
    }

    uint16_t qtype = (static_cast<uint16_t>(query[question_end]) << 8) | query[question_end + 1];
    uint16_t qclass = (static_cast<uint16_t>(query[question_end + 2]) << 8) | query[question_end + 3];

    if (question_end + 4 + 16 > response_len) {
        return 0;
    }

    std::memcpy(response, query, question_end + 4);
    response[2] = 0x81;
    response[3] = 0x80;
    response[4] = 0x00;
    response[5] = 0x01;
    response[6] = 0x00;
    response[7] = (qtype == 1 && qclass == 1) ? 0x01 : 0x00;
    response[8] = 0x00;
    response[9] = 0x00;
    response[10] = 0x00;
    response[11] = 0x00;

    if (qtype != 1 || qclass != 1) {
        return question_end + 4;
    }

    size_t offset = question_end + 4;
    response[offset++] = 0xC0;
    response[offset++] = 0x0C;
    response[offset++] = 0x00;
    response[offset++] = 0x01;
    response[offset++] = 0x00;
    response[offset++] = 0x01;
    response[offset++] = 0x00;
    response[offset++] = 0x00;
    response[offset++] = 0x00;
    response[offset++] = 0x3C;
    response[offset++] = 0x00;
    response[offset++] = 0x04;

    unsigned int octet0 = 192;
    unsigned int octet1 = 168;
    unsigned int octet2 = 4;
    unsigned int octet3 = 1;
    if (std::sscanf(s_ap_ip, "%u.%u.%u.%u", &octet0, &octet1, &octet2, &octet3) != 4) {
        return 0;
    }
    response[offset++] = static_cast<uint8_t>(octet0);
    response[offset++] = static_cast<uint8_t>(octet1);
    response[offset++] = static_cast<uint8_t>(octet2);
    response[offset++] = static_cast<uint8_t>(octet3);
    return offset;
}

static void captive_dns_task(void *arg)
{
    (void) arg;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create DNS socket");
        vTaskDelete(nullptr);
        return;
    }

    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(53);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind DNS socket");
        close(sock);
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TAG, "Captive DNS server listening on %s:53", s_ap_ip);

    while (true) {
        uint8_t query[512];
        uint8_t response[512];
        struct sockaddr_in client_addr = {};
        socklen_t client_len = sizeof(client_addr);
        ssize_t received = recvfrom(sock, query, sizeof(query), 0, reinterpret_cast<struct sockaddr *>(&client_addr), &client_len);
        if (received <= 0) {
            continue;
        }

        size_t response_len = build_dns_response(query, static_cast<size_t>(received), response, sizeof(response));
        if (response_len > 0) {
            sendto(sock, response, response_len, 0, reinterpret_cast<struct sockaddr *>(&client_addr), client_len);
        }
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void) arg;
    if (event_base != WIFI_EVENT) {
        return;
    }

    switch (event_id) {
    case WIFI_EVENT_AP_STACONNECTED: {
        auto *event = static_cast<wifi_event_ap_staconnected_t *>(event_data);
        ESP_LOGI(TAG, "station " MACSTR " joined, aid=%d", MAC2STR(event->mac), event->aid);
        break;
    }
    case WIFI_EVENT_AP_STADISCONNECTED: {
        auto *event = static_cast<wifi_event_ap_stadisconnected_t *>(event_data);
        ESP_LOGI(TAG, "station " MACSTR " left, aid=%d", MAC2STR(event->mac), event->aid);
        break;
    }
    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGI(TAG, "Matter station connected to upstream Wi-Fi");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        s_sta_ip[0] = '\0';
        ESP_LOGI(TAG, "Matter station disconnected from upstream Wi-Fi");
        break;
    default:
        break;
    }
}

static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void) arg;
    if (event_base != IP_EVENT) {
        return;
    }

    if (event_id == IP_EVENT_STA_GOT_IP) {
        auto *event = static_cast<ip_event_got_ip_t *>(event_data);
        std::snprintf(s_sta_ip, sizeof(s_sta_ip), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Matter station IP: %s", s_sta_ip);
        ESP_LOGI(TAG, "LAN web UI available at http://%s", s_sta_ip);
    }
}

static esp_err_t ensure_wifi_started()
{
    int8_t ignored = 0;
    esp_err_t err = esp_wifi_get_max_tx_power(&ignored);
    if (err == ESP_OK) {
        return ESP_OK;
    }
    if (err == ESP_ERR_WIFI_NOT_STARTED) {
        return esp_wifi_start();
    }
    return err;
}

static void start_softap_overlay()
{
    if (!s_network_handlers_registered) {
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, nullptr));
        s_network_handlers_registered = true;
    }

    if (!s_ap_netif) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        ESP_ERROR_CHECK(s_ap_netif ? ESP_OK : ESP_FAIL);
    }

    wifi_mode_t mode = WIFI_MODE_NULL;
    ESP_ERROR_CHECK(esp_wifi_get_mode(&mode));
    if (mode == WIFI_MODE_STA) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    } else if (mode == WIFI_MODE_NULL) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    }

    wifi_config_t wifi_config = {};
    std::memset(wifi_config.ap.ssid, 0, sizeof(wifi_config.ap.ssid));
    std::memset(wifi_config.ap.password, 0, sizeof(wifi_config.ap.password));
    size_t ssid_len = std::min(sizeof(wifi_config.ap.ssid), std::strlen(s_ap_ssid));
    size_t password_len = std::min(sizeof(wifi_config.ap.password) - 1, std::strlen(s_ap_password));
    std::memcpy(wifi_config.ap.ssid, s_ap_ssid, ssid_len);
    std::memcpy(wifi_config.ap.password, s_ap_password, password_len);
    wifi_config.ap.ssid_len = ssid_len;
    wifi_config.ap.channel = APP_WIFI_CHANNEL;
    wifi_config.ap.max_connection = APP_WIFI_MAX_STA_CONN;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.ap.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    copy_string_value(s_runtime_ap_ssid, sizeof(s_runtime_ap_ssid), s_ap_ssid);
    copy_string_value(s_runtime_ap_password, sizeof(s_runtime_ap_password), s_ap_password);
    ESP_ERROR_CHECK(ensure_wifi_started());

    refresh_ip_strings();
    ESP_LOGI(TAG, "SoftAP started: SSID=%s password=%s", s_runtime_ap_ssid, s_runtime_ap_password);
    ESP_LOGI(TAG, "Open http://%s", s_ap_ip);
    if (s_sta_ip[0] != '\0') {
        ESP_LOGI(TAG, "LAN access available at http://%s", s_sta_ip);
    }
}

static void init_led_strip()
{
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = APP_LED_GPIO;
    strip_config.max_leds = APP_LED_MAX_PIXELS;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.resolution_hz = APP_RMT_RESOLUTION_HZ;
    rmt_config.flags.with_dma = false;

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip));
    ESP_ERROR_CHECK(led_strip_clear(s_led_strip));
}

static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    (void) endpoint_id;
    (void) priv_data;
    ESP_LOGI(TAG, "Identification callback: type=%u effect=%u variant=%u", type, effect_id, effect_variant);
    return ESP_OK;
}

static esp_err_t persist_and_apply_locked()
{
    esp_err_t err = apply_led_state_locked(&s_led_state);
    if (err == ESP_OK) {
        err = save_state_to_nvs(&s_led_state);
    }
    return err;
}

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    (void) priv_data;
    if (type != esp_matter::attribute::PRE_UPDATE || endpoint_id != s_light_endpoint_id || s_syncing_matter) {
        return ESP_OK;
    }

    esp_err_t err = ESP_OK;
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    led_state_t updated = s_led_state;

    if (cluster_id == OnOff::Id && attribute_id == OnOff::Attributes::OnOff::Id) {
        updated.power = val->val.b;
    } else if (cluster_id == LevelControl::Id && attribute_id == LevelControl::Attributes::CurrentLevel::Id) {
        updated.brightness = matter_level_to_brightness(val->val.u8);
    } else if (cluster_id == ColorControl::Id && attribute_id == ColorControl::Attributes::CurrentHue::Id) {
        s_matter_hue = val->val.u8;
        matter_hs_to_rgb(s_matter_hue, s_matter_saturation, &updated.red, &updated.green, &updated.blue);
    } else if (cluster_id == ColorControl::Id && attribute_id == ColorControl::Attributes::CurrentSaturation::Id) {
        s_matter_saturation = val->val.u8;
        matter_hs_to_rgb(s_matter_hue, s_matter_saturation, &updated.red, &updated.green, &updated.blue);
    } else if (cluster_id == ColorControl::Id && attribute_id == ColorControl::Attributes::CurrentX::Id) {
        s_matter_x = val->val.u16;
        matter_xy_to_rgb(s_matter_x, s_matter_y, &updated.red, &updated.green, &updated.blue);
        refresh_matter_hs_trackers_from_rgb(updated.red, updated.green, updated.blue);
    } else if (cluster_id == ColorControl::Id && attribute_id == ColorControl::Attributes::CurrentY::Id) {
        s_matter_y = val->val.u16;
        matter_xy_to_rgb(s_matter_x, s_matter_y, &updated.red, &updated.green, &updated.blue);
        refresh_matter_hs_trackers_from_rgb(updated.red, updated.green, updated.blue);
    } else if (cluster_id == ColorControl::Id && attribute_id == ColorControl::Attributes::ColorTemperatureMireds::Id) {
        s_matter_temp_mireds = val->val.u16;
        color_temp_to_rgb(s_matter_temp_mireds, &updated.red, &updated.green, &updated.blue);
        refresh_matter_hs_trackers_from_rgb(updated.red, updated.green, updated.blue);
    } else {
        xSemaphoreGive(s_state_mutex);
        return ESP_OK;
    }

    clamp_state(&updated);
    s_led_state = updated;
    err = persist_and_apply_locked();
    xSemaphoreGive(s_state_mutex);
    return err;
}

static void set_initial_matter_color_attributes()
{
    uint8_t hue = 0;
    uint8_t saturation = 0;
    rgb_to_matter_hs(s_led_state.red, s_led_state.green, s_led_state.blue, &hue, &saturation);
    s_matter_hue = hue;
    s_matter_saturation = saturation;

    attribute_t *attribute = attribute::get(s_light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentHue::Id);
    if (attribute) {
        esp_matter_attr_val_t value = esp_matter_uint8(hue);
        attribute::set_val(attribute, &value);
    }

    attribute = attribute::get(s_light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentSaturation::Id);
    if (attribute) {
        esp_matter_attr_val_t value = esp_matter_uint8(saturation);
        attribute::set_val(attribute, &value);
    }

    attribute = attribute::get(s_light_endpoint_id, ColorControl::Id, ColorControl::Attributes::ColorMode::Id);
    if (attribute) {
        esp_matter_attr_val_t value = esp_matter_enum8(static_cast<uint8_t>(ColorControl::ColorMode::kCurrentHueAndCurrentSaturation));
        attribute::set_val(attribute, &value);
    }

    attribute = attribute::get(s_light_endpoint_id, ColorControl::Id, ColorControl::Attributes::EnhancedColorMode::Id);
    if (attribute) {
        esp_matter_attr_val_t value = esp_matter_enum8(static_cast<uint8_t>(ColorControl::ColorMode::kCurrentHueAndCurrentSaturation));
        attribute::set_val(attribute, &value);
    }
}

static void configure_matter_node()
{
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    assert(node != nullptr);

    endpoint::extended_color_light::config_t light_config;
    light_config.on_off.on_off = s_led_state.power;
    light_config.on_off_lighting.start_up_on_off = nullable<uint8_t>(s_led_state.power ? 1 : 0);
    light_config.level_control.current_level = nullable<uint8_t>(brightness_to_matter_level(s_led_state.brightness));
    light_config.level_control.on_level = nullable<uint8_t>(brightness_to_matter_level(s_led_state.brightness));
    light_config.level_control_lighting.start_up_current_level = nullable<uint8_t>(brightness_to_matter_level(s_led_state.brightness));
    light_config.color_control.color_mode = static_cast<uint8_t>(ColorControl::ColorMode::kCurrentHueAndCurrentSaturation);
    light_config.color_control.enhanced_color_mode = static_cast<uint8_t>(ColorControl::ColorMode::kCurrentHueAndCurrentSaturation);
    light_config.color_control_color_temperature.color_temperature_mireds = s_matter_temp_mireds;
    light_config.color_control_color_temperature.start_up_color_temperature_mireds = nullable<uint16_t>(s_matter_temp_mireds);
    light_config.color_control_xy.current_x = s_matter_x;
    light_config.color_control_xy.current_y = s_matter_y;

    endpoint_t *endpoint = endpoint::extended_color_light::create(node, &light_config, ENDPOINT_FLAG_NONE, nullptr);
    assert(endpoint != nullptr);
    s_light_endpoint_id = endpoint::get_id(endpoint);

    set_initial_matter_color_attributes();

    attribute_t *level_attribute = attribute::get(s_light_endpoint_id, LevelControl::Id, LevelControl::Attributes::CurrentLevel::Id);
    if (level_attribute) {
        attribute::set_deferred_persistence(level_attribute);
    }

    attribute_t *hue_attribute = attribute::get(s_light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentHue::Id);
    if (hue_attribute) {
        attribute::set_deferred_persistence(hue_attribute);
    }

    attribute_t *saturation_attribute = attribute::get(s_light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentSaturation::Id);
    if (saturation_attribute) {
        attribute::set_deferred_persistence(saturation_attribute);
    }

    attribute_t *temperature_attribute = attribute::get(s_light_endpoint_id, ColorControl::Id,
                                                        ColorControl::Attributes::ColorTemperatureMireds::Id);
    if (temperature_attribute) {
        attribute::set_deferred_persistence(temperature_attribute);
    }
}

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    (void) arg;
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Matter commissioning complete");
        refresh_ip_strings();
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Matter commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Matter commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Matter commissioning window opened");
        refresh_matter_onboarding_data();
        PrintOnboardingCodes(chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE));
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Matter commissioning window closed");
        break;

    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        refresh_ip_strings();
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved: {
        ESP_LOGI(TAG, "Matter fabric removed");
        esp_matter::lock::ScopedChipStackLock lock(portMAX_DELAY);
        if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) {
            auto &commission_mgr = chip::Server::GetInstance().GetCommissioningWindowManager();
            if (!commission_mgr.IsCommissioningWindowOpen()) {
                CHIP_ERROR err = commission_mgr.OpenBasicCommissioningWindow(
                    chip::System::Clock::Seconds16(kCommissioningTimeoutSeconds),
                    chip::CommissioningWindowAdvertisement::kDnssdOnly);
                if (err != CHIP_NO_ERROR) {
                    ESP_LOGE(TAG, "Failed to reopen commissioning window: %" CHIP_ERROR_FORMAT, err.Format());
                }
            }
        }
        break;
    }

    default:
        break;
    }
}

extern "C" void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_state_mutex = xSemaphoreCreateMutex();
    assert(s_state_mutex != nullptr);
    s_ota_mutex = xSemaphoreCreateMutex();
    assert(s_ota_mutex != nullptr);

    reset_effect_profiles_to_defaults(&s_led_state);
    reset_effect_colors_to_defaults(&s_led_state);
    load_state_from_nvs();
    apply_startup_power_policy();
    init_led_strip();

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    ESP_ERROR_CHECK(apply_led_state_locked(&s_led_state));
    xSemaphoreGive(s_state_mutex);

    configure_matter_node();
    ESP_ERROR_CHECK(esp_matter::start(app_event_cb));
    refresh_matter_onboarding_data();
    PrintOnboardingCodes(chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE));

    start_softap_overlay();
    start_webserver();
    xTaskCreate(effect_task, "effect_task", 4096, nullptr, 4, nullptr);
    xTaskCreate(captive_dns_task, "captive_dns", 4096, nullptr, 4, nullptr);

    ESP_LOGI(TAG, "Project ready. LEDs=%u power=%u brightness=%u effect=%s color=#%02X%02X%02X",
             s_led_state.count, s_led_state.power, s_led_state.brightness, effect_to_name(s_led_state.effect),
             s_led_state.red, s_led_state.green, s_led_state.blue);
}
