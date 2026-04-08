# ESP32-C6 Matter WS2812B Controller

This is a small ESP-IDF + ESP-Matter test project for an ESP32-C6 driving a WS2812B LED strip.

It exposes the strip in two ways:

- A local web page for LED count, color, brightness, effects, OTA, and reset actions
- A Matter extended color light endpoint for Apple Home
- A local OTA firmware upload flow from the same web page
- A firmware revert action that boots back into the other OTA slot
- A factory reset action that clears Matter pairing, Wi-Fi AP config, and saved LED settings
- Built-in LED effects: `glow`, `rainbow`, `chase`, `sparkle`, `wave`
- Per-effect controls in the web UI with unique meanings for each animation

The firmware starts a Wi-Fi SoftAP, serves the control page directly from the board, and includes captive-portal style redirects plus a small DNS responder so phones and laptops are more likely to open the page automatically. After the device joins your normal Wi-Fi through Matter commissioning, the same web UI is also reachable on its LAN IP.

## Defaults

- SoftAP SSID/password: generated per device on first boot and stored in NVS
- Web UI: `http://192.168.4.1`
- LED data GPIO: `17` (`D7` on XIAO ESP32C6)
- Maximum strip length compiled in: `120`
- Matter device type: extended color light

## Wiring

- `XIAO ESP32C6 D7 / GPIO17` -> `DIN` on WS2812B strip
- `GND` -> strip ground
- External `5V` -> strip power
- Share ground between the ESP32-C6 and LED power supply

For anything beyond a few LEDs, use an external 5V supply.

## Build and flash

```bash
cd ~/esp32c6-led-web
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

The first install still needs USB flashing. After that, you can upload new firmware from the web UI using the generated app binary:

```bash
~/esp32c6-led-web/build/esp32c6_led_web.bin
```

## Web UI layout

The device page is split into three tabs:

- `Overview`: Matter state, AP and LAN web UI URLs, firmware version, running slot, revert target
- `Configuration`: LED count, SoftAP SSID/password, OTA upload, revert button, factory reset, reboot
- `Control`: brightness, color, and one sub-tab per effect with its own parameters

The SoftAP credentials shown in `Configuration` are the credentials hosted by the ESP32-C6 itself for the local setup page. On a fresh device they are generated automatically and printed to the serial log when the AP starts.

## Pair with Apple Home

1. Flash the firmware and open the serial monitor.
2. Watch for the Matter onboarding output.
3. Open Apple Home on your iPhone or iPad.
4. Tap `+`, then `Add Accessory`.
5. Scan the Matter QR code from the serial log, or enter the manual setup code.
6. Finish commissioning while the phone is connected over BLE and the ESP32-C6 has network access.

The local web page also shows the current Matter state, manual setup code, QR URL, and both web UI addresses.

## OTA updates

1. Build a new firmware image with `idf.py build`.
2. Open the device web UI at `http://192.168.4.1` or its LAN IP after commissioning.
3. In the `Firmware Update` section, choose `build/esp32c6_led_web.bin`.
4. Click `Install OTA Update`.
5. Wait for the board to reboot into the other OTA slot.

After at least one successful OTA update, the `Revert To Previous Firmware` button in `Configuration` can boot the device back into the other application slot.

This OTA path updates the application partition only. Bootloader and partition table changes still require USB flashing.

## Reset options

- `Reset`: reboots the device and applies any saved SoftAP credential changes
- `Factory Reset`: clears the app settings namespace, then runs the Matter factory reset flow so pairing data and local settings are removed cleanly
- `Revert To Previous Firmware`: switches boot to the other OTA app slot without erasing settings

## Notes

- The web page stores settings in NVS, so they survive reboot.
- LED count, AP credentials, and effects stay local to the web UI. Matter and Apple Home sync power, color, and brightness.
- Each effect now has its own saved parameter set. Examples:
- `glow`: pulse speed, glow floor, pulse depth
- `rainbow`: drift speed, rainbow length, color blend, start offset, contrast
- `chase`: chase speed, tail length, tail sharpness
- `sparkle`: spark density, base glow, twinkle speed
- `wave`: wave speed, wavelength, wave depth
- The LED count set in the page is clamped to the compiled-in maximum.
- If your strip uses a different GPIO, update `APP_LED_GPIO` in `main/app_main.cpp`.
- On the XIAO ESP32C6, `D7` maps to `GPIO17`.
- If you want more than `120` pixels, update `APP_LED_MAX_PIXELS` in `main/app_main.cpp`.
- The serial monitor is the most reliable place to get the first Matter pairing codes after boot.
