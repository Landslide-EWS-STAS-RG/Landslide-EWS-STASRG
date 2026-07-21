# MQTT Upgrade — Landslide EWS (STASRG)

This upgrade removes the USB serial tether. The new data path is:

```
ESP32 ──WiFi──> MQTT Broker (Mosquitto) ──> Node-RED (mqtt in) ──> Dashboard + InfluxDB
```

`python/serial_bridge.py` is no longer needed once this is running.

## Files

- `LandslideMonitoringSystem_MQTT.ino` — your improved sketch with WiFi + MQTT added.
  The reserved `publishNetworkTelemetry()` extension point is now implemented.
- `landslide_flow_mqtt.json` — your Node-RED flow with MQTT input nodes added.
  The old HTTP endpoints still work, so you can migrate gradually.

## 1. Install an MQTT broker (Mosquitto)

On the machine that already runs Node-RED + InfluxDB:

```shell
# macOS
brew install mosquitto

# Linux (Debian/Ubuntu)
sudo apt install mosquitto mosquitto-clients
```

By default Mosquitto only listens on localhost. To accept the ESP32 from the
LAN, edit `mosquitto.conf` (macOS: `/opt/homebrew/etc/mosquitto/mosquitto.conf`):

```
listener 1883 0.0.0.0
allow_anonymous true
```

Then start it: `brew services start mosquitto` (or `sudo systemctl enable --now mosquitto`).

> For a field deployment where the ESP32 is far from your computer, use a free
> cloud broker instead (e.g. HiveMQ Cloud or EMQX Cloud) and set `MQTT_HOST`,
> `MQTT_USER`, `MQTT_PASS` accordingly. Both Node-RED and the ESP32 then only
> need internet access, not the same LAN.

## 2. Flash the ESP32

1. Open `LandslideMonitoringSystem_MQTT.ino` in Arduino IDE.
2. Install libraries **PubSubClient** (Nick O'Leary) and **ArduinoJson** (v7)
   via Library Manager. (PlatformIO users: they are already in `platformio.ini`.)
3. Edit the `NETWORK TELEMETRY` section:
   - `WIFI_SSID` / `WIFI_PASSWORD`
   - `MQTT_HOST` → the LAN IP of the broker machine (find it with `ipconfig` /
     `ifconfig`), or the cloud broker hostname.
4. Upload. The Serial Monitor will show `WIFI  : connecting...` then
   `MQTT  : connected` (you can unplug USB after verifying — power the board
   with any 5V USB adapter or battery).

## 3. Update Node-RED

1. Manage Palette → install `node-red-dashboard` and `node-red-contrib-influxdb`
   (same as before).
2. Import `landslide_flow_mqtt.json` (replace the old flow to avoid duplicate
   HTTP endpoints, or delete the old tab first).
3. Set your InfluxDB API token on the `influxdb out` node (same as before).
4. If the broker is not on the same machine as Node-RED, double-click any
   `mqtt in` node → edit the **Local Mosquitto** broker config → change host.
5. Deploy. The MQTT nodes should show a green **connected** dot.

## 4. Verify

```shell
# Watch everything the ESP32 publishes:
mosquitto_sub -h localhost -t 'landslide/#' -v
```

You should see JSON once per second on `landslide/ews-esp32-01/telemetry`:

```json
{"device":"ews-esp32-01","ts":123456,"tilt":1.23,"gravity":0.004,
 "rotation":0.5,"piezo":12,"hazard":3,"state":"NORMAL",
 "confidence":97.1,"events":0,"rssi":-58}
```

Topics used:

| Topic | Content |
|---|---|
| `landslide/<device>/telemetry` | JSON sensor record, 1 Hz (throttled) |
| `landslide/<device>/event` | Published immediately when a new event fires |
| `landslide/<device>/status` | Retained `online` / `offline` (MQTT Last Will) |

## Design notes

- **Non-blocking**: WiFi/MQTT reconnection never blocks the 10 Hz sampling
  loop, so the local siren/buzzer still fires even with no network.
- **Same JSON keys** as the old serial bridge (`tilt`, `gravity`, `rotation`,
  `piezo`, `hazard`, `state`), so the existing dashboard and InfluxDB format
  functions are reused unchanged.
- **Throttling**: telemetry is limited to 1 msg/s (`TELEMETRY_PUBLISH_INTERVAL_MS`),
  but new hazard events bypass the throttle and publish instantly.
- **Last Will**: if the ESP32 loses power/WiFi, the broker automatically
  publishes `offline` to the status topic so the dashboard log shows it.
