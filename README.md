# Intercom MQTT Backpack

See https://michael.stapelberg.ch/posts/2021-03-13-smart-intercom-backpack/

## Components

### tinypico-mqtt-nuki/

ESP32 (TinyPICO) firmware that bridges the BTicino SCS bus and a Nuki Opener
to MQTT. Listens for door-station ring telegrams, drives the Nuki opener (or
the local relay), and exposes everything as MQTT topics.

#### MQTT topics

Published by the doorbell:

| Topic                           | Payload                                                | Notes                  |
|---------------------------------|--------------------------------------------------------|------------------------|
| `doorbell/status`               | `online` / `offline`                                   | retained, LWT          |
| `doorbell/heartbeat`            | `{"uptime_s":N,"rssi":N,"silent":bool}`                | every 60 s             |
| `doorbell/events/scs`           | hex-encoded 7-byte SCS telegram (e.g. `a8 91 03 60 …`) | every detected frame   |
| `doorbell/events/ring`          | `house` or `floor`                                     | one per ring event     |
| `doorbell/silent_state`         | `on` / `off`                                           | retained, doorbell echo |
| `doorbell/debug/suppressed`     | `nukiRing` / `floorRingNuki`                           | when silent suppressed |
| `doorbell/debug/nukiring`       | `ring`                                                 | after Nuki yellow pulse |
| `doorbell/debug/doorunlock`     | `doorunlock`                                           | after relay pulse      |
| `doorbell/debug/floorring`      | `low` / `high`                                         | after floor ring debounce |

Subscribed by the doorbell (set from anywhere on the LAN):

| Topic                       | Payload                | Effect                                   |
|-----------------------------|------------------------|------------------------------------------|
| `doorbell/cmd/unlock`       | any                    | pulses the door-open relay (500 ms)      |
| `doorbell/debug/cmd/ring`   | any                    | simulates a ring (drives Nuki yellow)    |
| `doorbell/silent`           | `on` / `off` (retained) | suppresses Nuki auto-trigger; rings still observable on `events/ring` |

`doorbell/silent` should be published with the retained flag so the state
survives doorbell reboots:

```
mosquitto_pub -h mqtt.lan -t doorbell/silent -r -m on
```

#### Build and flash on NixOS

Pinned to esp32:esp32 2.0.17, PubSubClient 2.8, TinyPICO Helper Library 1.3.0.

One-time bootstrap (downloads the ESP32 core and the two libraries into
`~/.arduino15` and `~/Arduino`):

```bash
nix-shell -p arduino-cli --run '
  arduino-cli config init --overwrite
  arduino-cli config add board_manager.additional_urls \
    https://espressif.github.io/arduino-esp32/package_esp32_index.json
  arduino-cli core update-index
  arduino-cli core install esp32:esp32@2.0.17
  arduino-cli lib install "PubSubClient@2.8"
  arduino-cli lib install "TinyPICO Helper Library@1.3.0"
'
```

esp32 2.0.17 ships an `esptool.py` whose shebang is `#!/usr/bin/env python`
(not `python3`) and that imports `pyserial`. NixOS provides neither in the
default PATH, so build/flash needs both `arduino-cli` and a
`python3.withPackages [pyserial]` plus a one-line `python` shim:

```bash
mkdir -p ~/.local/share/arduino-pyshim
ln -sf "$(command -v python3)" ~/.local/share/arduino-pyshim/python
```

Build (verify):

```bash
nix-shell -p arduino-cli 'python3.withPackages (ps: [ps.pyserial])' --run '
  PATH=~/.local/share/arduino-pyshim:$PATH \
    arduino-cli compile --fqbn esp32:esp32:tinypico ./tinypico-mqtt-nuki
'
```

Expected: ~760 KB sketch (58 % of program flash), ~45 KB globals (13 % of
DRAM), no warnings.

Flash (TinyPICO usually shows up as `/dev/ttyUSBN` via its CP2104 USB-UART
bridge — check `ls /dev/serial/by-id/`):

```bash
nix-shell -p arduino-cli 'python3.withPackages (ps: [ps.pyserial])' --run '
  PATH=~/.local/share/arduino-pyshim:$PATH \
    arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:tinypico ./tinypico-mqtt-nuki
'
```

Access to `/dev/ttyUSB*` requires the user to be in the `dialout` group.

### qt-py-scs2uart/

Adafruit QT Py firmware that decodes SCS bus signals and forwards them as
9600-baud UART to whatever sits behind it (in this repo, the TinyPICO above).
Same arduino-cli mechanics as the tinypico build, different board package
(this one has not been re-validated on NixOS in 2026):

```bash
nix-shell -p arduino-cli --run '
  arduino-cli config add board_manager.additional_urls \
    https://www.adafruit.com/package_adafruit_index.json
  arduino-cli core update-index
  arduino-cli core install adafruit:samd
  arduino-cli compile --fqbn adafruit:samd:adafruit_qtpy_m0 ./qt-py-scs2uart
'
```

### kicad-backpack/

A board that is mounted onto the [Adafruit QT
Py](https://www.adafruit.com/product/4600) (hence “backpack”) and adds 5V/3.3V
conversion for interfacing with a BTicino intercom unit (5V).
