# Intercom MQTT Backpack

See https://michael.stapelberg.ch/posts/2021-03-13-smart-intercom-backpack/

## Components

### tinypico-mqtt-nuki/

```
# Only debian:testing is recent enough to provide arduino-1.8.13.
# Too old: debian:buster, ubuntu:18.04, ubuntu:20.04
% docker run -ti -v $PWD:/usr/src:ro debian:testing

# apt update && apt install -y arduino python3 python3-serial

# arduino --install-library "PubSubClient:2.8"

# arduino \
  --pref boardsmanager.additional.urls=https://dl.espressif.com/dl/package_esp32_index.json \
  --install-boards "esp32:esp32"

# The esp32 package needs to be updated to use python3,
# as the python2 package python-serial is no longer in Debian.
# https://github.com/espressif/arduino-esp32/issues/4717
# sed -i 's,python ,python3 ,g' /root/.arduino15/packages/esp32/hardware/esp32/1.0.5/platform.txt

# arduino --install-library "TinyPICO Helper Library:1.3.0"

# cd /usr/src/tinypico-mqtt-nuki/
# arduino --verbose --verify --board esp32:esp32:tinypico *.ino
```

### qt-py-scs2uart/

```
# Only debian:testing is recent enough to provide arduino-1.8.13.
# Too old: debian:buster, ubuntu:18.04, ubuntu:20.04
% docker run -ti -v $PWD:/usr/src:ro debian:testing

# apt update && apt install -y arduino

# arduino --pref boardsmanager.additional.urls=https://www.adafruit.com/package_adafruit_index.json --install-boards "adafruit:samd"

# cd /usr/src/qt-py-scs2uart/
# arduino --verbose --verify --board adafruit:samd:adafruit_qtpy_m0 *.ino
```

### kicad-backpack/

A board that is mounted onto the [Adafruit QT
Py](https://www.adafruit.com/product/4600) (hence “backpack”) and adds 5V/3.3V
conversion for interfacing with a BTicino intercom unit (5V).
