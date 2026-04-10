# ESP32-C6 micro-ROS (ESP-IDF 5.5) – Working Setup Guide

## Environment

This setup **works inside a Python virtual environment (.venv)**.

Always activate it before doing anything:

```bash
source .venv/bin/activate
```

---

## ESP-IDF Setup

```bash
cd ~/esp/esp-idf-v5.5
. ./export.sh
```

Verify:

```bash
which idf.py
idf.py --version
```

---

## Go to Project

```bash
cd ~/mamri_build/Mamri_v6_PlatformIO
```

---

## IMPORTANT FIX (WiFi Crash)

Disable hardware SHA (this was causing WPA crash):

```bash
idf.py menuconfig
```

Navigate to:

```
Component config → mbedTLS → Hardware acceleration
```

Set:

```
[ ] Enable hardware SHA acceleration   ← DISABLED
[*] SHA1                               ← ENABLED
```

Verify later:

```bash
grep -E "MBEDTLS.*SHA|HARDWARE_SHA|SHA1" sdkconfig
```

Expected:

```text
CONFIG_MBEDTLS_SHA1_C=y
# CONFIG_MBEDTLS_HARDWARE_SHA is not set
```

---

## Clean Build

```bash
idf.py fullclean
idf.py build
```

---

## Flash

```bash
idf.py -p /dev/cu.usbserial-140 flash
```

---

## Monitor

```bash
idf.py -p /dev/cu.usbserial-140 monitor
```

Exit monitor:

```
Ctrl + ]
```

---

## Expected Output

* WiFi connects successfully
* IP is assigned
* micro-ROS initializes
* Publisher runs:

```text
micro_ros_task started
micro-ROS init complete
Publishing: 0
Publishing: 1
...
```

---

## Key Notes

* Everything runs inside `.venv`
* ESP-IDF must be sourced every new terminal
* Hardware SHA **must stay disabled** (critical fix)
* Same hotspot + config works after this fix

---

## Quick Full Workflow

```bash
source .venv/bin/activate

cd ~/esp/esp-idf-v5.5
. ./export.sh

cd ~/mamri_build/Mamri_v6_PlatformIO

idf.py fullclean
idf.py build
idf.py -p /dev/cu.usbserial-140 flash monitor


cd ~/esp/esp-idf-v5.5
. ./export.sh
cd ~/mamri_build/Mamri_v6_PlatformIO
idf.py build
idf.py -p /dev/cu.usbserial-140 flash monitor


cd ~/esp/esp-idf-v5.5
. ./export.sh
cd ~/mamri_build/Mamri_v6_PlatformIO
idf.py build
```

---

## Status

✔ WiFi stable
✔ micro-ROS working
✔ Publisher confirmed

```bash
ls /dev/cu.* 

cd ~/esp/esp-idf-v5.5
. ./export.sh
cd ~/mamri_build/Mamri_v6_PlatformIO
idf.py build
idf.py -p /dev/cu.usbmodem1401 flash monitor
```
