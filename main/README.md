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

>

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
idf.py -p /dev/cu.usbserial-140 flash monitor


idf.py -p /dev/cu.usbserial-1140 flash monitor
```



Save the data:

```bash

cd ~/esp/esp-idf-v5.5
. ./export.sh
cd ~/mamri_build/Mamri_v6_PlatformIO                                            

idf.py -p /dev/cu.usbserial-140 flash                                             
mkdir -p logs
idf.py -p /dev/cu.usbserial-140 monitor | tee logs/raw.txt


grep 'DATA_CSV' logs/raw.txt | sed -E 's/^.*DATA_CSV: //' > logs/data_joint1.csv
wc -l logs/data_joint1.csv
head -n 5 logs/data_joint1.csv
```

Motion path CSV:

```bash
cd ~/esp/esp-idf-v5.5
. ./export.sh
cd ~/mamri_build/Mamri_v6_PlatformIO

idf.py -p /dev/cu.usbserial-140 flash
mkdir -p logs
idf.py -p /dev/cu.usbserial-140 monitor | tee logs/raw.txt
```

In another terminal, or after exiting monitor with `Ctrl + ]`, extract the motion path data:

```bash
mkdir -p logs
printf 'time_ms,sample,event,control_mode,waypoint,total_waypoints,joint,q_ol,q_ref,remaining_ol,ferris_valid,ferris_raw_deg,ferris_tared_deg,ferris_scale,ferris_joint_deg,q_sensor,sensor_target_error,slip_error,q_cmd,cl_tolerance,needs_correction,correction_attempt,joint_velocity,joint_setpoint_velocity,joint_max_velocity\n' > logs/motion_path.csv
grep 'MOTION_CSV' logs/raw.txt | sed -E 's/^.*MOTION_CSV: //' >> logs/motion_path.csv

wc -l logs/motion_path.csv
head -n 5 logs/motion_path.csv
tail -n 5 logs/motion_path.csv
```

Extract Ferris wheel raw/degree samples:

```bash
printf 'time_ms,joint,absolute_raw_counts,mechanical_zero_raw_counts,calibration_valid,angle_deg,calibrated_deg,tared_deg,ferris_scale,joint_deg\n' > logs/ferris_values.csv
grep 'FERRIS_CSV' logs/raw.txt | sed -E 's/^.*FERRIS_CSV: //' >> logs/ferris_values.csv

wc -l logs/ferris_values.csv
head -n 5 logs/ferris_values.csv
tail -n 5 logs/ferris_values.csv
```

Optional: split one CSV per joint:

```bash
for j in 0 1 2 3 4; do
  awk -F, -v joint="$j" 'NR==1 || $7==joint' logs/motion_path.csv > "logs/motion_path_joint${j}.csv"
done

wc -l logs/motion_path_joint*.csv
```

Useful quick checks:

```bash

# Final measured error at every reached waypoint.
awk -F, 'NR==1 || $3=="waypoint_reached" {print}' logs/motion_path.csv > logs/motion_waypoints.csv

# Path-finish rows only.
awk -F, 'NR==1 || $3=="path_finished" {print}' logs/motion_path.csv > logs/motion_finished.csv

# Worst absolute slip and sensor-target error per joint.
awk -F, '
NR>1 {
  joint=$7
  sensor=$17+0
  slip=$18+0
  abs_sensor=(sensor<0?-sensor:sensor)
  abs_slip=(slip<0?-slip:slip)
  if (abs_sensor > max_sensor[joint]) max_sensor[joint]=abs_sensor
  if (abs_slip > max_slip[joint]) max_slip[joint]=abs_slip
}
END {
  for (j=0; j<=4; ++j) {
    printf "joint[%d] max_abs_sensor_target_error=%d max_abs_slip_error=%d\n", j, max_sensor[j], max_slip[j]
  }
}' logs/motion_path.csv
```

```bash
ls /dev/cu.* 

cd ~/esp/esp-idf-v5.5
. ./export.sh
cd ~/mamri_build/Mamri_v6_PlatformIO
idf.py build
idf.py -p /dev/cu.usbserial-140 flash monitor


idf.py -p /dev/cu.usbserial-1140 flash monitor
```


```bash
cd ~/esp/esp-idf-v5.5
. ./export.sh
cd ~/mamri_build/Mamri_v6_PlatformIO

idf.py -p /dev/cu.usbserial-140 flash
mkdir -p logs
idf.py -p /dev/cu.usbserial-140 monitor | tee logs/raw.txt
```

```bash
printf 'time_ms,sample,event,control_mode,waypoint,total_waypoints,joint,q_ol,q_ref,remaining_ol,ferris_valid,ferris_raw_deg,ferris_tared_deg,ferris_scale,ferris_joint_deg,q_sensor,sensor_target_error,slip_error,q_cmd,cl_tolerance,needs_correction,correction_attempt,joint_velocity,joint_setpoint_velocity,joint_max_velocity\n' > logs/motion_path.csv
grep 'MOTION_CSV' logs/raw.txt | sed -E 's/^.*MOTION_CSV: //' >> logs/motion_path.csv

wc -l logs/motion_path.csv
head -n 5 logs/motion_path.csv
tail -n 5 logs/motion_path.csv
```

printf 'time_ms,sample,event,control_mode,waypoint,total_waypoints,joint,q_ol,q_ref,remaining_ol,ferris_valid,ferris_raw_deg,ferris_tared_deg,ferris_scale,ferris_joint_deg,q_sensor,sensor_target_error,slip_error,q_cmd,cl_tolerance,needs_correction,correction_attempt,joint_velocity,joint_setpoint_velocity,joint_max_velocity\n' > logs/motion_path.csv
grep 'MOTION_CSV' logs/raw.txt | sed -E 's/^.*MOTION_CSV: //' | perl -pe 's/\e\[[0-9;]*m//g' >> logs/motion_path.csv