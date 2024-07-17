192.168.50.206
ip at home "192.168.68.68"

# TODO
- Somehow deted the tennis ball is grabbed
- Fine tune the PIDs
- Make sure data is being transferred at constant rate.
- Detect we are stuck

# checklist:
- check if the arm goes high enough and not more than enough

# Clipboard

```bash
sudo systemctl restart nvcortexnano.service
```


# Docker commands

```bash
# build
docker build -f host.Dockerfile -t vexcompetition .

# develop
xhost + \
&& docker run \
  --name vexcompetiton \
  --runtime nvidia \
  --rm -it \
  -v `realpath ..`:`realpath ..` \
  --network host \
  --gpus all \
  --privileged \
  -v /dev/bus/usb/:/dev/bus/usb/ \
  -e DISPLAY \
  -v /dev/input:/dev/input \
  vexcompetition \
  bash -c "cd `realpath .` && bash"
```

# For Demo day

1. Start the container
2. Figure out the jetson ip and the service port and set it in `constant.py`.
3. re-plug the camera
3. `ssh user@<jetson ip>` and `sudo systemctl restart nvcortexnano.service`
4. Test with manual control
```bash
# test_manual_control
pytho robot_interface_node.py
# or
python practices/robot.py
```
5. For autonomous system we can run
```bash
python planning_and_control.py
```

# On Jetson
```
sudo iw dev wlan0 set power_save off
iw dev wlan0 get power_save
sudo systemctl restart nvcortexnano.service
```


To get the intrinsic parameters of the camera
```bash
./rs-enumerate-devices -c
```

## Setup connection on jetson

Connect the USB cable
find the ip through USB

```bash
ifconfig
# read usb0 ip and ssh via that one
ssh user@192.168.55.1
# if issues with ssh keys, runs the given commands in the output of above command

# setup wifi
nmcli c # list of saved connections
nmcli d wifi list # list of wifi hotspots

# or use nmtui
sudo nmcli d connect wlan0 # search for new wifi
sudo nmtui
```

### wifi powermode

https://github.com/robwaat/Tutorial/blob/master/Jetson%20Disable%20Wifi%20Power%20Management.md

```
sudo iw dev wlan0 set power_save on
sudo iw dev wlan0 set power_save off

```

# April Tags

https://github.com/AprilRobotics/apriltag#coordinate-system

The coordinate system has the origin at the camera center. The z-axis points from the camera center out the camera lens. The x-axis is to the right in the image taken by the camera, and y is down. The tag's coordinate frame is centered at the center of the tag, with x-axis to the right, y-axis down, and z-axis into the tag.

## Images:
Tag Family: tag16h5
https://github.com/AprilRobotics/apriltag-imgs


# Links
- Wifi power mode: https://github.com/robwaat/Tutorial/blob/master/Jetson%20Disable%20Wifi%20Power%20Management.md

# VexCortex Connections:

## Sensors
- ArmAngle: in1       [2542, 3305]
- Switch 1: digital 1
- Switch 2: digital 2

## Actuators
- Left Motor: port 1
- Right Motor: port 10
- Claw Motor: port 8
- Arm Motor: port 9

## Camera calibration


### Versions

/vexbot/CortexNanoBridge:
0c97988b1f8e4ddbcf1f2c356a694dc102c7a4ef

e3ed77a93e11b9d8ce0bf646e9b41bfa9ca264c9

400a358a3975230e827a3d14d03bbd82df81525f


## Bluetooth

```bash
sudo bluetoothctl
scan on
trust 00:04:4B:94:76:D3
pair 00:04:4B:94:76:D3
connect 00:04:4B:94:76:D3
bluetoothctl connect 00:04:4B:94:76:D3

hcitool -i hci0 scan

```