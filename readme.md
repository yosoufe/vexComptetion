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
  --rm -it \
  -v `realpath ..`:`realpath ..` \
  --network host \
  --gpus all \
  --privileged \
  -v /dev/bus/usb/:/dev/bus/usb/ \
  -e DISPLAY \
  vexcompetition \
  bash -c "cd `realpath .` && bash"
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


# April Tags

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

[[ 0.0387005  -0.99832564  0.0417403   0.        ]
 [-0.42239487 -0.05450909 -0.90101783  0.        ]
 [ 0.90558536  0.01723895 -0.43176918  0.        ]
 [ 0.          0.          0.          0.        ]]

camera to robot transformation: 
 [[ 0.04183374 ,-0.40959374 , 0.91130835 ,-0.33556501]
 [-0.99828073 ,-0.05146286,  0.02269589,  0.10270806]
 [ 0.03691244 ,-0.90393521 ,-0.42607349 , 0.2442368 ]
 [ 0.          ,0.         , 0.         , 1.        ]]
robot to camera transformation: 
 [[ 0.04183374 ,-0.99828073 , 0.03691244  ,0.1076191 ]
 [-0.40959374, -0.05146286, -0.90393521 , 0.08528469]
 [ 0.91130835 , 0.02269589 ,-0.42607349  ,0.40161486]
 [ 0.         , 0.         , 0.          ,1.        ]]


3rd try

camera to robot transformation: 
 [[ 0.04892364 -0.41772631  0.90725477 -0.32358759]
 [-0.99831263 -0.0435315   0.03379077  0.09919108]
 [ 0.02448331 -0.89973047 -0.43575871  0.24580773]
 [ 0.          0.          0.          1.        ]]
robot to camera transformation: 
 [[ 0.04892364 -0.99831263  0.02448331  0.10879795]
 [-0.41772631 -0.0435315  -0.89973047  0.08688687]
 [ 0.90725477  0.03379077 -0.43575871  0.39080503]
 [ 0.          0.          0.          1.        ]]
