# Persistent names for usb-serial devices (example for rplidar device).

Get Vendor ID and Product ID.

```shell
ubuntu@robot:~$ lsusb
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 004: ID 17ef:6019 Lenovo 
Bus 001 Device 003: ID 045e:07b9 Microsoft Corp. 
Bus 001 Device 006: ID 10c4:ea60 Cygnal Integrated Products, Inc. CP210x UART Bridge / myAVR mySmartUSB light
Bus 001 Device 002: ID 2109:3431 VIA Labs, Inc. Hub
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
```

If more than one device on a bus. Pair it with some other unique attribute.

```shell
ubuntu@robot:~$ ls /dev | grep ttyUSB
ttyUSB0
ubuntu@robot:~$ udevadm info -a -n /dev/ttyUSB0
```

For example:

```shell
ATTRS{manufacturer}=="Silicon Labs"
ATTRS{product}=="CP2102 USB to UART Bridge Controller"
ATTRS{serial}=="0001"
```

Create a new UDEV rule. Make a new file `99-usb-serial.rules` inside `/etc/udev/rules.d`. Under root.

```shell
root@robot:~# nano /etc/udev/rules.d/99-usb-serial.rules
```
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar" 
```

Reload rules and check the device.

```shell
root@robot:~# udevadm control --reload-rules && udevadm trigger
root@robot:~# ls -l /dev/lidar 
lrwxrwxrwx 1 root root 7 Sep 16 11:58 /dev/lidar -> ttyUSB0
```

# Chrony

RPI side:

```shell
ubuntu@robot:~$ sudo apt-get install chrony
root@robot:~# systemctl enable --now chrony
root@robot:~# nano /etc/chrony/chrony.conf
allow 192.168.88.0/16
root@robot:~# systemctl restart chronyd
```

Remote machine side:

```shell
root@robot-user:~# sudo apt-get install chrony
root@robot-user:~# ntpdate -q 192.168.88.94
root@robot-user:~# nano /etc/chrony/chrony.conf
server 192.168.88.94 minpoll 0 maxpoll 5 maxdelay .05
root@robot-user:~# /etc/init.d/chrony stop && ntpdate 192.168.88.94 && /etc/init.d/chrony start
root@robot-user:~# chronyc sources -v
```

# Rsync

```shell
people@robot-user:~/ROS_TEMP$ rsync -avz -e ssh ubuntu@robot:ROS/abot/ abot/
```

RPI
```shell
ubuntu@robot:~$ cd ROS/abot
ubuntu@robot:~$ rsync -avz -e ssh people@robot-user:ROS_DEV/abot/src src
```

# RPI Headless Boot without HDMI

```shell
ubuntu@robot:~$ sudo nano /boot/firmware/config.txt
hdmi_force_hotplug=1
```

# ROS manual package install

Exmaple, melodic package to noetic: 
```shell
root@robot-user:/home/people/ROS_PACKAGES# source /opt/ros/noetic/setup.bash
root@robot-user:/home/people/ROS_PACKAGES# catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic install
root@robot-user:/home/people/ROS_PACKAGES# rospack find gmapping
```

# ROS map saver

```shell
rosrun map_server map_saver -f src/abot_slam/map/map1
```