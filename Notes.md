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

# NTP server RPI4

RPI side:

```shell
ubuntu@robot:~$ sudo apt-get install ntp
root@robot:~# nano /etc/ntp.conf 
```
```bash
pool 0.ru.pool.ntp.org iburst
pool 1.ru.pool.ntp.org iburst
pool 2.ru.pool.ntp.org iburst
pool 3.ru.pool.ntp.org iburst

restrict 127.0.0.1
restrict ::1

restrict 192.168.88.0 mask 255.255.255.0 nomodify notrap
```

```shell
root@robot:~# systemctl restart ntp || service restart ntp
root@robot:~# systemctl status ntp || service status ntp
root@robot:~# ntpq -p
```

Remote machine:

```shell
people@robot-user:~$ sudo ntpdate 192.168.88.94
```