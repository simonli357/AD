# Useful commands for 2026

## Jetson IPs

- BoschFMC: `192.168.50.110`
- hotspot: `10.89.16.119`

## Command laptop IP (Dell)

- BoschFMC: `192.168.50.175`
- hotspot: `10.89.16.85`

## SSH to Jetson over USB-C

```bash
ssh scandy@192.168.55.1
```

## Wi-Fi

Make sure the Wi-Fi radio is on:

```bash
nmcli radio wifi on
```

Scan for available networks:

```bash
nmcli device wifi list
```

Connect to a network:

```bash
sudo nmcli device wifi connect "BoschFMC" password "bosch23581321"
sudo nmcli device wifi connect "slsecret357" password "simonli357"
```

Create a persistent connection:

```bash
sudo nmcli connection modify "BoschFMC" connection.autoconnect yes
sudo nmcli connection modify "slsecret357" connection.autoconnect yes
```

Show the Jetson IP:

```bash
ip a show wlan0
```

## Jetson performance

```bash
sudo nvpmodel -m 0
sudo jetson_clocks
```

## Monitor serial

```bash
sudo minicom -D /dev/ttyACM0 -b 115200D
```

If the device changed:

```bash
sudo minicom -D /dev/ttyACM1 -b 115200D
```

## Controller launch

```bash
roslaunch control controller.launch camera:=true ip:=192.168.50.175 use_gps:=false real:=true realsense:=true
```

With traffic and GPS:

```bash
roslaunch control controller.launch camera:=true ip:=192.168.50.175 use_gps:=true real:=true realsense:=true use_traffic_server:=true gps_points:=25 gps_id:=10
```

## GUI

```bash
rosrun gui main.py
```
