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
nmcli -f IN-USE,BSSID,SSID,CHAN,FREQ,SIGNAL device wifi list
```

Connect to a network:

```bash
sudo nmcli device wifi connect "BoschFMC" password "bosch23581321"
sudo nmcli device wifi connect "slsecret357" password "simonli357"
```

Switch between already-saved Jetson Wi-Fi profiles:

```bash
# Use this over USB SSH: ssh scandy@192.168.55.1
sudo nmcli con up BoschFMC
sudo nmcli con up slsecret357
```

Create persistent connections and force BoschFMC to 2.4 GHz:

```bash
sudo nmcli connection modify "BoschFMC" connection.autoconnect yes 802-11-wireless.band bg
sudo nmcli connection modify "slsecret357" connection.autoconnect yes
sudo nmcli connection down "BoschFMC"
sudo nmcli connection up "BoschFMC"
```

If BoschFMC has multiple 2.4 GHz access points, pick the 2.4 GHz BSSID from the scan output (channels 1-14) and pin it:

```bash
sudo nmcli connection modify "BoschFMC" 802-11-wireless.bssid AA:BB:CC:DD:EE:FF
sudo nmcli connection up "BoschFMC"
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
roslaunch control controller.launch camera:=true ip:=10.89.16.85 use_gps:=false real:=true realsense:=true
```

With traffic and GPS:

```bash
# Traffic server is on BoschFMC; confirm the laptop IP is 192.168.50.175 first.
# Probe first. Use the localization tag ID that returns live location data.
scripts/traffic_server_probe.py --discover --loc-id <LIVE_ID> --timeout 5

roslaunch control controller.launch camera:=true ip:=192.168.50.175 use_gps:=true real:=true realsense:=true use_traffic_server:=true gps_points:=25 gps_id:=3
```

Arena 
```bash
roslaunch control controller.launch camera:=true ip:=192.168.50.175 use_gps:=false real:=true realsense:=true use_traffic_server:=true  gps_id:=3 path:=run_arena do_parking:=false
```

## GUI

```bash
rosrun gui main.py
```

- streaming computer:
```bash
rosrun gui main.py -ip 192.168.50.175 # boschfmc
rosrun gui main.py -ip 10.89.16.85 # hotspot
```

