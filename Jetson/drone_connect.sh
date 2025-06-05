#!/bin/bash

TELLO_SSID="$1"
TELLO_IFACE="wlan1"
JETSON_IP="192.168.10.2"
DRONE_IP="192.168.10.1"
NETMASK="255.255.255.0"

#Check connection, if alread connected, dont reconnect
if ping -I "$TELLO_IFACE" -c 1 "$DRONE_IP" >/dev/null 2>&1; then
    echo "[INFO] Drone already reachable at $DRONE_IP on $TELLO_IFACE. Skipping connection setup."
    exit 0
fi

echo "Attempting to connect to Tello SSID: $TELLO_SSID on interface $TELLO_IFACE"
#disconnect wlan1 from any existing network
nmcli device disconnect $TELLO_IFACE
#scan ssid
echo "[INFO] Scanning for available Wi-Fi networks..."
AVAILABLE=$(nmcli -t -f SSID dev wifi list ifname $TELLO_IFACE | grep "^${TELLO_SSID}$")
if [ -z "$AVAILABLE" ]; then
    echo "[ERROR] SSID '$TELLO_SSID' not found."
    #exit 1
fi
# connect to the Tello SSID (no password)
nmcli dev wifi connect "$TELLO_SSID" ifname $TELLO_IFACE
sleep 3

#assign static IP to wlan1 for Tello network
echo "assigning static IP $JETSON_IP to $TELLO_IFACE..."
sudo ip addr flush dev $TELLO_IFACE
sudo ip addr add "$JETSON_IP/24" dev $TELLO_IFACE
sudo ip link set $TELLO_IFACE up

# Ping the Tello drone to confirm communication
echo "[INFO] Pinging Tello drone at $DRONE_IP..."
ping -c 3 "$DRONE_IP"

echo "Done. connected to $TELLO_SSID"
