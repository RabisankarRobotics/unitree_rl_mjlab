#!/bin/bash

start_can() {
    local iface=$1
    local bitrate=$2

    if ip link show "$iface" 2>/dev/null | grep -q "state UP"; then
        echo "$iface is already up"
    else
        echo "Bringing up $iface..."
        sudo ip link set "$iface" up type can bitrate "$bitrate"
    fi
}

start_can can0 1000000
start_can can1 1000000