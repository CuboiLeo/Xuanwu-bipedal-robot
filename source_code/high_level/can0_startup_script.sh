#!/bin/bash

sudo modprobe can_raw # load the can_raw module
sudo modprobe mttcan # load the mttcan module
sudo ip link set can0 up type can bitrate 1000000 # start the can0 interface with a bitrate of 1Mbit/s

sudo ip link set can0 txqueuelen 2000 # set the txqueuelen of the can0 interface to 2000

sudo sysctl -w net.core.rmem_max=262144 # set the maximum receive buffer size to 262144
sudo sysctl -w net.core.wmem_max=262144 # set the maximum send buffer size to 262144


