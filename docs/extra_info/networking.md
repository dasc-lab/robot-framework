---
layout: default
title:  "Networking"
date:   2024-06-01
math: katex
has_children: false
has_toc: true
nav_order: 4
---
# Networking Tips Tricks and Solutions

All of these details are assuming linux/ubuntu. 

## `nmcli`: Managing Connections

Awesome tool to be able to see and manage wifi networks. 

**To list** available wifi networks, and their strengths:
```
nmcli dev wifi
```

**To connect** to a specific network:
```
nmcli device wifi connect "$SSID" password "$PASSWORD"
nmcli --ask device wifi connect "$SSID"
```
The second one will prompt you for a password if necessary.


**To get details** on the networking devices on your computer:
```
nmcli -p -f general,wifi-properties device show
```

**To install** `nmcli`:
```
sudo apt-get update && sudo apt-get install network-manager
```
To enable it without rebooting:
```
systemctl start NetworkManager.service 
```
or to enable on boot:
```
systemctl enable NetworkManager.service
```

Reference: https://man.archlinux.org/man/nmcli-examples.7.en

## `ip`, `ifconfig`, `iwconfig`: Details on current network

To get the computers ip or mac addresses, run any of 
```
ip a
ifconfig # all network interfaces
iwconfig # specifically for wireless
```


## `iperf3`: Bandwidth Measurement

`iperf3` is a tool to measure wifi strength and bandwidth. 

**To install**:
```
sudo apt-get install iperf3
```
**To test** the connection speed between two computers, you will start the first computer as a server:
```
iperf3 -s -f m
```
Flags:
- `-s` enables the server mode
- `-f K` sets the format to KBytes. You can also use `k, m, g, K, M, G` for kbits, mbits, gbits, kbytes, mbytes, gbytes respectively. 
- `-p 3000` sets the port to 3000 (default is 5201)

Then on the client (i.e.,  the machine where the actual benchmarking takes place), run 
```
iperf3 -c [server_hostname/ip_address] -f m
```
Flags:
- `-c $SERVER_IP` enables client mode, and specifies the server's ip address. 
- `-f` is as before
- `-w 500K` will set a TCP window size
- `-R` will test it in reverse mode
- `-d` will test it both directions simultaeneously
- `--udp` will test UDP instead of TCP

Reference: https://www.tecmint.com/test-network-throughput-in-linux/

## `ping`: Measuring Latency

Run
```
ping [hostname/ip_address]
```
or specify whichever ip address you want to measure the latency of. Pay attention to the summary outputs at the end to see a spread.
Flags:
- `-i [interval]` the interval in seconds at which you want to run ping. Important to set to a small number like 0.2 to avoid network cards intermittantly going to sleep and artificially resulting in a large ping value. 
- `-s [size]` to specify the size (in bytes) of each packet
- `-f` to test it in flood mode. Might need to run in `sudo` mode, and the results are printed when you hit `ctrl-c`

## `nmap`: scan the local network for other devices

Assuming you are on a local network with ip address `192.168.1.144` you can scan others by running 
```
sudo nmap -sn 192.168.1.*
```
which will search for all computers with ip addresses of the form `192.168.1.*`

Reference: https://phoenixnap.com/kb/how-to-install-use-nmap-scanning-linux




