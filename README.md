# Robot Project for OS 2025

## Build (Cross-Compiling with Docker)

### 1. Pull the Docker Image
```bash
sudo docker pull ev3dev/debian-stretch-cross
```

### 2. Run the Docker Image and Mount the Project Directory

Replace `<project path>` with the absolute path to your project folder.

```bash
sudo docker run \
    -e LOCAL_USER_ID=id \
    -u $USER \
    --rm -it \
    -v <project path>:/src \
    -w /src \
    ev3dev/debian-stretch-cross
```

### 3. Build the Project

Inside the Docker container, run:

```bash
cd ev3dev-c/source/ev3/ && make && sudo make install && make shared && sudo make shared-install
cd ../../../ && make
```

---

## Running the Program

### Transfer Executable to Robot

```bash
scp <executable> robot@<host>:<output directory>
./<executable>
```

## To enable internet connectivity via local USB (ethernet loopback) connection on Linux

### On Dev machine
```bash
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
sudo iptables -A FORWARD -i eth0 -o eth1 -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i eth1 -o eth0 -j ACCEPT
```

eth0: outgoing interface to internet on host
eth1: interface with robot

### On robot
Set static IP, correct netmask and gateway IP to eth1 IP. Might have to change DNS in /etc/resolv.conf.
..

