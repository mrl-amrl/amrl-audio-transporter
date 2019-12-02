## Audio Transporter ROS Nodes
ROS nodes that provides audio-type topics. It's a fork of `rosjack` ROS package.

#### Installation

```
$ sudo apt install libjack-dev
$ sudo apt install jack jackd
```

#### List of your sound cards

```
$ cat /proc/asound/cards
```

#### Running JACK server

```
$ jackd -R -P1 -dalsa -p512 -n4 -D -Chw:0 -Phw:0
```
