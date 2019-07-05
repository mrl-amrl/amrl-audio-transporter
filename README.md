## Audio Transporter ROS Nodes
ROS nodes that provides audio-type topics.

#### Installation

```
$ sudo apt install libjack-dev
$ sudo apt install jack jackd
```

#### Running JACK server

```
$ jackd -R -P4 -dalsa -r44100 -p512 -n4 -D -Chw:0 -Phw:0
```
