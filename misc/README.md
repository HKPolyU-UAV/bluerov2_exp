# Patty Notes Here

# Build Firmware from Source

```
./waf configure --board Pixhawk1
./waf -j7 sub
```
The ```ardusub.apj``` file should be under ```~/home/<user>/ardupilot/build/Pixhawk1/bin```; you can then flash this to the onboard controller.

# Stream Rate
under ->
```
SR1_{}
```

# On Eigen with Raspberry Pi
```
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```

# SSH Service
```
service ssh start
```

# check output
```
rostopic echo /mavros/rc/out
# within QGC -> SERVO_OUTPUT_RAW

```

# ardu parameter for vision (tested)
![alt text](image.png)

# px4 parameter for vision (not tested, wait for heavy)
![alt text](image-1.png)