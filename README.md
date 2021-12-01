# bwt901cl_pkg

This package publishes data from BWT901CL, an IMU sensor, into ROS2.

### contents

- package: bwt901cl_pkg
    - node: imu_bwt901cl(publisher type)
    - msg:
        - Imu: '/sensor/bwt901cl/imu'
        - MagneticField: '/sensor/bwt901cl/MagneticField'
        - Temperature: '/sensor/bwt901cl/Temperature'
        - Vector3: '/sensor/bwt901cl/Angle'

### build

```
colcon build --packages-select bwt901cl_pkg
```

### running

```
ros2 run bwt901cl_pkg imu_bwt901cl
```

### Warning

- Your user should belong to `dialout` group, otherwise you may not have access to device `/dev/ttyUSB*`．

# Further reading

https://github.com/WITMOTION/BWT901CL

https://github.com/ros2/common_interfaces

https://sgrsn1711.hatenablog.com/entry/2018/02/14/204044