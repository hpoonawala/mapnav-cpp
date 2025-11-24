# Mapping and Navigation in C++

This repo contains `C++` code for navigation using 2D LiDAR measurements only. Code was created with a lot of help from Claude. 

It contains implementations for scan matching, SLAM, path planning, pure-pursuit based tracking, and a reactive collision avoidance controller. As a by-product, it contains a wrapper around the `rplidar_sdk`. These implementations have an ancestry rooted in python versions for navigation simulations using `MuJoCo`. 

The aim is to demonstrate autonomous movement through an unmapped environment and to compute a map. The results are mixed. LiDAR artifacts and high angular velocities introduce mapping errors that would require human correction. For runs where these issues do not arise, the map and the planned path are quite good. Unfortunately, the current implementation does not run fast enough on a Rasberry Pi 4 to enable long-term navigation unless one tolerates a stop-motion behavior. These limitations suggest what one would expect:

- Use a more powerful processor 
- Use optimized pre-existing libraries where possible 

One lesson that may be true but is not yet tested is that trying to avoid wheel encoders and IMU was always a losing battle. 

The project allowed me to better understand the intricacies of scan matching and SLAM. A lot of details turn out to be important. It also improved my understanding of 'real-world' programming a great deal. A nice outcome is that I don't have to worry about incompatible Ubuntu and ROS versions.


## Compile 

Run the `Makefile`, potentially modifying it to point to the correct locations of the `rplidar_sdk`, `Eigen`, and `asio` repos. 

## Usage 

Run 
```
./lidar.exe --channel --serial /dev/ttyUSB0 115200
```
where the last two options are the USB port and baud rate of a companion controller that accepts a structured message over serial containing desired body velocities as integers. 

The companion controller stops the wheels if a new velocity does not arrive within some pre-defined timeout period (~$150$-$250$ ms). 

To run a purely reactive controller -- no scan match, no map, therefore no planned path to follow -- run 
```
./lidar.exe --channel --serial /dev/ttyUSB0 115200
```

## Modifications 

There is a lot of technical debt in this code. 

One easy modification is to enable use of a Nelder-Mead optimization to search for the angular offset before scan-matching. This optimization took $100$ ms on the RPi4, but it improved the scan matching results significantly, especially under higher angular velocities demanded by collision avoidance. 


