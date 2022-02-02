## Drone with TOMBO propeller
The repository contains programs and ROS packages for controlling (s500) drone embedded with TOMBO propellers.

## Documentation
### Aerodynamic modeling


### Setup motion capture system (MOCAP):

Please be assure turn off Firewall for rigid-body data communication.

### Flight test - Trajectory Tracking:

1. Move to Catkin workspace (ROS) working directory.

```
$ cd [home/username]/catkin_ws/src
```
Then make the catkin environment and source the setup file.
```
$ catkin_make && source devel/setup.bash
```

2. Run vrpn node for subscribing drone pose (position and orientation) in real-time, pass the ip address of the computer running the MOCAP system to server argument.
```
$ roslaunch vrpn_client_ros sample.launch server:=192.168.11.13
```

3. Run mavros node to connect to the drone (PX4 flight stack) and Qground control

4. Relay orientation topic.

### Equilibrium Bounce Reaction Strategy:

## Contact

## Acknowledgments
This work was supported by JST SCORE project, Grant-in-aid for Scientific Research projects No. 18H01406 and 21H01287

## License
Distributed under the MIT License. See `LICENSE` for more information.


