# USV_surge_velocity_controller
A discrete PID controller for surface vessel surge control



![blockdiagram1 drawio](https://user-images.githubusercontent.com/5917472/219367004-c423d82e-5afd-495f-9f82-c43067d474b2.svg)


## Notes
- publishes at a fixed rate
- Basic PID controller
- Output is limited between configurable upper and lower bounds
- Does not do any thrust effort allocation. It attempts to control vessel surge speed by means of the main propellers, which are assumed to point approximately backwards, or at least with relatively small angles (e.g. if main thrusters are 90degrees rotated sidways this controller will lose ability to correct surge speed to reference). 
- Only relevant fields in the actuation array are given values. Other values (e.g. thruster angles) are not broadcasted, but left as empty fields. These angle refences should come from another source (e.g. a heading controller, that leaves null at vector indices referring to propeller velocities)

## Startup
Make sure you have the right repo branch for ros1 or ros2

### ROS 1
Source ros1 and run the python script specifying the vessel identifier
```
source ${ROS1_INSTALL_PATH}/setup.bash
python3 surgevelocitycontroller1.py 'RAS_TN_DB'
```

### ROS 2
Not developed yet
