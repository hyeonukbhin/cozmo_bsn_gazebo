# cozmo_gazebo (BSN is not yet intgrated)

cozmo croessed a gap using gazebo and rrbot

## Usage
* Launching rrbot simulation world
```
roslaunch rrbot_gazebo rrbot_3dof_world.launch
```
* Launching cozmo gazebo model
```
roslaunch cozmo_gazebo cozmo.launch 
```
* Launching cozmo & gazebo controller
```
rosrun cozmo_controller cozmo_controller.py
roslaunch gazebo_simple cozmo.launch 
```

## Configuration
* cozmo model profile
```
cozmo_bsn_gazebo/cozmo_gazebo/model/model.config
cozmo_bsn_gazebo/cozmo_gazebo/model/cozmo_model.sdf
```
* rrbot model profile
```
cozmo_bsn_gazebo/rrbot_gazebo/rrbot_gazebo/rrbot_control/worlds/rrbot_3dof.world
cozmo_bsn_gazebo/rrbot_gazebo/rrbot_gazebo/rrbot_description/urdf/rrbot_3dof.xacro
```
* rrbot contorl profile
```
cozmo_bsn_gazebo/rrbot_gazebo/rrbot_gazebo/rrbot_control/launch/rrbot_3dof_control.launch
cozmo_bsn_gazebo/rrbot_gazebo/rrbot_gazebo/rrbot_control/config/rrbot_3dof_control.yaml
```