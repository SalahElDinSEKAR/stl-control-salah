# stl-imitation-control

The code is organized between shared_code folder for everything comon and 
folders specific to each case study. 

## Pre-requisite

- update breach-dev to the latest commit
- modify init_paths.m to fix the path to breach-dev on your system.
- download the git for the simulation using ROS2

    ```https://github.com/SalahElDinSEKAR/ros2-matlab.git```

## Vehicle example

To reproduce results for the vehicle case study:
```matlab
>> init_vehicle;
>> res = pb_vehicle.algo1();
```
## main function

in the **PidWatetank_Model** folder we have:

**sim_breach_watertank.m**

this script contains the main simulation function. If we want to work with combing PIDs this fnction will have different algorithm than the one with ros2 simulation 

## combining PID

- copy the content from **2_PID.txt** to **sim_breach_watertank.m** if they are different

**sim_breach_watertank.m** exists inside **useful code**

**2_PID.txt** exists inside **PidWatetank_Model**

i have added another control function for the fast PID: **pid_watertank_control.m**

and the slow PID is: **pid_watertank_control_S.m**

there is a new script that calls the 2 PIDs: **final_control.m**

this funtion shuld be called so there is another change that should be made

in the file **PidWatertank_imitation_pb.m**

- make sure tha the final_control function is selected

```pb.mpc_controlfn = @(H) final_control(H);```

- now that all is set you can try to verify with 
**test_pid_watertank.m**

- to train the neural network in launch ```test_watertank.m```. the function responsible for training is algo2


## ROS2

- download ROS2 humble and source the environment

- add those line to **.bashrc**
```
export ROS_LOCAL_HOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/Documents/stage/ros2-matlab/install/setup.bash
```
instead of /Documents/stage/ros2-matlab choose the path of the project you created 

- download the git for ROS2 that i mentioned in the beginning

- copy the content from **ros2_sim.txt** to **sim_breach_watertank.m** if they are different

**sim_breach_watertank.m** exists inside **useful code**

**ros2_sim.txt** exists inside **PidWatetank_Model**

- inside of the function **launch_nodes.m**
change the path of all files to the paths on you pc

- there is no need to change the function of control because the simulation will happen outside of matlab

- test with ```test_pid_watetatnk.m```

### Note

- the **bag_to_csv.py** can be replaced with ros2 command line:

`rostopic echo -b my_bag_file.bag -p /my_topic>my_csv_file.csv`

- the function for downsampling takes from teh data the useful infromation for example here:
```
downsampling_factor = 10;

for i = 1:downsampling_factor:1010
```
we only take the first 1010 sample and we are taking 1 sample from each 10 

so we can change this in the way we see it the best to maintain valuable infromation 

we should also be careful to the time step taken in ROS2 and the integration time . i was testing lot of stuff now it is a bet messed up if you want TF=20 with Ts=0.1


