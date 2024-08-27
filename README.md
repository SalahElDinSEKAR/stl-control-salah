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

copy the content from **2_PID.txt** to **sim_breach_watertank.m** if they are different

**sim_breach_watertank.m** exists inside **useful code**

**2_PID.txt** exists inside **PidWatetank_Model**

i have added another control function for the fast PID: **pid_watertank_control.m**

and the slow PID is: **pid_watertank_control_S.m**

there is a new script that calls the 2 PIDs: **final_control.m**

this funtion shuld be called so there is another change that should be made

in the file **PidWatertank_imitation_pb.m**

make sure tha the final_control function is selected

```pb.mpc_controlfn = @(H) final_control(H);```

now that all is set you can try to verify with 
**test_pid_watertank.m**