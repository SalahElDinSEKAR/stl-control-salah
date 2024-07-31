# stl-imitation-control

The code is organized between shared_code folder for everything comon and 
folders specific to each case study. 

## Pre-requisite

- update breach-dev to the latest commit
- modify init_paths.m to fix the path to breach-dev on your system.

## Vehicle example

To reproduce results for the vehicle case study:
```matlab
>> init_vehicle;
>> res = pb_vehicle.algo1();
```
