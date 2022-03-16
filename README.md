# AutonomousRobot
This project simulates our path-planning for an autonomous robot that tries to find a way to reach a goal (target)
in certainly environment 
##### Usage:
``` python Robot_main.py -n <number of run times> -m <map name> -w <worldname> -r vision_range -sx <x> -sy <y> -gx <x> -gy <y> ```

* n: number of run times
    - < 0 or 0: run until meet the given goal
    - n: number of run times
    - default: 1
* m: input map name, default _map.csv
* w: input world model, no default.
* r: robot's vision range.
* sx, sy: start point of (x,y), type = float, default = 0.0, 0.0
* gx, gy: goal point of (x,y), type = float, default = 50.0, 50.0

Example: 
```
python Robot_main.py -n 5 -m _MuchMoreFun.csv -sx 5 -sy 5 -gx 35.0 -gy 50.0
python Robot_main.py -n 0 -m _map.csv -sx 5 -sy 5 -gx 35.0 -gy 50.0
python Robot_main.py -n 0 -w _world.png -sx 5 -sy 10 -gx 180.0 -gy 180
```
##### To run demo for the the assumption of An and Hoai's Theory:
``` 
python Robot_theory.py -n 0 -r 100 -m _forest.csv -gx 500 -gy 500
python Robot_theory.py -n 0 -r  90 -m _forest.csv -gx 500 -gy 500
python Robot_theory.py -n 0 -r  80 -m _forest.csv -gx 500 -gy 500
```
* Set robot_vision parameter (option -r ) to see the different outcomes of experiments

##### To generate a map:
Usage:

``` python map_generator.py -n <number of obstacles> -m <map name> -img <from_world_image>```

* mn: input map name, default _map_temp.csv
* n: number of obstacles.

Example 1 (generating map from user input):  

``` python map_generator.py -n 5 -m _map_temp.csv  ```
- Click on the given plot to input points
- Middle mouse click to turn next obstacle. Each obstacle contains a maximum of 100000 vertices

<img src="https://github.com/ThanhBinhTran/autonomousRobot/blob/main/Map_generator/map_display_user_input_demo.png" width="150" alt="world image">

Example 2 (generating map from image):

``` python map_generator.py -img _world.png ```

From world image <img src="https://github.com/ThanhBinhTran/autonomousRobot/blob/main/Map_generator/_world.png" width="150" alt="world image"> to map data (csv) <img src="https://github.com/ThanhBinhTran/autonomousRobot/blob/main/Map_generator/map_display_world_demo.png" width="150" alt="map data csv">



##### To display a map: 
``` python map_display.py -m <map name> ```

Example: ``` python map_display.py -m _map_temp.csv ```