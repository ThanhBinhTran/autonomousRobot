# autonomousRobot
This project is to simulate an autonomousRobot :car: that try to find a way to reach a goal

##### To run demo:
```bash
python Robots.py -n <number of run times> -m <map name> -w <worldname> -sx <x> -sy <y> -gx <x> -gy <y>
```
* n: number of run times
    - < 0 or 0: run until meet the given goal
    - n: number of run times
    - default: 1
* m: input map name, default _map.csv
* w: input world model, no default.

* sx, sy: start point of (x,y), type = float, default = 0.0, 0.0
* gx, gy: goal point of (x,y), type = float, default = 50.0, 50.0

Example: 
```bash
python Robots.py -n 5 -m _MuchMoreFun.csv -sx 5 -sy 5 -gx 35.0 -gy 50.0
python Robots.py -n 0 -m _map.csv -sx 5 -sy 5 -gx 35.0 -gy 50.0
python Robots.py -n 0 -w _world.png -sx 5 -sy 10 -gx 180.0 -gy 180
```
##### To run demo for the the assumption of An and Hoai's Theory:
```bash
python Robots_theory.py -n 0 -m _forest.csv -gx 500 -gy 500
```
* set robot_vision parameter in Program_config.py to see the different outcome of experiments

##### To generate a map:
```bash
python map_generator.py -n <number of points> -m <map name>
```
Example: ``` python map_generator.py -n 100 -m _map.csv ```

- Click on the given plot to input points
- Middle mouse click to terminate input

##### To display a map:
```bash
python map_display.py -m <map name>
```
Example: ``` python map_display.py -m _map.csv ```