# autonomousRobot
This project is to simulate an autonomousRobot :car: that try to find a way to reach a goal (target) 

##### To run demo:
```
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
```
python Robots.py -n 5 -m _MuchMoreFun.csv -sx 5 -sy 5 -gx 30.0 -gy 70.0
python Robots.py -n 5 -w _MuchMoreFun.csv -sx 5 -sy 5 -gx 30.0 -gy 70.0
```

##### To generate a map:
```
python map_generator.py -n <number of points> -m <map name>
```
Example: ``` python map_generator.py -n 100 -m _map.csv ```

- Click on the given plot to input points
- Middle mouse click to terminate input

##### To display a map:
```
python map_display.py -m <map name>
```
Example: ``` python map_display.py -m _map.csv ```