# autonomousRobot
This project is to simulate an autonomousRobot :car: that try to find a way to reach a goal (target) 

###### To run demo type:
```
python Robots.py -n <number of run times> -m <map name> -s <start point> -g <goal point>
```
1. number of run times:
    - < 0: run until meet the given goal
    - n: number of run times
    - default: 1
- goal point
    - x_y: input x y of goal
    - r: random point for goal
    
Example: ``` python Robots.py -n 1 -m _MuchMoreFun.csv -s <start point x_y> -g <goal point x_y>```

###### To generate a map:
```
python map_generator.py -n <number of points> -m <map name>
```
Example: ``` python map_generator.py -n 100 -m _map.csv ```

- Click on the given plot to input points
- Middle mouse click to terminate input

###### To display map:
```
python map_display.py -m <map name>
```
Example: ``` python map_display.py -m _map.csv ```