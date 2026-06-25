# ai_ros2_task

REF: [link](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#run-navigation-nodes)

```bash
ros2 launch my_gazebo labirinto.launch.py
```

# RUN THIS NODE
    
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True \
map:='topic://map'
ros2 run robot_ai navigator
```

## SLAM

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

### Save Map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

## Navigation

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml use_sim_time:=True
```

without map

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True \
map:='topic://map'

```