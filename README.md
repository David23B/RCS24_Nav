# RCS24_Nav

## RUN

仿真建立在CMU开源的导航系统上。在此基础上，我们引入上层全局规划器进行静态规划

```bash
git clone https://github.com/David23B/RCS24_Nav.git
./src/autonomous_exploration_development_environment/src/vehicle_simulator/mesh/download_environments.sh
```

```bash
catkin_make
source devel/setup.bash
```

### 建图

在仿真环境中自主探索并构建二维栅格地图

```bash
roslaunch vehicle_simulator build_map.launch
```

当图建的差不多了，使用以下命令保存地图，**注意，需要手动修改yaml文件中的origin**

```bash
rosrun map_server map_saver map:=/projected_map -f /home/jason/Project/github/RCS24_Nav/src/global_planner/map/map
```

### 导航

```
roslaunch global_planner global_planner.launch
```

## BUG

- [x] 接受不到地图中选中的目标点
- [x] 起始点设置为车的位置
- [ ] 规划出的路径很扭曲（如果global做的是方向引导，似乎不需要平滑）
- [ ] 封装
- [ ] 考虑是否要在全局规划中更新地图（先验+实时地图规划）（先验不足）
- [x] **较远的位置规划不出路径**
- [x] 当前waypoint是根据时间间隔发送，修改成根据waypoint与vehicle的距离进行发送（3.6）
- [x] **无法到达指定点（应该是少发了最后的点）**
- [ ] 膨胀栅格可视化（costmap）
- [ ] https://github.com/ms-iot/ROSOnWindows/issues/279
- [ ] RVIZ无法订阅自定义的消息类型（l_path）


## THINKING

- 可以修改A*的邻居间隔，邻居不是紧紧挨着的，可以是隔几个格子，这样子可以减少搜索时间，并且可以减少转折点
- global planner引入的目的是给一个方向的引导（local planner探测范围有限），如果是这样的话，似乎没有必要对规划出的路径进行过多的平滑处理，毕竟只是做引导作用
