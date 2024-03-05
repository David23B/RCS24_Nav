# RCS24_Nav

## 仿真测试

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

### BUG

- [x] 接受不到地图中选中的目标点
- [x] 起始点设置为车的位置
- [ ] 规划出的路径很扭曲
- [ ] 封装
- [ ] 考虑是否要在全局规划中更新地图
- [ ] 较远的位置规划不出路径
