# 使用方法
## 1. 需要的库 
* vtk(是安装PCL的依赖库，编译时需要勾选Qt)
* PCL

## 2. 前置条件
可能是我一些发布订阅的设置写的不太对，使用ROS2默认的FastDDS会导致程序运行很卡，目前还没找到原因，所以请按照下述方法将DDS修改为cyclonedds

### 2.1 安装cyclonedds
```
sudo apt install ros-lyrical-rmw-cyclonedds-cpp
```

### 2.2 修改默认的DDS
```
grep -q "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" ~/.bashrc || echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```
`grep -q ... ||` 保证重复执行不会往 `.bashrc` 里写入多行。

### 2.3 检查是否修改成功
```
ros2 doctor --report | grep -i "middleware name"
```
输出显示rmw_cyclonedds_cpp则说明修改成功

## 3. 编译

### 3.1 编译工作空间
```
cd ~/ego-planner-swarm
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --continue-on-error \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```
* `-DPython3_EXECUTABLE=/usr/bin/python3` **必须带上**。PATH 中 `~/.local/bin` 下若有 uv 等工具安装的 Python，CMake 会优先选中它，而它没有 `catkin_pkg`，会导致包在配置阶段报 `ModuleNotFoundError: No module named 'catkin_pkg'`
* `--continue-on-error`：某个包编译失败后继续编译其余包，便于一次性看到全部错误
* `--symlink-install`：Python 脚本与 launch 文件以软链接方式安装，改动后无需重新编译

### 3.2 加载工作空间环境
每个新终端都要执行一次：
```
source ~/ego-planner-swarm/install/setup.bash
```
未加载时运行 launch 会报 `Package 'ego_planner' not found`。可把下面这行加进 `~/.bashrc`，省掉每次手动 source：
```
[ -f "$HOME/ego-planner-swarm/install/setup.bash" ] && source "$HOME/ego-planner-swarm/install/setup.bash"
```
前面加文件存在判断，是为了删掉 `install/` 重新编译时，新终端不会每次都报错。

## 4. 代码运行
### 4.1 运行Rviz
```
ros2 launch ego_planner rviz.launch.py 
```
### 4.2 运行规划程序
新开一个终端，输入以下指令
* 单机
```
ros2 launch ego_planner single_run_in_sim.launch.py 
```
* swarm
```
ros2 launch ego_planner swarm.launch.py 
```
* large swarm
```
ros2 launch ego_planner swarm_large.launch.py  
```
* 附加参数，可以选择地图生成模式以及是否考虑动力学
    * use_mockamap:地图生成方式，默认为False，False时使用Random Forest, True时使用mockamap
    * use_dynamic:是否考虑动力学，默认为False, False时不考虑, True时考虑
```
ros2 launch ego_planner single_run_in_sim.launch.py use_mockamap:=True use_dynamic:=False
```
