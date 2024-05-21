# PanoSim-Autoware Autoware 配置
panosim-autoware 中 autoware 的部分文件进行修改，可根据具体情况进行下载。
- [PanoSim-Autoware Autoware 配置](#panosim-autoware-autoware-配置)
  - [1. 软件安装](#1-软件安装)
  - [2. 联合仿真资源加载](#2-联合仿真资源加载)

## 1. 软件安装
1.1 官方安装
[Autoware 官方文档](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/)
1.2 下载 PanoSim-Autoware/Autoware 文件
- 1.2.1 克隆 `panosim-autoware`
  ```
  git clone https://gitee.com/wobuzhuchele/panosim-autoware.git -b -v1.0
  cd  panosim-autoware/Autoware   
  ```
- 1.2.2 如果您是第一次安装Autoware，您可以使用提供的Ansible脚本自动安装依赖项。
  ```
  ./setup-dev-env.sh
  ```
- 1.2.3 安装依赖的ROS包
  -  除了核心组件外，Autoware还需要一些ROS 2软件包。工具rosdep允许自动搜索和安装这些依赖项。您可能需要在安装rosdep之前运行rosdep update。
  ```
  source /opt/ros/humble/setup.bash
  rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
  ```


- 构建工作空间
  -  Autoware 使用[ colcon ](https://github.com/colcon)来构建工作区。有关更高级的选项，请参阅[文档](https://colcon.readthedocs.io/)。
  ```
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```
## 2. 联合仿真资源加载
2.1 地图下载
- [PanoTown]()
- [PanoTown01]()
- [chuangxinyuan]()
  
2.2 将下载的地图文件解压到 `~/autoware_map/` 下
![1715708196822](image/PanoSim-Autoware%20Autoware/1715708196822.png)
PanoSimTown文件结构(例)
![1715708226880](image/PanoSim-Autoware%20Autoware/1715708226880.png)

>[跳转到 PanoSim-AutoWare](./PanoSim-Autoware.md)