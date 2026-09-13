# 安装与部署

在新 Ubuntu 机器或新 TRON1 上部署本系统的标准流程。

## 1. 安装 ROS Noetic

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/ros/distros/noetic/ros.key | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full python3-catkin-tools
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo rosdep init
rosdep update
```

## 2. 获取工作空间

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 方式 A：复制 phone_gps_bridge 整个目录（含 catkin_ws/src）
# 方式 B：分别 clone phone_gps_bridge 与 tron_open_space_nav
```

目录结构：

```
~/catkin_ws/src/
├── phone_gps_bridge/
└── tron_open_space_nav/
```

## 3. 安装依赖

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# 可选：UTM 坐标支持
pip3 install pyproj

# 可选：RViz 多点插件（见 docs/RUN_GUIDE.md）
```

## 4. 编译

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make_isolated
source ~/catkin_ws/devel_isolated/setup.bash
```

## 5. 配置机器人

```bash
roscd tron_open_space_nav/config
cp robot.example.yaml robot.yaml
nano robot.yaml
```

编辑：

```yaml
ws_url: "ws://<ROBOT_WLAN_IP>:5000"   # TRON1 WebSocket 地址
accid: "<YOUR_ROBOT_ACCID>"           # 如 WF_TRON1A_412
```

## 6. 网络配置

### 机器人（roscore 运行在此）

```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=<ROBOT_IP>
# 写入 ~/.bashrc
```

### 开发电脑（RViz）

```bash
export ROS_MASTER_URI=http://<ROBOT_IP>:11311
export ROS_IP=<DEV_PC_IP>
```

## 7. 验证安装

```bash
rospack find tron_open_space_nav
rospack find phone_gps_bridge
rosrun tron_open_space_nav check_system.sh   # 需导航栈已启动
```

## Docker 开发环境（可选）

```bash
cd phone_gps_bridge
./scripts/start_container.sh
./scripts/enter_container.sh
cd /root/catkin_ws && catkin_make_isolated
```

容器内路径为 `/root/catkin_ws`，挂载自宿主机 `phone_gps_bridge/catkin_ws`。

## 部署到新 TRON1 检查清单

- [ ] `robot.yaml` 中 WebSocket 地址与 accid
- [ ] 手机 UDP 目标 IP = 机器人 IP，端口 10110
- [ ] FAST-LIO + Livox 官方栈已运行
- [ ] `gps_global_nav.launch` 使用 `use_rviz:=false`（默认）
- [ ] 开发电脑 RViz 单独启动
