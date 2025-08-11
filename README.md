# mindvision SDK ROS2 开发记录

## 开发/测试环境
ubuntu 22.04
ros-humble
cmake 3.22.1
gcc 11.4.0

## 安装方法

### ROS2

小鱼一键安装

```bash
wget http://fishros.com/install -O fishros && . fishros
```

或者(不保证成功)

```bash
# ros-humble-desktop
echo "deb [arch=$(dpkg --print-architecture)] https://repo.huaweicloud.com/ros2/ubuntu/ $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt install curl gnupg2 -y
curl -s https://gitee.com/ohhuo/rosdistro/raw/master/ros.asc | sudo apt-key add -
sudo apt update    
sudo apt install ros-humble-desktop
sudo apt install python3-argcomplete -y

# rosdep
sudo apt install python3-rosdep -y
sudo rosdep init || true
rosdep update
rosdep install --from-paths . --ignore-src -r -y
```

### mindvision-sdk

```bash
mkdir mindvision-sdk
wget https://www.mindvision.com.cn/wp-content/uploads/2023/08/linuxSDK_V2.1.0.37.tar.gz -O mindvision-sdk/SDK.tar.gz
tar -zxvf mindvision-sdk/SDK.tar.gz --directory=mindvision-sdk
sed -i 's/usr\/include/usr\/include\/mindvision/g' mindvision-sdk/install.sh
sed -i 17a\\"mkdir -p /usr/include/mindvision" mindvision-sdk/install.sh
cd mindvision-sdk && sudo bash ./install.sh && cd ..
rm -r mindvision-sdk
```

## 注意事项

在快于最大 FPS 的时间间隔下读取缓冲区会读取失败，这里包装了一层，会使用上一帧代替并警告

如果出现奇怪的依赖问题，可以尝试运行 `setup_env.sh` 
该脚本会重设 python 环境，可以解决 conda 造成的 python 解释器问题

该脚本适配了 `zsh` 和 `bash`

## 开发中