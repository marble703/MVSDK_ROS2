# mindvision SDK ROS2 开发记录

## SDK 安装方法

mindvision-sdk

```bash
mkdir mindvision-sdk
wget https://www.mindvision.com.cn/wp-content/uploads/2023/08/linuxSDK_V2.1.0.37.tar.gz -O mindvision-sdk/SDK.tar.gz
tar -zxvf mindvision-sdk/SDK.tar.gz --directory=mindvision-sdk
sed -i 's/usr\/include/usr\/include\/mindvision/g' mindvision-sdk/install.sh
sed -i 17a\\"mkdir -p /usr/include/mindvision" mindvision-sdk/install.sh
cd mindvision-sdk && sudo bash ./install.sh && cd ..
rm -r mindvision-sdk
```

## 开发中