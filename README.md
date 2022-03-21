# sagittarius_sdk

非ROS下的SDK

## 依赖库安装

**Eigen 库安装**

```bash
sudo apt-get install libeigen3-dev
```

如果在编译时找不到`Eigen/Dense`而报错，原因是Eigen3相比前一个版本多套了一层文件夹.

解决方式是软连接到上一层目录。先找到Eigen3的安装目录

```bash
whereis eigen3
```

假设Eigen3的安装目录是`/usr/include/eigen3`

```bash
cd /usr/include/
sudo ln -s eigen3/Eigen Eigen
```

安装机械臂动态库：

如果你的硬件平台是x86的：

```bash
cd sagittarius_sdk
sudo cp ./lib/x86_64/libsagittarius_sdk.so /usr/lib/
```

如果你的硬件平台arm64的：

```bash
cd sagittarius_sdk
sudo cp ./lib/arm64/libsagittarius_sdk.so /usr/lib/
```

## 

## 例程编译

此程序为非ROS下的例程编译
使用动态库的编译:

```bash
g++ -I ./ -o example sagittarius_example.cpp -L ./ -lsagittarius_sdk -lpthread -lboost_system -lboost_thread
```

或者直接执行(请先根据硬件平台修改Makefile里的PLATFORM平台类型):

```bash
make
```

