# Road-boundary-extraction
输入：分割之后的路面点云/或者分割出的路面图像＋点云
输出：道路边界点
提供的提取方法：
１．分线束提取
２．极坐标扫描提取

使用方法：
环境需要：pcl >1.7 opencv>=3.2
config 文件夹下为参数配置文件，根据需要配置
1.cd到main文件一级
２.mkdir build
３．cd build
4.cmake ..
5.make
6. ./map_label true
