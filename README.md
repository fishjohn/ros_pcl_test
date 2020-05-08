# ros_pcl_test
点云匹配静态ros测试

## 环境

Ubuntu :16.04 LTS

PCL : 1.7

ROS : kinetic

CLion

电脑配置：

![](https://s1.ax1x.com/2020/05/08/Ynuh2q.png)

## pcl的初步使用(ROS)

1. 从pcd文件中读取点云数据

   `pcl::io::loadPCDFile("/home/luohx/code/pcl_test/table_scene_lms400.pcd", *cloud_in)`

   定义一个PointCloud类型的对象，从指定路径读取cloud数据

2. VoxelGrid体素滤波器

   ```
     pcl::VoxelGrid<pcl::PointXYZ> sor;
     sor.setInputCloud(cloud_in);
     sor.setLeafSize(0.02, 0.02, 0.02);
     sor.filter(*cloud_in);
   ```

   为了减小计算量，将读取的数据通过滤波器，设置叶的大小，叶的大小越大点越稀疏，叶的大小越小点越密集．

   滤波前图像

   ![](https://s1.ax1x.com/2020/05/08/Yn1yND.png)

   滤波后图像

![](https://s1.ax1x.com/2020/05/08/Ynl6ln.png)

​	滤波前后数据比较

![](https://s1.ax1x.com/2020/05/08/Yn8kFS.png)

3. PCL对ROS的接口

   PCL对ROS的接口提供PCL数据结构的交流，通过通过ROS提供的以消息为基础的交流系统。

   sensor_msgs::PointCloud2是个重要的消息类型，这个消息通常是用来转换pcl::PointCloud类型的，我们可以定义一个sensor_msgs::PointCloud2类型的对象，承接由上面pcl::PointCloud类型转换而来的与ROS兼容的数据类型，转换的方式是使用`pcl::toROSMsg`

   ```
     sensor_msgs::PointCloud2 output_in;
     pcl::toROSMsg(*cloud_in, output_in);
     output_in.header.frame_id = "odom";
   ```

   经过转换后可以将数据通过ROS发布出去在Rviz显示

## 点云匹配测试　(icp算法)

在此我使用了icp算法做静态的点云匹配测试，相当于只有1帧的数据．

1. 首先我先将原图通过位置变换得到一个另一个坐标的图像，如下所示

![](https://s1.ax1x.com/2020/05/08/YntY4K.png)

2. 然后通过键盘输入控制，每次输入一个空格进行10次的icp算法迭代匹配两个图像

   ![](https://s1.ax1x.com/2020/05/08/YnrKg0.png)

3. 但是迭代十次平均需要0.3s的时间，实在太慢了,还需要进行进一步优化才行

![](https://s1.ax1x.com/2020/05/08/YnDz4A.png)