# 滤波

PCL中总结了集中需要进行点云滤波处理的情况，分别如下：
(1) 点云数据密度不规则需要平滑。
(2) 因为遮挡等问题造成离群点需要去除。
(3) 大量数据需要进行下采样。
(4) 噪声数据需要去除。

PCL点云格式分为有序点云和无序点云，针对有序点云提供了双边滤波、高斯滤波、中值滤波等，针对无序点云提供了体素栅格、随机采样等。

下边给出两个下采样类的应用实例。（VoxelGrid、UniformSampling）

* **VoxelGrid、UniformSampling**

```
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// 包含相关头文件
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>

typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
	// 读取点云
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile(argv[1], *cloud);
	std::cout << "original cloud size : " << cloud->size() << std::endl;

	// 使用体素化网格(VoxelGrid)进行下采样
	pcl::VoxelGrid<PointT> grid; //创建滤波对象
	const float leaf = 0.005f; 
	grid.setLeafSize(leaf, leaf, leaf); // 设置体素体积
	grid.setInputCloud(cloud); // 设置点云
	pcl::PointCloud<PointT>::Ptr voxelResult(new pcl::PointCloud<PointT>);
	grid.filter(*voxelResult); // 执行滤波，输出结果
	std::cout << "voxel downsample size :" << voxelResult->size() << std::endl;

	// 使用UniformSampling进行下采样
	pcl::UniformSampling<PointT> uniform_sampling;
	uniform_sampling.setInputCloud(cloud);
	double radius = 0.005f;
	uniform_sampling.setRadiusSearch(radius);
	pcl::PointCloud<PointT>::Ptr uniformResult(new pcl::PointCloud<PointT>);
	uniform_sampling.filter(*uniformResult);
	std::cout << "UniformSampling size :" << uniformResult->size() << std::endl;
	
	system("pause");
	return 0;
}
```

双边滤波PCL提供了两种方法，FastBilateralFilter 和 BilateralFilter。 FastBilateralFilter需要有序点云数据，BilateralFilter需要带强度的点云数据。

* **双边滤波**

```
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/impl/bilateral.hpp>
#include <pcl/visualization/pcl_visualizer.h>

void bilateralFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr &input, pcl::PointCloud<pcl::PointXYZI>::Ptr& output)
{
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZI>);
	// Apply the filter  
	pcl::BilateralFilter<pcl::PointXYZI> fbf;
	fbf.setInputCloud(input);
	fbf.setSearchMethod(tree1);
	fbf.setStdDev(0.1);
	fbf.setHalfSize(0.1);
	fbf.filter(*output);
}
int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>); // 需要PointXYZI 
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read<pcl::PointXYZI>(argv[1], *cloud);
	
	bilateralFilter(cloud, cloud_filtered);

	/*pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud_filtered);
	viewer.spin();*/

	return (0);
}
```

* **剔除离群点**

参考：http://www.pointclouds.org/documentation/tutorials/statistical_outlier.php
