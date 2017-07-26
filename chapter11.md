# 点云特征描述

3D点云特征描述与提取是点云信息处理中的最基础也是最关键的一部分，点云的识别、分割、重采样、配准、曲面重建等处理大部分算法，都严重依赖特征描述与提取的结果。从尺度上来分，一般分为局部特征描述和全局特征描述。<http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_4:_3D_object_recognition_(descriptors)>有各种描述子较为详细的介绍。
下面给出常用的PFH、FPFH、SHOT、RoPS使用实例。

* **PFH**

参考文献：
(1) R.B. Rusu, N. Blodow, Z.C. Marton, M. Beetz. Aligning Point Cloud Views using Persistent Feature Histograms. In Proceedings of the 21st IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Nice, France, September 22-26 2008.
(2) R.B. Rusu, Z.C. Marton, N. Blodow, M. Beetz. Learning Informative Point Classes for the Acquisition of Object Model Maps. In Proceedings of the 10th International Conference on Control, Automation, Robotics and Vision (ICARCV), Hanoi, Vietnam, December 17-20 2008.

```
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// 包含相关头文件
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>

#include "resolution.h" // 用于计算模型分辨率

typedef pcl::PointXYZ PointT;
typedef pcl::Normal PointNT; 
typedef pcl::PFHSignature125 FeatureT;

int main(int argc, char** argv)
{
	// 读取点云
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile(argv[1], *cloud);

	// 读取关键点，也可以用之前提到的方法计算
	pcl::PointCloud<PointT>::Ptr keys(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile(argv[2], *keys);

	double resolution = computeCloudResolution(cloud);

	// 法向量
	pcl::NormalEstimation<PointT, PointNT> nest;
	//	nest.setRadiusSearch(10 * resolution);
	nest.setKSearch(10);
	nest.setInputCloud(cloud);
	nest.setSearchSurface(cloud);
	pcl::PointCloud<PointNT>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	nest.compute(*normals);
	std::cout << "compute normal\n";

	// 关键点计算PFH描述子
	pcl::PointCloud<FeatureT>::Ptr features(new pcl::PointCloud<FeatureT>);
	pcl::PFHEstimation<PointT, PointNT, FeatureT> fest;
	fest.setRadiusSearch(18 * resolution);
	fest.setSearchSurface(cloud);
	fest.setInputCloud(keys);
	fest.setInputNormals(normals);
	fest.compute(*features);
	std::cout << "compute feature\n";

	system("pause");
	return 0;
}
```