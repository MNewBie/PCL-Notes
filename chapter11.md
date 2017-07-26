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

* **FPFH**

参考文献：
(1) R.B. Rusu, N. Blodow, M. Beetz. Fast Point Feature Histograms (FPFH) for 3D Registration. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), Kobe, Japan, May 12-17 2009.
(2) R.B. Rusu, A. Holzbach, N. Blodow, M. Beetz. Fast Geometric Point Labeling using Conditional Random Fields. In Proceedings of the 22nd IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), St. Louis, MO, USA, October 11-15 2009.

相关类用法与PFH类似，只需要引入 `#include <pcl/features/fpfh.h>`;
将 `typedef pcl::PFHSignature125 FeatureT` 替换为 `typedef pcl::FPFHSignature33 FeatureT`;
将`pcl::PFHEstimation<PointT, PointNT, FeatureT> fest;` 替换为 `pcl::FPFHEstimation<PointT, PointNT, FeatureT> fest;` 即可。

* **SHOT**

参考文献：
(1) F. Tombari, S. Salti, L. Di Stefano Unique Signatures of Histograms for Local Surface Description. In Proceedings of the 11th European Conference on Computer Vision (ECCV), Heraklion, Greece, September 5-11 2010.
(2) F. Tombari, S. Salti, L. Di Stefano A Combined Texture-Shape Descriptor For Enhanced 3D Feature Matching. In Proceedings of the 18th International Conference on Image Processing (ICIP), Brussels, Belgium, September 11-14 2011.

```
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// 包含相关头文件
#include <pcl/features/shot.h>
#include <pcl/features/normal_3d.h>

#include "resolution.h" // 用于计算模型分辨率

typedef pcl::PointXYZ PointT;
typedef pcl::Normal PointNT;
typedef pcl::SHOT352 FeatureT;

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
	//	nest.setRadiusSearch(5*resolution);
	nest.setKSearch(10);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
	nest.setInputCloud(cloud);
	nest.setSearchSurface(cloud);
	nest.compute(*cloud_normal);
	std::cout << "compute normal\n";

	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
	shot.setRadiusSearch(18 * resolution);
	shot.setInputCloud(keys);
	shot.setSearchSurface(cloud);
	shot.setInputNormals(cloud_normal);
	//shot.setInputReferenceFrames(lrf);  //也可以自己传入局部坐标系
	pcl::PointCloud<FeatureT>::Ptr features(new pcl::PointCloud<FeatureT>);
	shot.compute(*features);
	std::cout << "compute feature\n";

	system("pause");
	return 0;
}
```

* **RoPS**

参考文献：
"Rotational Projection Statistics for 3D Local Surface Description and Object Recognition" by Yulan Guo, Ferdous Sohel, Mohammed Bennamoun, Min Lu and Jianwei Wan.

由于RoPS是基于网格数据，所以如果输入的是点云数据需要先进行网格化处理。

```
#include <pcl/io/pcd_io.h>
#include <pcl/features/rops_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

int main(int argc, char** argv)
{
	// 加载点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(argv[1], *cloud);
	
	// 加载关键点
	pcl::PointCloud<pcl::PointXYZ>::Ptr key_points(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(argv[2], *key_points);

	// 计算法向量
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);

	// 连接数据
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// ---- rops基于网格，所以要先将pcd点云数据重建网格 ---

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;// Initialize objects
	pcl::PolygonMesh triangles;
	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);
	gp3.setMu(2.5); // Set typical values for the parameters
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);	// Get result
	
	// ----- rops 描述-------
	// 由于pcl_1.8.0中rops还没有定义好的结构，所以采用pcl::Histogram存储描述子
	pcl::ROPSEstimation<pcl::PointXYZ, pcl::Histogram<135>> rops;
	rops.setInputCloud(key_points);
	rops.setSearchSurface(cloud);
	rops.setNumberOfPartitionBins(5);
	rops.setNumberOfRotations(3);
	rops.setRadiusSearch(0.01);
	rops.setSupportRadius(0.01);
	rops.setTriangles(triangles.polygons);
	rops.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree < pcl::PointXYZ>));
	//feature size = number_of_rotations * number_of_axis_to_rotate_around * number_of_projections * number_of_central_moments
	//unsigned int feature_size = number_of_rotations_ * 3 * 3 * 5; //计算出135
	pcl::PointCloud<pcl::Histogram<135>> description;
	rops.compute(description);  // 结果计算的是描述子。。需传入inputcloud和surface
	std::cout << "size is " << description.points.size()<<std::endl;
	//pcl::io::savePCDFile("rops_des.pcd", description); // 此句出错！！pcl::Histogram没有对应的保存方法
	
	system("pause");
	return 0;
}
```
