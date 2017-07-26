# 提取关键点

关键点也称为兴趣点，它是2D图像或3D点云或曲面模型上，可以通过定义检测标准来获取的具有稳定性、区别性的点集。从技术上来说，关键点的数量相比于原始点云或图像数据量上减小很多，与局部特征描述子结合一起，组成关键点描述子常用来形成原始数据的紧凑表示，而且不失代表性和描述性，从而加快后续识别、追踪等对数据的处理速度。故而，关键点提取就成为2D与3D信息处理中不可或缺的关键技术。

PCL中pcl_keypoints库目前提供几种常用的关键点检测算法，下面给出几种常用算法的实例。

* **ISSKeypoint3D**

参考文献：
Yu Zhong, “Intrinsic shape signatures: A shape descriptor for 3D object recognition,”Computer Vision Workshops (ICCV Workshops), 2009 IEEE 12th International Conference on ,vol., no., pp.689-696, Sept. 27 2009-Oct. 4 2009

```
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// 包含相关头文件
#include <pcl/keypoints/iss_3d.h>

#include "resolution.h" // 用于计算模型分辨率

typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
	// 读取点云
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile(argv[1], *cloud);
	std::cout << "original cloud size : " << cloud->size() << std::endl;

	double resolution = computeCloudResolution(cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	pcl::ISSKeypoint3D<PointT, PointT> iss_detector;
	iss_detector.setSearchMethod(tree);
	iss_detector.setSalientRadius(6 * resolution);
	iss_detector.setNonMaxRadius(4 * resolution);
	iss_detector.setThreshold21(0.975);
	iss_detector.setThreshold32(0.975);
	iss_detector.setMinNeighbors(5);
	iss_detector.setNumberOfThreads(4);
	iss_detector.setInputCloud(cloud);

	pcl::PointCloud<PointT>::Ptr keys(new pcl::PointCloud<PointT>);
	iss_detector.compute(*keys);
	std::cout << "key points size : " << keys->size() << std::endl;

	system("pause");
	return 0;
}
```

* **HarrisKeypoint3D**

HarrisKeypoint3D是对2D的Harris提取关键点算法的一个三维扩展。

```
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// 包含相关头文件
#include <pcl/keypoints/harris_3d.h>

#include "resolution.h" // 用于计算模型分辨率

typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
	// 读取点云
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile(argv[1], *cloud);
	std::cout << "original cloud size : " << cloud->size() << std::endl;

	double resolution = computeCloudResolution(cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	pcl::HarrisKeypoint3D<PointT, pcl::PointXYZI> detector;
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_temp(new pcl::PointCloud<pcl::PointXYZI>);
	detector.setNonMaxSupression(true);
	detector.setRadiusSearch(10 * resolution);
	detector.setThreshold(1E-6);
	detector.setSearchMethod(tree); // 不写也可以，默认构建kdtree
	detector.setInputCloud(cloud);
	detector.compute(*keypoints_temp);
	pcl::console::print_highlight("Detected %d points !\n", keypoints_temp->size());
	pcl::PointCloud<PointT>::Ptr keys(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*keypoints_temp, *keys);

	system("pause");
	return 0;
}
```

* **SIFTKeypoint**

SIFTKeypoint是对2D的sift算法的一个扩展。参考：David G. Lowe, "Distinctive image features from scale-invariant keypoints," International Journal of Computer Vision, 60, 2 (2004), pp. 91-110.

```
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// 包含相关头文件
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>

#include "resolution.h" // 用于计算模型分辨率

typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
	// 读取点云
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile(argv[1], *cloud);
	std::cout << "original cloud size : " << cloud->size() << std::endl;

	double resolution = computeCloudResolution(cloud); // 模型分辨率

	// 法向量
	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree_n);
	//	ne.setRadiusSearch(10 * resolution);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	// 拷贝数据
	for (size_t i = 0; i < cloud_normals->points.size(); ++i)
	{
		cloud_normals->points[i].x = cloud->points[i].x;
		cloud_normals->points[i].y = cloud->points[i].y;
		cloud_normals->points[i].z = cloud->points[i].z;
	}

	// sift参数
	const float min_scale = 0.001f;
	const int n_octaves = 5; //3
	const int n_scales_per_octave = 6; //4
	const float min_contrast = 0.001f;

	// 使用法向量作为强度计算关键点，还可以是rgb、z值或者自定义，具体参看API
	pcl::SIFTKeypoint<pcl::PointNormal, PointT> sift; //PointT 可以是 pcl::PointWithScale包含尺度信息
	pcl::PointCloud<PointT> keys;
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
	sift.setSearchMethod(tree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud(cloud_normals);
	sift.compute(keys);
	std::cout << "No of SIFT points in the result are " << keys.points.size() << std::endl;

	system("pause");
	return 0;
}
```
