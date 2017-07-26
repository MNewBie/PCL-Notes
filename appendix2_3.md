# 常用函数(3)

这里给出两种计算局部坐标系(LRF)的方式。

* **BOARDLocalReferenceFrame**

参考文献：
A. Petrelli, L. Di Stefano, "On the repeatability of the local reference frame for partial shape matching", 13th International Conference on Computer Vision (ICCV), 2011

```
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/board.h>
#include <pcl/search/kdtree.h>

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("rabbit.pcd", *cloud);
	std::cout << "load " << cloud->points.size() << std::endl;

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	ne.setKSearch(10);
	ne.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
	ne.compute(*normals);
	std::cout << normals->points.size();

	pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame> rftest;
	rftest.setInputCloud(cloud);
	rftest.setInputNormals(normals);
	rftest.setSearchSurface(cloud);
	rftest.setRadiusSearch(0.01);
	rftest.setFindHoles(false);

	pcl::PointCloud<pcl::ReferenceFrame>::Ptr rf(new pcl::PointCloud<pcl::ReferenceFrame>);
	rftest.compute(*rf);

	system("pause");
	return 0;
}
```

* **SHOTLocalReferenceFrame**

参考文献：shot描述子

```
#include <pcl/io/pcd_io.h>
#include <pcl/features/shot_lrf.h>

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("rabbit.pcd", *cloud);
	std::cout << "load " << cloud->points.size() << std::endl;

	pcl::SHOTLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::ReferenceFrame>::Ptr lrf_estimator(new pcl::SHOTLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::ReferenceFrame>);
	lrf_estimator->setRadiusSearch(0.001);
	lrf_estimator->setInputCloud(cloud);
	lrf_estimator->setSearchSurface(cloud);
	pcl::PointCloud<pcl::ReferenceFrame>::Ptr cloud_lrf(new pcl::PointCloud<pcl::ReferenceFrame>);
	lrf_estimator->compute(*cloud_lrf);
	std::cout << "compute lrf size: " << cloud_lrf->size() << std::endl;

	system("pause");
	return 0;
}
```
