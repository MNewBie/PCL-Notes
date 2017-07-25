# 利用KDTree近邻搜索

k-d树（k-dimensional树的简称），是一种分割k维数据空间的数据结构。主要应用于多维空间关键数据的搜索（如：范围搜索和最近邻搜索）。K-D树是二进制空间分割树的特殊的情况。

索引结构中相似性查询有两种基本的方式：一种是范围查询（range searches），另一种是K近邻查询（K-neighbor searches）。范围查询就是给定查询点和查询距离的阈值，从数据集中找出所有与查询点距离小于阈值的数据；K近邻查询是给定查询点及正整数K，从数据集中找到距离查询点最近的K个数据，当K=1时，就是最近邻查询（nearest neighbor searches）。

PCL中类`pcl::KdTree<PointT>`是kd-tree数据结构的实现。并且提供基于FLANN进行快速搜索的一些相关子类与包装类。具体可以参考相应的API。下面给出2个类的具体用法。

* **pcl::search::KdTree &lt; PointT &gt;**

`pcl::search::KdTree<PointT>`是`pcl::search::Search< PointT >`的子类，是`pcl::KdTree<PointT>`的包装类。包含(1) k 近邻搜索；(2) 邻域半径搜索。

```
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/search/kdtree.h> // 包含kdtree头文件

typedef pcl::PointXYZ PointT;

int main()
{
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("read.pcd", *cloud);

	// 定义KDTree对象
	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	kdtree->setInputCloud(cloud); // 设置要搜索的点云，建立KDTree

	std::vector<int> indices; // 存储查询近邻点索引
	std::vector<float> distances; // 存储近邻点对应距离的平方

	PointT point = cloud->points[0]; // 初始化一个查询点
	
	// 查询距point最近的k个点
	int k = 10;
	int size = kdtree->nearestKSearch(point, k, indices, distances);

	std::cout << "search point : " << size << std::endl;

	// 查询point半径为radius邻域球内的点
	double radius = 2.0;
	size = kdtree->radiusSearch(point, radius, indices, distances);

	std::cout << "search point : " << size << std::endl;

	system("pause");
	return 0;
}
```

**注意：** 搜索结果默认是按照距离point点的距离从近到远排序；如果InputCloud中含有point点，搜索结果的的第一个点是point本身。

* **pcl::KdTreeFLANN &lt; PointT &gt;**

`pcl::KdTreeFLANN<PointT>`是`pcl::KdTree<PointT>`的子类，可以实现同样的功能。

```
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// 包含相关头文件
#include <pcl/kdtree/kdtree_flann.h>

typedef pcl::PointXYZ PointT;

int main()
{
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("read.pcd", *cloud);

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree; //创建KDtree
	kdtree.setInputCloud(cloud); // 设置要搜索的点云，建立KDTree

	std::vector<int> indices; // 存储查询近邻点索引
	std::vector<float> distances; // 存储近邻点对应距离的平方

	PointT point = cloud->points[0]; // 初始化一个查询点
	
	// 查询距point最近的k个点
	int k = 10;
	int size = kdtree.nearestKSearch(point, k, indices, distances);

	std::cout << "search point : " << size << std::endl;

	// 查询point半径为radius邻域球内的点
	double radius = 2.0;
	size = kdtree.radiusSearch(point, radius, indices, distances);

	std::cout << "search point : " << size << std::endl;

	system("pause");
	return 0;
}
```
