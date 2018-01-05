# 曲面重建 (分割与)

曲面重建可以用于逆向工程、数据可视化、自动化建模等领域。CL中目前实现了多种基于点云的曲面重建算法，如：泊松曲面重建、贪婪投影三角化、移动立方体、EarClipping等算法。可以参考<http://pointclouds.org/documentation/tutorials/>相关内容、CSDN博客<http://blog.csdn.net/xuezhisdc/article/details/51034359/>。

还有一个特殊的B样条拟合( B-splines )，需要在pcl编译时特殊支持。

* **贪婪投影三角化算法**

贪婪投影三角化算法对有向点云进行三角化，具体方法是先将有向点云投影到某一局部二维坐标平面内，再在坐标平面内进行平面内的三角化，再根据平面内三位点的拓扑连接关系获得一个三角网格曲面模型。

贪婪投影三角化算法原理是处理一系列可以使网格“生长扩大”的点（边缘点），延伸这些点直到所有符合几何正确性和拓扑正确性的点都被连上。该算法的优点是可以处理来自一个或者多个扫描仪扫描得到并且有多个连接处的散乱点云。但该算法也有一定的局限性，它更适用于采样点云来自于表面连续光滑的曲面并且点云密度变化比较均匀的情况。


```
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "resolution.h"

int main(int argc, char** argv)
{
	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile(argv[1], cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);
	//* the data should be available in cloud

	double resolution = computeCloudResolution(cloud);

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	// normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	// cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(5 * resolution);  //设置连接点之间的最大距离（最大边长）用于确定k近邻的球半径【默认值 0】
	// Set typical values for the parameters
	gp3.setMu(2.5); //设置最近邻距离的乘子，以得到每个点的最终搜索半径【默认值 0】
	gp3.setMaximumNearestNeighbors(100); //设置搜索的最近邻点的最大数量
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees（pi）最大平面角
	gp3.setMinimumAngle(M_PI / 18);  // 10 degrees 每个三角的最小角度
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees 每个三角的最大角度
	gp3.setNormalConsistency(false); //如果法向量一致，设置为true

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	// show
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPolygonMesh(triangles, "mesh");
	viewer.spin();

	// Finish
	return (0);
}
```