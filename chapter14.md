
# 曲面重建 

曲面重建可以用于逆向工程、数据可视化、自动化建模等领域。PCL中目前实现了多种基于点云的曲面重建算法，如：泊松曲面重建、贪婪投影三角化、移动立方体、GridProjection、EarClipping等算法。可以参考<http://pointclouds.org/documentation/tutorials/>相关内容、CSDN博客<http://blog.csdn.net/xuezhisdc/article/details/51034359/>。

还有一个特殊的B样条拟合( B-splines )，需要在pcl编译时特殊支持。OrganizedFastMesh需要输入有序的点云。
而移动最小二乘(mls)虽然放在surface模块下，此类并不能输出拟合后的表面，不能生成Mesh或者Triangulations，只是将点云进行了MLS的映射，使得输出的点云更加平滑。

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

* **泊松曲面重建**

泊松曲面重建基于泊松方程。根据泊松方程使用矩阵迭代求出近似解，采用移动立方体算法提取等值面，对所测数据点集重构出被测物体的模型，泊松方程在边界处的误差为零，因此得到的模型不存在假的表面框。(http://blog.csdn.net/jennychenhit/article/details/52126156?locationNum=8)

```
#include <pcl/surface/poisson.h>

pcl::Poisson<pcl::PointNormal> pn;
pn.setConfidence(false);
pn.setDegree(2);
pn.setDepth(8);
pn.setIsoDivide(8);
pn.setManifold(false);
pn.setOutputPolygons(false);
pn.setSamplesPerNode(3.0);
pn.setScale(1.25);
pn.setSolverDivide(8);
pn.setSearchMethod(tree2);
pn.setInputCloud(cloud_with_normals);
pcl::PolygonMesh mesh;
pn.performReconstruction(mesh);
(其他同上)
```

* **移动立方体**

MarchingCubes(移动立方体)算法是目前三围数据场等值面生成中最常用的方法。它实际上是一个分而治之的方法，把等值面的抽取分布于每个体素中进行。对于每个被处理的体素，以三角面片逼近其内部的等值面片。每个体素是一个小立方体，构造三角面片的处理过程对每个体素都“扫描”一遍，就好像一个处理器在这些体素上移动一样，由此得名移动立方体算法。

```
#include <pcl/surface/marching_cubes_hoppe.h>

pcl::MarchingCubes<pcl::PointNormal>::Ptr mc(new pcl::MarchingCubesHoppe<pcl::PointNormal>);
//设置MarchingCubes对象的参数
mc->setIsoLevel(0.0f);
mc->setGridResolution(5, 5, 5);
mc->setPercentageExtendGrid(0.0f);
mc->setSearchMethod(tree2);
mc->setInputCloud(cloud_with_normals);

pcl::PolygonMesh mesh;//执行重构，结果保存在mesh中
mc->reconstruct(mesh);
(其他同上)
```

* **B样条拟合**

参考：http://pointclouds.org/documentation/tutorials/bspline_fitting.php#bspline-fitting

* **移动最小二乘**

虽说此类放在了Surface下面，但是通过反复的研究与使用，发现此类并不能输出拟合后的表面，不能生成Mesh或者Triangulations，只是将点云进行了MLS的映射，使得输出的点云更加平滑，进行上采样和计算法向量。

```
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "resolution.h"

int main(int argc, char** argv)
{
	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Load bun0.pcd -- should be available with the PCL archive in test 
	pcl::io::loadPCDFile(argv[1], *cloud);

	double resolution = computeCloudResolution(cloud);

	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal> mls_points;

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setInputCloud(cloud);
	mls.setComputeNormals(true); //我们都知道表面重构时需要估计点云的法向量，这里MLS提供了一种方法来估计点云法向量。（如果是true的话注意输出数据格式）。
	mls.setPolynomialFit(true); //对于法线的估计是有多项式还是仅仅依靠切线。
	mls.void setPolynomialOrder(3); //MLS拟合曲线的阶数，这个阶数在构造函数里默认是2，但是参考文献给出最好选择3或者4
	mls.setSearchMethod(tree);
	mls.setSearchRadius(10 * resolution);
	// Reconstruct
	mls.process(mls_points);
	
	// Save output
	//pcl::io::savePCDFile("bun0-mls.pcd", mls_points);
	// show
	pcl::visualization::PCLVisualizer viewer_1;
	viewer_1.addPointCloud(cloud, "cloud");
	viewer_1.spin();

	pcl::visualization::PCLVisualizer viewer_2;
	pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(mls_points, *result);
	viewer_2.addPointCloud(result, "mls_points");
	viewer_2.spin();

	return 0;
}
```







