# 表面法线

表面法线是几何体表面的重要属性，属于特征描述范畴，由于下章关键点需要此操作，所以提前给出用法。

```
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// 包含相关头文件
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT; // 也可以pcl::Normal,但无法用PCLVisualizer显示。

int main(int argc, char** argv)
{
	// 读取点云
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile(argv[1], *cloud);
	
	// 计算法向量
	pcl::NormalEstimation<PointT, PointNT> nest;
	//nest.setRadiusSearch(0.01); // 设置拟合时邻域搜索半径，最好用模型分辨率的倍数
	nest.setKSearch(50); // 设置拟合时采用的点数
	nest.setInputCloud(cloud);
	pcl::PointCloud<PointNT>::Ptr normals(new pcl::PointCloud<PointNT>);
	nest.compute(*normals);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{	// 生成时只生成了法向量，没有将原始点云信息拷贝，为了显示需要复制原信息
		// 也可用其他方法进行连接，如：pcl::concatenateFields
		normals->points[i].x = cloud->points[i].x;
		normals->points[i].y = cloud->points[i].y;
		normals->points[i].z = cloud->points[i].z;
	}

	// 显示
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud, "cloud");
	int level = 100; // 多少条法向量集合显示成一条
	float scale = 0.01; // 法向量长度
	viewer.addPointCloudNormals<PointNT>(normals, level, scale, "normals");

	viewer.spin();

	system("pause");
	return 0;
}
```