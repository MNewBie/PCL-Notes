# 数据输入输出

* **从PCD文件中读取与保存点云文件**

```
#include <iostream> //标准c++库输入输出相关头文件
#include <pcl/io/pcd_io.h> // pcd读写相关头文件
#include <pcl/point_types.h> // pcl中支持的点类型头文件

// 定义点云格式，具体见下章
typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
	// 定义点云
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

	// 读取点云，失败返回-1
	if (pcl::io::loadPCDFile<PointT>("readName.pcd", *cloud) == -1)
	{
		PCL_ERROR("couldn't read file\n");
		return (-1);
	}

	// 输出点云大小 cloud->width * cloud->height
	std::cout << "点云大小：" << cloud->size() << std::endl; 
	
	// 保存点云文件
	pcl::io::savePCDFile("saveName.pcd", *cloud);

	system("pause"); // windows命令行窗口暂停
	return (0);
}

```

* **读取与保存PLY文件**

后缀命名为.ply格式文件，常用的点云数据文件。ply文件不仅可以存储点数据，而且可以存储网格数据. 用编辑器打开一个ply文件，观察表头，如果表头element face的值为0,则表示该文件为点云文件，如果element face的值为某一正整数N，则表示该文件为网格文件，且包含N个网格.所以利用pcl读取 ply 文件，不能一味用`pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PintT>)`来读取。在读取ply文件时候，首先要分清该文件是点云还是网格类文件。如果是点云文件，则按照一般的点云类去读取即可，如果ply文件是网格类，则需要:

```
#include <pcl/io/ply_io.h>

pcl::PolygonMesh mesh;
pcl::io::loadPLYFile("readName.ply", mesh);
pcl::io::savePLYFile("saveName.ply", mesh);

```
