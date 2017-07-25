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
	if (pcl::io::loadPCDFile<PointT>("test.pcd", *cloud) == -1)
	{
		PCL_ERROR("couldn't read file\n");
		return (-1);
	}

	// 输出点云大小 cloud->width * cloud->height
	std::cout << "点云大小：" << cloud->size() << std::endl; 
	
	// 保存点云文件
	pcl::io::savePCDFile("saveName.pcd", *cloud);

	system("pause"); // windows命令行暂停
	return (0);
}

```

*