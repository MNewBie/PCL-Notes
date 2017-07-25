# 数据输入输出

* **从PCD文件中读取点云文件**

```
#include <iostream> //标准c++库输入输出相关头文件
#include <pcl/io/pcd_io.h> // pcd读写相关头文件
#include <pcl/point_types.h> // pcl中支持的点类型头文件

int main(int argc, char** argv)
{
	// 定义点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 读取点云，失败返回-1
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("test.pcd", *cloud) == -1)
	{
		PCL_ERROR("couldn't read file\n");
		return (-1);
	}
	std::cout << "点云大小：" << cloud->size() << std::endl; // 输出点云大小 cloud->width * cloud->height
	
	system("pause"); // windows命令行暂停
	return (0);
}

```