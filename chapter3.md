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
}

```