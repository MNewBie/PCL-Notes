# 工具函数

给出一些可能会用到的工具函数。

* **时间计算**

pcl中计算程序运行时间有很多函数，其中利用控制台的时间计算是：

````
#include <pcl/console/time.h>

pcl::console::TicToc time; time.tic(); 

+程序段 + 

cout<<time.toc()/1000<<"s"<<endl;

```

就可以以秒输出“程序段”的运行时间。

* **pcl::PointCloud::Ptr和pcl::PointCloud的两个类相互转换**

```
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointer(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> cloud;
cloud = *cloudPointer;
cloudPointer = cloud.makeShared();
```

* **如何查找点云的x，y，z的极值？**

```
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile<pcl::PointXYZ> ("your_pcd_file.pcd", *cloud);
pcl::PointXYZ minPt, maxPt;
pcl::getMinMax3D (*cloud, minPt, maxPt);
```