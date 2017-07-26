# 常用函数(1)

给出一些可能会用到的工具函数。代码来自<https://segmentfault.com/a/1190000007125502>

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

* **知道需要保存点的索引，从原点云中拷贝点到新点云**

```
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile<pcl::PointXYZ>("C:\office3-after21111.pcd", *cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
std::vector<int > indexs = { 1, 2, 5 };
pcl::copyPointCloud(*cloud, indexs, *cloudOut);
```

* **如何从点云里删除和添加点**

```
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile<pcl::PointXYZ>("C:\office3-after21111.pcd", *cloud);
pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
cloud->erase(index);//删除第一个
index = cloud->begin() + 5;
cloud->erase(cloud->begin());//删除第5个
pcl::PointXYZ point = { 1, 1, 1 };
//在索引号为5的位置1上插入一点，原来的点后移一位
cloud->insert(cloud->begin() + 5, point);
cloud->push_back(point);//从点云最后面插入一点
std::cout << cloud->points[5].x;//输出1
```

* **链接两个点云字段（两点云大小必须相同）**

```
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile("/home/yxg/pcl/pcd/mid.pcd",*cloud);
pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
ne.setInputCloud(cloud);
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
ne.setSearchMethod(tree);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>()); 
ne.setKSearch(8);
 //ne.setRadisuSearch(0.3);
ne.compute(*cloud_normals);    
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_nomal (new pcl::PointCloud<pcl::PointNormal>);
pcl::concatenateFields(*cloud,*cloud_normals,*cloud_with_nomal);
```

* **如何从点云中删除无效点**

pcl中的无效点是指：点的某一坐标值为nan.

```
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
    
using namespace std;
typedef pcl::PointXYZRGBA point;
typedef pcl::PointCloud<point> CloudType;
    
int main (int argc,char **argv)
{
  CloudType::Ptr cloud (new CloudType);
  CloudType::Ptr output (new CloudType);
              
  pcl::io::loadPCDFile(argv[1],*cloud);
  cout<<"size is:"<<cloud->size()<<endl;
                     
  vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud,*output,indices);
  cout<<"output size:"<<output->size()<<endl;
             
  pcl::io::savePCDFile("out.pcd",*output);
    
  return 0;
}
```

* **计算质心**

```
Eigen::Vector4f centroid;  //质心
pcl::compute3DCentroid(*cloud_smoothed,centroid); //估计质心的坐标
```

* **从网格提取顶点（将网格转化为点）**

```
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
using namespace pcl;
int main(int argc,char **argv)
{
   pcl::PolygonMesh mesh;
  //   pcl::io::loadPolygonFileOBJ(argv[1], mesh);
  pcl::io::loadPLYFile(argv[1],mesh);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new     pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
  pcl::io::savePCDFileASCII("result.pcd", *cloud);
  return 0;
}
```

以上代码可以从.obj或.ply面片格式转化为点云类型。

