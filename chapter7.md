# 点云可视化

可视化（Visualization）是利用计算机图形学和图像处理技术，将数据转换成图形或图像在屏幕显示出来，并且进行交互处理的理论、方法和技术。

PCL中pcl\_visualization库中提供了可视化相关的数据结构和组件，其主要是为了可视化其他模块算法处理后的结果，可直观的反馈给用户。其依赖于pcl\_common、pcl\_range\_image、pcl\_kdtree、pcl\_IO模块以及VTK外部开源可视化库。下面给出2个常用的可视化类。

* **pcl::visualization::PCLVisualizer**

PCLVisualizer是PCL可视化3D点云的主要类。其内部实现了添加各种3D对象以及交互的实现等，比其他类实现的功能更齐全。

**基础显示功能：** 显示点云、网格、设置颜色、连线

```
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
// 包含相关头文件
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;

int main()
{
	// 读取点云
	pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("read1.pcd", *cloud1);

	pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("read2.pcd", *cloud2);

	// 定义对象
	pcl::visualization::PCLVisualizer viewer; 
	//设置背景颜色，默认黑色
	viewer.setBackgroundColor(100, 100, 100); // rgb

	// --- 显示点云数据 ----
	// "cloud1" 为显示id，默认cloud,显示多个点云时用默认会报警告。
	viewer.addPointCloud(cloud1, "cloud1"); 

	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(cloud2, 255, 0, 0); // rgb
	// 将点云设置颜色，默认白色
	viewer.addPointCloud(cloud2, red, "cloud2");

	// 将两个点连线
	PointT temp1 = cloud1->points[0];
	PointT temp2 = cloud1->points[1];
	viewer.addLine(temp1, temp2, "line0"); 
	// 同样可以设置线的颜色，
	//viewer.addLine(temp1, temp2, 255，0，0， "line0");

	// --- 显示网格数据 ---
	pcl::PolygonMesh mesh;
	pcl::io::loadPLYFile("read.ply", mesh);

	viewer.addPolygonMesh(mesh);


	// 开始显示2种方法,任选其一
	// 1. 阻塞式
	viewer.spin();

	// 2. 非阻塞式
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		// 可添加其他操作
	}

	system("pause");
	return 0;
}
```

**高级功能：**设置回掉函数进行交互、显示区域分割

	+ 按键事件
	
```
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

// 回掉函数所用数据结构
struct callback_args {
	bool *isShow;
	pcl::PointCloud<pcl::PointXYZ>::Ptr orgin_points;
	pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

// 按键事件回掉函数
void kb_callback(const pcl::visualization::KeyboardEvent& event, void* args)
{
	if (event.keyDown() && event.getKeyCode() == 'a')
	{
		std::cout << "a has pressed" << std::endl;
		struct callback_args* data = (struct callback_args *)args;
		if (*(data->isShow))
		{
			data->viewerPtr->removePointCloud("cloud");
			*(data->isShow) = false;
			std::cout << "remove" << std::endl;
		}
		else {
			data->viewerPtr->addPointCloud(data->orgin_points, "cloud");
			*(data->isShow) = true;
			std::cout << "add" << std::endl;
		}
	}
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("bunny.pcd", *cloud);
	pcl::console::print_highlight("load cloud !\n");

	// 定义对象
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
	viewer->addPointCloud(cloud, "cloud");

	// 初始化参数
	bool isShow = true;
	struct callback_args kb_args;
	kb_args.isShow = &isShow;
	kb_args.orgin_points = cloud;
	kb_args.viewerPtr = viewer;

	// 设置回掉函数
	viewer->registerKeyboardCallback(kb_callback, (void*)&kb_args);

	viewer->spin();

	return 0;
}
```

	+ 点选取事件

```
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

struct callback_args {
	// structure used to pass arguments to the callback function
	pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d;
	pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
	struct callback_args* data = (struct callback_args *)args;

	if (event.getPointIndex() == -1)
		return;
	int index = event.getPointIndex();
	std::cout << "index: " << index << std::endl;
	pcl::PointXYZ current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z);
	data->clicked_points_3d->points.push_back(current_point);
	// Draw clicked points in red:
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(data->clicked_points_3d, 255, 0, 0);
	data->viewerPtr->removePointCloud("clicked_points");
	data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
	data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("bunny.pcd", *cloud);
	pcl::console::print_highlight("load cloud !\n");

	pcl::visualization::PCLVisualizer viewer;

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud, 0, 255, 0);
	viewer.addPointCloud(cloud, green, "cloud");

	// Add point picking callback to viewer:
	struct callback_args cb_args;
	pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>);
	cb_args.clicked_points_3d = clicked_points_3d;
	cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer);
	viewer.registerPointPickingCallback(pp_callback, (void*)&cb_args);
	pcl::console::print_highlight("Shift+click on three floor points, then press 'Q'...\n");

	// Spin until 'Q' is pressed:
	viewer.spin();

	system("pause");
	return 0;
}
```

	+ 区域选取事件

```
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

struct callback_args{
	// structure used to pass arguments to the callback function
	pcl::PointCloud<pcl::PointXYZ>::Ptr orgin_points;
	pcl::PointCloud<pcl::PointXYZ>::Ptr chosed_points_3d;
	pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void ap_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
{
	struct callback_args* data = (struct callback_args *)args;
	std::vector<int> indiecs;

	if (!event.getPointsIndices(indiecs))
		return;
	for (int i = 0; i < indiecs.size(); ++i)
	{
		data->chosed_points_3d->push_back(data->orgin_points->points[indiecs[i]]);
	}
	
	// Draw clicked points in red:
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(data->chosed_points_3d, 255, 0, 0);
	data->viewerPtr->removePointCloud("chosed_points");
	data->viewerPtr->addPointCloud(data->chosed_points_3d, red, "chosed_points");
	data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "chosed_points");
	std::cout << "selected " << indiecs.size() << " points , now sum is " << data->chosed_points_3d->size() << std::endl;
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("rabbit.pcd", *cloud);
	pcl::console::print_highlight("load cloud !\n");

	pcl::visualization::PCLVisualizer viewer;

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud, 0, 255, 0);
	viewer.addPointCloud(cloud, green, "cloud");

	// Add point picking callback to viewer:
	struct callback_args cb_args;
	cb_args.orgin_points = cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr chosed_points_3d(new pcl::PointCloud<pcl::PointXYZ>);
	cb_args.chosed_points_3d = chosed_points_3d;
	cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer);
	viewer.registerAreaPickingCallback(ap_callback, (void*)&cb_args);
	pcl::console::print_highlight("press x enter slected model, then press 'qQ'...\n");

	// Spin until 'Q' is pressed:
	viewer.spin();

	system("pause");
	return 0;
}
```

	+ 显示区域分割

pcl可以将显示区域分割，从(xmin,ymin)到(xmax,ymax)一个矩形区域，范围是（0，1）。左下角(0,0)，右上角(1,1)。之前所有的函数都支持区域显示。

```
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// 包含相关头文件
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;

int main()
{
	// 读取点云
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("read.pcd", *cloud);

	// 定义对象
	pcl::visualization::PCLVisualizer viewer; 

	int v1(1); // viewport
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.setBackgroundColor(255, 0, 0, v1);
	viewer.addPointCloud(cloud, "cloud1", v1);;

	int v2(2);// viewport
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v1);
	viewer.setBackgroundColor(0, 255, 0, v2);
	viewer.addPointCloud(cloud, "cloud2", v2);;

	viewer.spin();

	system("pause");
	return 0;
}
```

* **pcl::visualization::CloudViewer**

```
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// 包含相关头文件
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZ PointT;

int main()
{
	// 读取点云
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("read.pcd", *cloud);

	pcl::visualization::CloudViewer viewer("simple cloud viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
		// todo::
	}

	system("pause");
	return 0;
}
```
