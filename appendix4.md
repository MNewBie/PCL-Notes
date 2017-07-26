# 新建QT界面项目

此处给出利用QT和VTK生成的插件，在QT界面中显示点云的例子。

* 1、新建QT GUI项目，原生QT或者VS插件建立都可以。(此处我的项目名字叫Qtttttt)
注意：VS添加“附加包含目录”、“附加库目录”、“附加依赖项”。QT则添加INCLUDEPATH、LIBS。

* 2、用Qt Designer打开.ui文件，左侧工具栏可以看到QVTK插件。

![](/images/qt_1.png)

拖入GUI中，调整合适大小。

![](/images/qt_2.png)

属性中可以看到默认名字为qvtkWidget

![](/images/qt_3.png)

* 3、修改代码

修改Qtttttt.h文件，添加如下内容

```
#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_Qtttttt.h"

// --------------- 添加 ------------
#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
// -----------------------------------

class Qtttttt : public QMainWindow
{
	Q_OBJECT

public:
	Qtttttt(QWidget *parent = Q_NULLPTR);

	// -------------- 添加----------------
	//点云数据存储
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	//初始化vtk部件
	void initialVtkWidget();

	private slots:
	//创建打开槽
	void onOpen();
	// ------------------------------=

private:
	Ui::QttttttClass ui;
};
```
修改Qtttttt.p文件，添加如下内容

```
#include "Qtttttt.h"
// ------------------ 添加-------------
#include <QMenu>				
#include <QFileDialog>	
#include <iostream>
#include <vtkRenderWindow.h>
// ------------------------------------

Qtttttt::Qtttttt(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);


	// ------------- 添加 ----------------------
	QMenu *littleTools;
	QAction *openAction;

	littleTools = new QMenu("File");
	ui.menuBar->addMenu(littleTools);
	openAction = new QAction("open");
	littleTools->addAction(openAction);

	//初始化vtk
	initialVtkWidget();
	//连接信号和槽
	connect(openAction, SIGNAL(triggered()), this, SLOT(onOpen()));
	// -------------------------------------------
}

// ----------------- 添加-----------------
void Qtttttt::initialVtkWidget()
{
	cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	viewer->addPointCloud(cloud, "cloud");

	ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());
	ui.qvtkWidget->update();
}

//读取文本型和二进制型点云数据
void Qtttttt::onOpen()
{
	//只能打开PCD文件
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open PointCloud"), ".",
		tr("Open PCD files(*.pcd)"));

	if (!fileName.isEmpty())
	{
		std::string file_name = fileName.toStdString();
		//sensor_msgs::PointCloud2 cloud2;
		pcl::PCLPointCloud2 cloud2;
		//pcl::PointCloud<Eigen::MatrixXf> cloud2;
		Eigen::Vector4f origin;
		Eigen::Quaternionf orientation;
		int pcd_version;
		int data_type;
		unsigned int data_idx;
		int offset = 0;
		pcl::PCDReader rd;
		rd.readHeader(file_name, cloud2, origin, orientation, pcd_version, data_type, data_idx);

		if (data_type == 0)
		{
			pcl::io::loadPCDFile(fileName.toStdString(), *cloud);
		}
		else if (data_type == 2)
		{
			pcl::PCDReader reader;
			reader.read<pcl::PointXYZ>(fileName.toStdString(), *cloud);
		}

		viewer->updatePointCloud(cloud, "cloud");
		viewer->resetCamera();
		ui.qvtkWidget->update();
	}
}
//--------------------------------
```

