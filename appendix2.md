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