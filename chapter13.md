# 精配准

精配准大多采用ICP(Iterative Closest Point)算法或者其变种算法完成。ICP算法本质上是基于最小二乘法的最优配准方法。该算法重复进行选择对应关系点对， 计算最优刚体变换，直到满足正确配准的收敛精度要求。

改进的ICP可以采用GPU对迭代进行加速，或者针对最近点选取上采用Point to Point、Point to Plane、Point to Projection等一些方式完成，或者针对收敛函数做一些改变。

PCL中也提供了ICP算法和一些改进算法。

* **ICP**

参考：http://pointclouds.org/documentation/tutorials/iterative_closest_point.php#iterative-closest-point

* **GeneralizedIterativeClosestPoint**

论文：http://www.robots.ox.ac.uk/~avsegal/resources/papers/Generalized_ICP.pdf

```
pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

gicp.setInputSource(cloud_src);
gicp.setInputTarget(cloud_tgt);

gicp.setMaximumIterations(100);
gicp.setTransformationEpsilon(1e-6);
gicp.setEuclideanFitnessEpsilon(0.1);
gicp.setMaxCorrespondenceDistance(0.01);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);
gicp.align(*cloud_final);
```

* **Sparse ICP**

参考:http://lgg.epfl.ch/sparseicp