# 精配准

精配准大多采用ICP(Iterative Closest Point)算法或者其变种算法完成。ICP算法本质上是基于最小二乘法的最优配准方法。该算法重复进行选择对应关系点对， 计算最优刚体变换，直到满足正确配准的收敛精度要求。

改进的ICP可以采用GPU对迭代进行加速，或者针对最近点选取上采用Point to Point、Point to Plane、Point to Projection等一些方式完成，或者针对收敛函数做一些改变。

PCL中也提供了ICP算法和一些改进算法。


gicp

sicp