# 可用Point类型

为了涵盖能想到的所有可能的情况，PCL中定义了大量的point类型。下面是一小段，在point_types.hpp中有完整目录，这个列表很重要，因为在定义你自己的类型之前，需要了解已有的类型，如果你需要的类型，已经存在于PCL，那么就不需要重复定义了。

* **PointXYZ–成员变量: float x, y, z;**

PointXYZ是使用最常见的一个点数据类型，因为它只包含三维xyz坐标信息，这三个浮点数附加一个浮点数来满足存储对齐，用户可利用points[i].data[0]，或者points[i].x访问点的x坐标值。

```
union
{
    float data[4];
    struct
    {
        float x;
        float y;
        float z;
    };
};
```

* **PointXYZI–成员变量: float x, y, z, intensity;**

