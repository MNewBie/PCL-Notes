# 可用Point类型(上)

为了涵盖能想到的所有可能的情况，PCL中定义了大量的point类型。下面是一小段，在point_types.hpp中有完整目录，这个列表很重要，因为在定义你自己的类型之前，需要了解已有的类型，如果你需要的类型，已经存在于PCL，那么就不需要重复定义了。

* **目录**
    + [PointXYZ](#PointXYZ)
    + [PointXYZI](#PointXYZ)
    + [PointXYZRGBA](#PointXYZRGBA)
    + [PointXYZRGB ](#PointXYZRGB )
    + [PointXY](#PointXY)
    + [InterestPoint](#InterestPoint)
    + [Normal](#Normal)
    + [PointNormal](#PointNormal)
    + [PointXYZRGBNormal](#PointXYZRGBNormal)
    + [PointXYZINormal](#PointXYZINormal)
    

* **PointXYZ–成员变量: float x, y, z;**<span id = "PointXYZ" />

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

* **PointXYZI–成员变量: float x, y, z, intensity;**<span id = "PointXYZI"/>



PointXYZI是一个简单的XYZ坐标加intensity的point类型，理想情况下，这四个变量将新建单独一个结构体，并且满足存储对齐，然而，由于point的大部分操作会把data[4]元素设置成0或1（用于变换），不能让intensity与xyz在同一个结构体中，如果这样的话其内容将会被覆盖。例如，两个点的点积会把他们的第四个元素设置成0，否则该点积没有意义，等等。因此，对于兼容存储对齐，用三个额外的浮点数来填补intensity，这样在存储方面效率较低，但是符合存储对齐要求，运行效率较高。

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
union
{
    struct
    {
        float intensity;
    };
    float data_c[4];
};
```

* **PointXYZRGBA–成员变量: float x, y, z; uint32_t rgba;**<span id = "PointXYZRGBA"/>


除了rgba信息被包含在一个整型变量中，其它的和PointXYZI类似。

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
union
{
    struct
    {
        uint32_t rgba;
    };
    float data_c[4];
};

```

* **PointXYZRGB - float x, y, z, rgb;** <span id = "PointXYZRGB"/>

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
union
{
    struct
    {
        float rgb;
    };
    float data_c[4];
};
```

* **PointXY-float x, y;**<span id = "PointXY"/>

简单的二维x-y point结构

```
struct
{
    float x;
    float y;
};
```

* **InterestPoint-float x, y, z, strength;**<span id = "InterestPoint"/>

除了strength表示关键点的强度的测量值，其它的和PointXYZI类似。

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
union
{
    struct
    {
        float strength;
    };
    float data_c[4];
};
```

* **Normal - float normal[3], curvature;**<span id = "Normal"/>

另一个最常用的数据类型，Normal结构体表示给定点所在样本曲面上的法线方向，以及对应曲率的测量值（通过曲面块特征值之间关系获得——查看NormalEstimation类API以便获得更多信息，后续章节有介绍），由于在PCL中对曲面法线的操作很普遍，还是用第四个元素来占位，这样就兼容SSE和高效计算，例如，用户访问法向量的第一个坐标，可以通过points[i].data\_n[0]或者points[i].normal[0]或者points[i].normal\_x，再一次强调，曲率不能被存储在同一个结构体中，因为它会被普通的数据操作覆盖掉。

```
union
{
    float data_n[4];
    float normal[3];
    struct
    {
        float normal_x;
        float normal_y;
        float normal_z;
    };
}
union
{
    struct
    {
        float curvature;
    };
    float data_c[4];
};
```

* **PointNormal - float x, y, z; float normal[3], curvature;**<span id = "PointNormal"/>


PointNormal是存储XYZ数据的point结构体，并且包括采样点对应法线和曲率。

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
union
{
    float data_n[4];
    float normal[3];
    struct
    {
        float normal_x;
        float normal_y;
        float normal_z;
    };
};
union
{
    struct
    {
        float curvature;
    };
    float data_c[4];
};
```

* **PointXYZRGBNormal - float x, y, z, rgb, normal[3], curvature;**<span id = "PointXYZRGBNormal"/>


PointXYZRGBNormal存储XYZ数据和RGB颜色的point结构体，并且包括曲面法线和曲率

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
union
{
    float data_n[4];
    float normal[3];
    struct
    {
        float normal_x;
        float normal_y;
        float normal_z;
    };
}
union
{
    struct
    {
        float rgb;
        float curvature;
    };
    float data_c[4];
};

```

* **PointXYZINormal - float x, y, z, intensity, normal[3], curvature;**<span id = "PointXYZINormal"/>


PointXYZINormal存储XYZ数据和强度值的point结构体，并且包括曲面法线和曲率。

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
union
{
    float data_n[4];
    float normal[3];
    struct
    {
        float normal_x;
        float normal_y;
        float normal_z;
    };
}
union
{
    struct
    {
        float intensity;
        float curvature;
    };
    float data_c[4];
};
```
