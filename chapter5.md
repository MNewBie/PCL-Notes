# 可用Point类型(下)

* **目录**
    + [PointWithRange](#PointWithRange)
    + [PointWithViewpoint](#PointWithViewpoint)
    + [MomentInvariants](#MomentInvariants)
    + [PrincipalRadiiRSD](#PrincipalRadiiRSD)
    + [Boundary](#Boundary)
    + [PrincipalCurvatures](#PrincipalCurvatures)
    + [BounPFHSignature125dary](#PFHSignature125)
    + [FPFHSignature33](#FPFHSignature33)
    + [VFHSignature308](#VFHSignature308)
    + [Narf36](#Narf36)
    + [BorderDescription](#BorderDescription)
    + [IntensityGradient](#IntensityGradient)
    + [Histogram](#Histogram)
    + [PointWithScale](#PointWithScale)
    + [PointSurfel](#PointSurfel)
    

* **PointWithRange - float x, y, z (union with float point[4]), range;**<span id = "PointWithRange"/>

PointWithRange除了range包含从所获得的视点到采样点的距离测量值之外，其它与PointXYZI类似。

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
        float range;
    };
    float data_c[4];
};

```

* **PointWithViewpoint - float x, y, z, vp_x, vp_y, vp_z;**<span id = "PointWithViewpoint"/>

ointWithViewpoint除了vp_x、vp_y和vp_z以三维点表示所获得的视点之外，其它与PointXYZI一样。

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
        float vp_x;
        float vp_y;
        float vp_z;
    };
    float data_c[4];
};

```

* **MomentInvariants - float j1, j2, j3;**<span id = "MomentInvariants"/>

MomentInvariants是一个包含采样曲面上面片的三个不变矩的point类型，描述面片上质量的分布情况。查看MomentInvariantsEstimation以获得更多信息。

```
struct
{
    float j1,j2,j3;
};

```

* **PrincipalRadiiRSD - float r_min, r_max;**<span id = "PrincipalRadiiRSD"/>

PrincipalRadiiRSD是一个包含曲面块上两个RSD半径的point类型，查看RSDEstimation以获得更多信息。

```
struct
{
    float r_min,r_max;
};

```

* **Boundary - uint8_t boundary_point;**<span id = "Boundary"/>

Boundary存储一个点是否位于曲面边界上的简单point类型，查看BoundaryEstimation以获得更多信息。

```
struct
{
    uint8_t boundary_point;
};
```

* **PrincipalCurvatures - float principal_curvature[3], pc1, pc2;**<span id = "PrincipalCurvatures"/>

PrincipalCurvatures包含给定点主曲率的简单point类型。查看PrincipalCurvaturesEstimation以获得更多信息。

```
struct
{
    union
    {
        float principal_curvature[3];
        struct
        {
            float principal_curvature_x;
            float principal_curvature_y;
            float principal_curvature_z;
        };
    };
    float pc1;
    float pc2;
};
```

* **PFHSignature125 - float pfh[125];**<span id = "PFHSignature125"/>

PFHSignature125包含给定点的PFH（点特征直方图）的简单point类型,查看PFHEstimation以获得更多信息。

```
struct
{
    float histogram[125];
};
```

* **FPFHSignature33 - float fpfh[33];**<span id = "FPFHSignature33"/>

FPFHSignature33包含给定点的FPFH（快速点特征直方图）的简单point类型，查看FPFHEstimation以获得更多信息。

```
struct
{
    float histogram[33];
};
```

* **VFHSignature308 - float vfh[308];**<span id = "VFHSignature308"/>

VFHSignature308包含给定点VFH（视点特征直方图）的简单point类型，查看VFHEstimation以获得更多信息。

```
struct
{
    float histogram[308];
};
```

* **Narf36 - float x, y, z, roll, pitch, yaw; float descriptor[36];**<span id = "Narf36"/>

Narf36包含给定点NARF（归一化对齐半径特征）的简单point类型，查看NARFEstimation以获得更多信息。

```
struct
{
    float x,y,z,roll,pitch,yaw;
    float descriptor[36];
};
```

* **BorderDescription - int x, y; BorderTraits traits;**<span id = "BorderDescription"/>

BorderDescription包含给定点边界类型的简单point类型，看BorderEstimation以获得更多信息。

```
struct
{
    int x,y;
    BorderTraitstraits;
};
```

* **IntensityGradient - float gradient[3];**<span id = "IntensityGradient"/>

IntensityGradient包含给定点强度的梯度point类型，查看IntensityGradientEstimation以获得更多信息。

```
struct
{
    union
    {
        float gradient[3];
        struct
        {
            float gradient_x;
            float gradient_y;
            float gradient_z;
        };
    };
};
```

* **Histogram - float histogram[N];**<span id = "Histogram"/>

Histogram用来存储一般用途的n维直方图。

```
template<int N>
struct Histogram
{
    float histogram[N];
};
```

* **PointWithScale - float x, y, z, scale;**<span id = "PointWithScale"/>

PointWithScale除了scale表示某点用于几何操作的尺度（例如，计算最近邻所用的球体半径，窗口尺寸等等），其它的和PointXYZI一样。

```
struct
{
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
    float scale;
};
```

* **PointSurfel - float x, y, z, normal[3], rgba, radius, confidence, curvature;**<span id = "PointSurfel"/>

PointSurfel存储XYZ坐标、曲面法线、RGB信息、半径、可信度和曲面曲率的复杂point类型。

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
        uint32_trgba;
        float radius;
        float confidence;
        float curvature;
    };
    float data_c[4];
};
```

---

以上内容来自<http://www.pclcn.org/study/news.php?lang=cn&class3=105>


