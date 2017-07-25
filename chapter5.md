# 可用Point类型(下)

* **PointWithViewpoint - float x, y, z, vp_x, vp_y, vp_z;**

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

* **MomentInvariants - float j1, j2, j3;**

MomentInvariants是一个包含采样曲面上面片的三个不变矩的point类型，描述面片上质量的分布情况。查看MomentInvariantsEstimation以获得更多信息。

```
struct
{
    float j1,j2,j3;
};

```

* **PrincipalRadiiRSD - float r_min, r_max;**

PrincipalRadiiRSD是一个包含曲面块上两个RSD半径的point类型，查看RSDEstimation以获得更多信息。

```
struct
{
    float r_min,r_max;
};

```

* **Boundary - uint8_t boundary_point;**

Boundary存储一个点是否位于曲面边界上的简单point类型，查看BoundaryEstimation以获得更多信息。

```
struct
{
    uint8_t boundary_point;
};
```

* **PrincipalCurvatures - float principal_curvature[3], pc1, pc2;**

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

* **PFHSignature125 - float pfh[125];**

PFHSignature125包含给定点的PFH（点特征直方图）的简单point类型,查看PFHEstimation以获得更多信息。

```
struct
{
    float histogram[125];
};
```

* **FPFHSignature33 - float fpfh[33];**

FPFHSignature33包含给定点的FPFH（快速点特征直方图）的简单point类型，查看FPFHEstimation以获得更多信息。

```
struct
{
    float histogram[33];
};
```

* **VFHSignature308 - float vfh[308];**

VFHSignature308包含给定点VFH（视点特征直方图）的简单point类型，查看VFHEstimation以获得更多信息。

```
struct
{
    float histogram[308];
};
```

* **Narf36 - float x, y, z, roll, pitch, yaw; float descriptor[36];**

Narf36包含给定点NARF（归一化对齐半径特征）的简单point类型，查看NARFEstimation以获得更多信息。

```
struct
{
    float x,y,z,roll,pitch,yaw;
    float descriptor[36];
};
```

* **BorderDescription - int x, y; BorderTraits traits;**

BorderDescription包含给定点边界类型的简单point类型，看BorderEstimation以获得更多信息。

```
struct
{
    int x,y;
    BorderTraitstraits;
};
```
















