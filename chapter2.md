# PCD（点云数据）文件格式

* **pcd文件数据举例**

```
# .PCD v.7 - Point Cloud Data file format
VERSION .7
FIELDS x y z rgb
SIZE 4 4 4 4
TYPE F FFF
COUNT 1 1 1 1
WIDTH 213
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 213
DATA ascii
0.93773 0.33763 0 4.2108e+06
0.90805 0.35641 0 4.2108e+06
```

* **文件格式详解**

**PCD版本**

在点云库（PCL）1.0版本发布之前，PCD文件格式有不同的修订号。这些修订号用PCD\_Vx来编号（例如，PCD\_V5、PCD\_V6、PCD\_V7等等），代表PCD文件的0.x版本号。然而PCL中PCD文件格式的正式发布是0.7版本（PCD\_V7）。

**文件头格式**

每一个PCD文件包含一个文件头，它确定和声明文件中存储的点云数据的某种特性。PCD文件头必须用ASCII码来编码。PCD文件中指定的每一个文件头字段以及ascii点数据都用一个新行（\n）分开了，从0.7版本开始，PCD文件头包含下面的字段：

·VERSION –指定PCD文件版本

·FIELDS –指定一个点可以有的每一个维度和字段的名字。例如：
FIELDS x y z                                   # XYZ data
FIELDS x y z rgb                          # XYZ + colors
FIELDS x y z normal_xnormal\_y normal\_z         # XYZ + surface normals
FIELDS j1 j2 j3                                # moment invariants
...

·SIZE –用字节数指定每一个维度的大小。例如：
unsigned char/char has 1 byte
unsigned short/short has 2 bytes
unsignedint/int/float has 4 bytes
double has 8 bytes

·TYPE –用一个字符指定每一个维度的类型。现在被接受的类型有：
I –表示有符号类型int8（char）、int16（short）和int32（int）；
U – 表示无符号类型uint8（unsigned char）、uint16（unsigned short）和uint32（unsigned int）；
F –表示浮点类型。

·COUNT –指定每一个维度包含的元素数目。例如，x这个数据通常有一个元素，但是像VFH这样的特征描述子就有308个。实际上这是在给每一点引入n维直方图描述符的方法，把它们当做单个的连续存储块。默认情况下，如果没有COUNT，所有维度的数目被设置成1。

·WIDTH –用点的数量表示点云数据集的宽度。根据是有序点云还是无序点云，WIDTH有两层解释：
1)它能确定无序数据集的点云中点的个数（和下面的POINTS一样）；
2)它能确定有序点云数据集的宽度（一行中点的数目）。
注意：有序点云数据集，意味着点云是类似于图像（或者矩阵）的结构，数据分为行和列。这种点云的实例包括立体摄像机和时间飞行摄像机生成的数据。有序数据集的优势在于，预先了解相邻点（和像素点类似）的关系，邻域操作更加高效，这样就加速了计算并降低了PCL中某些算法的成本。
例如：
WIDTH 640       # 每行有640个点

·HEIGHT –用点的数目表示点云数据集的高度。类似于WIDTH ，HEIGHT也有两层解释：
1)它表示有序点云数据集的高度（行的总数）；
2)对于无序数据集它被设置成1（被用来检查一个数据集是有序还是无序）。
有序点云例子：
WIDTH 640       # 像图像一样的有序结构，有640行和480列，
HEIGHT 480      # 这样该数据集中共有640*480=307200个点
无序点云例子：
WIDTH 307200
HEIGHT 1        # 有307200个点的无序点云数据集

·VIEWPOINT–指定数据集中点云的获取视点。VIEWPOINT有可能在不同坐标系之间转换的时候应用，在辅助获取其他特征时也比较有用，例如曲面法线，在判断方向一致性时，需要知道视点的方位，
视点信息被指定为平移（txtytz）+四元数（qwqxqyqz）。默认值是：
VIEWPOINT 0 0 0 1 0 0 0

·POINTS–指定点云中点的总数。从0.7版本开始，该字段就有点多余了，因此有可能在将来的版本中将它移除。
例子：
POINTS 307200   #点云中点的总数为307200

·DATA –指定存储点云数据的数据类型。从0.7版本开始，支持两种数据类型：ascii和二进制。查看下一节可以获得更多细节。
注意：文件头最后一行（DATA）的下一个字节就被看成是点云的数据部分了，它会被解释为点云数据。
警告：PCD文件的文件头部分必须以上面的顺序精确指定，也就是如下顺序：
VERSION、FIELDS、SIZE、TYPE、COUNT、WIDTH、HEIGHT、VIEWPOINT、POINTS、DATA
之间用换行隔开。

* **数据存储类型**

在0.7版本中，.PCD文件格式用两种模式存储数据：
如果以ASCII形式，每一点占据一个新行：
p_1
p_2
...
p_n
注意：从PCL 1.0.1版本开始，用字符串“nan”表示NaN，此字符表示该点的值不存在或非法等。

* **其他文件格式**

PLY是一种多边形文件格式，由Stanford大学的Turk等人设计开发；

STL是3D Systems公司创建的模型文件格式，主要应用于CAD、CAM领域；

OBJ是从几何学上定义的文件格式，首先由Wavefront Technologies开发；

X3D是符合ISO标准的基于XML的文件格式，表示3D计算机图形数据；

其他许多种格式。


