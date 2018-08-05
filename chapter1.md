# PCL的编译与安装

**pcl需要第三方库的支持：Boost、Eigen、FLANN、Qhull、VTK、OpenNI2、\[QT、CUDA\]**

* **工具**  
    VS  
    cmake

* **编译Boost库**

若需要mpi支持，首先下载安装mpi，然后到boost\_1\_60\_0\tools\build\src\tools\mpi.jam中修改下面几行：

249-251 line

```
local microsoft_mpi_sdk_path = "C:\\Program Files (x86)\\Microsoft SDKs\\MPI" ;
local microsoft_mpi_path = "C:\\Program Files\\Microsoft MPI" ;
if [ GLOB $(microsoft_mpi_sdk_path)\\Include : mpi.h ]
```

260-262 line

```
options = <include>$(microsoft_mpi_sdk_path)/Include
<address-model>64:<library-path>$(microsoft_mpi_sdk_path)/Lib/x64
<library-path>$(microsoft_mpi_sdk_path)/Lib/x86
```

268 line

```
.mpirun = "\"$(microsoft_mpi_path)\\Bin\\mpiexec.exe"\" ;
```

然后以管理员身份运行VS自带的cmd\(“VS2013 x86 本机工具命令提示”\)，进入boost文件夹，运行bootstrap.bat,运行结束后会生成project-config.jam，打开并在第四行加上：using mpi ;（注意“；”前面有一个空格！） 接下来还是用cmd进入boost文件夹，运行如下命令编译boost：

```
Win32：
b2.exe toolset=msvc-12.0 address-model=32 --build-dir=build\x86 install --prefix="C:\Program Files (x86)\Boost" -j8
X64:
b2.exe toolset=msvc-12.0 address-model=64 --build-dir=build\x64 install --prefix="C:\Program Files\Boost" -j8
```

* **编译Eigen库**

使用cmake，分别设置eigen的source和build路径（source路径是含有CMakeLists.txt的文件夹，其实就是source的根目录），如build不存在，点击Configure会提示新建build文件夹，选择vs编译器，这里注意Configure时有CMAKE\_INSTALL\_PREFIX这个选项，默认为C:\Program Files\Eigen．这里的路径即为该软件最后的安装路径\(也是环境变量中要设置的EIGEN\_ROOT的路径，可设置为你想要的其它路径，后边的FLANN，QHULL，VTK也是一样道理\)．然后Generate。

以管理员身份运行VS（否则install时会失败），打开bulid文件夹下的eigen.sln工程，待加载完文件后，VS-&gt;生成-&gt;批生成-&gt;勾选ALL\_BUILD的Debug和 Release完成生成，完成后同理生成INSTALL\(Debug & Release\)。可以看到eigen安装路径中出现include文件夹。最后在环境变量中建立EIGEN\_ROOT变量，值为eigen的安装路径。

* **编译qhull库**

使用cmake，分别设置qhull的source和build路径，选择vs编译器．注意根据需要修改CMAKE\_INSTALL\_PREFIX，然后添加一个entry：

> Name: CMAKE\_DEBUG\_POSTFIX  
> Type: STRING  
> Value: -d

修改完后再次点Configure，然后Generate。以管理员身份运行VS并打开qhull.sln工程文件，待加载完文件后，完成后生成ALL\_BUILD（debug & release），然后生成INSTALL\(Debug & Release\)。完成后可以看到qhull安装路径中出现include和lib文件夹。最后在环境变量中建立QHULL\_ROOT变量，值为qhull安装地址。

* **编译flann库**

使用cmake，分别设置Flann的source和build路径，选择vs编译器．注意根据需要修改CMAKE\_INSTALL\_PREFIX，然后去掉BULID\_MATLAB\_BINDINGS和BULID\_PYTHON\_BINDINGS的勾选，不bulid它们。然后添加一个entry：

> Name: CMAKE\_DEBUG\_POSTFIX  
> Type: STRING  
> Value: -gd

修改完后再次点Configure，然后Generate。在C:\flann\src\cpp\flann\util\(源码\)中找到serialization.h文件 在92行BASIC\_TYPE\_SERIALIZER\(bool\)之后加入以下代码：

```
#ifdef _MSC_VER

BASIC_TYPE_SERIALIZER( unsigned __int64 );//注意此处__int64是两个下划线连一起

#endif
```

修改完后以管理员身份运行VS并打开flann.sln工程文件，待加载完文件后，（Debug & Release）生成all\_build，完成后生成install\(Debug & Release\)。完成后可以看到flann安装路径下出现include和lib文件夹。最后在环境变量中建立FLANN\_ROOT变量，值为flann安装路径。

* **编译QT**

```
configure -platform win32-msvc2015 -confirm-license -opensource -debug-and-release -opengl desktop -prefix "" -nomake examples
```

将qt的bin目录添加到环境变量path中。

注：如果需要编译64位qt，只需要打开vs64位命令行即可，还需要一些其他第三方软件，百度即可。

* **编译VTK库**

使用cmake，分别设置VTK的source和build路径，选择vs编译器．注意根据需要修改CMAKE\_INSTALL\_PREFIX选项，然后添加一个entry：

> Name: CMAKE\_DEBUG\_POSTFIX  
> Type: STRING  
> Value: -gd

修改完后再次点Configure，然后Generate。以管理员身份运行VS并打开VTK.sln工程文件，待加载完文件后，生成ALL\_BUILD（debug & release），完成后生成install（debug & release）。完成后可以看到VTK文件夹中出现include和lib文件夹。最后在环境变量中建立VTK\_ROOT，为VTK安装路径。

如果需要编译QT支持插件，在cmake时勾选VTK\_Group\_Qt，VTK\_RENDERING\_BACKEND可以是OpenGL也可以是OpenGL2，我使用OpenGL2。 Configure 后可能会出现报错，将VTK\_QT\_VERSION 修改成 5，QT\_QMAKE\_EXECUTABLE 为QTDIR\bin目录下的qmake.exe。点击 Configure 后仍然会报错，修改 Qt5\_DIR 路径为 QTDIR\lib\cmake\Qt5，再次 Configure 后若出现红色部分为NOTFOUND根据名字自行添加。Configure 后再点击 Generate完成二进制生成。然后打开sln生成ALL\_Build 和 Install。

> 添加了qt INSTALL会出错！原因是QVTKWidgetPlugin.dll也加了-gd，vs找不到，去掉即可。

生成完成后，在VTKBuildDIR\plugins\designer 中找到 QVTKWidgetPlugin.dll 插件，将其复制到 QtCreatorDIR\bin\plugins\designer目录中，用于在 Qt Creator的 designer中显示 QVTKWidget 控件。

注：如果编译qt插件，目录中不能包含中文路径，否则报错。建议其他的也都不要中文路径。

* **OpenNI2**

下载安装即可。

* **CUDA**

如果需要编译Kinfu等内容，需要Nvida显卡、CUDA支持，CUDA下载安装即可。

* **编译PCL库**

同样使用cmake，打开CMakeLists.txt，按照提示添加第三方的库文件，勾选自己需要编译的内容即可。  
注意：

> 在win10上用vs2015编译PCL1.8的时候，编译到visualization模块时,pcl\_visulalizer.cpp如下语句会报错。
>
> ```
> if (!pcl::visualization::getColormapLUT (static_cast<LookUpTableRepresentationProperties>(value), table))
>             break;
> 解决方案： 
> 将所有的
> static_cast<LookUpTableRepresentationProperties>(value)
> 修改成
> static_cast<LookUpTableRepresentationProperties>(int(value))
> ```
>
> pcl1.8.0 用eigen3.3编译失败，ndt2d无法编译，换3.2可以
>
> （3）带nurbs编译\(用于曲面拟合\):（[http://pointclouds.org/documentation/tutorials/bspline\_fitting.php\#bspline-fitting）](http://pointclouds.org/documentation/tutorials/bspline_fitting.php#bspline-fitting）)  
>     Please note that the modules for NURBS and B-splines are not enabled by default. Make sure you enable “BUILD\_surface\_on\_nurbs” in your ccmake configuration, by setting it to ON.  
>     If your license permits, also enable “USE\_UMFPACK” for sparse linear solving. This requires SuiteSparse \(libsuitesparse-dev in Ubuntu\) which is faster, allows more degrees of freedom \(i.e. control points\) and more data points.  
>     windows下需要编译SuiteSparse库才能勾选“USE\_UMFPACK”，比较麻烦，可以不用勾选。

* **添加环境变量**

将以上所有编译的库中含有bin目录的添加到path中即可，OpenNI2添加Tools目录。

**参考：**

[VS编译PCL1.8.0](http://blog.csdn.net/yzheately/article/details/50938322)

[VTK7.0&QTCreator5.6环境配置教程.pdf](https://wenku.baidu.com/view/ef13a7c94a7302768f9939ad.html)

---

* **现有exe安装**

一路next即可，添加环境变量同上。

