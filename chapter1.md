# PCL的编译与安装

**pcl需要第三方库的支持：Boost、Eigen、FLANN、Qhull、VTK、OpenNI、[CUDA]**

* **工具**
    VS
    cmake

* **编译Boost库**

若需要mpi支持，首先下载安装mpi，然后到boost_1_60_0\tools\build\src\tools\mpi.jam中修改下面几行：

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
然后以管理员身份运行VS自带的cmd(“VS2013 x86 本机工具命令提示”)，进入boost文件夹，运行bootstrap.bat,运行结束后会生成project-config.jam，打开并在第四行加上：using mpi ;（注意“；”前面有一个空格！） 接下来还是用cmd进入boost文件夹，运行如下命令编译boost：

```
Win32：
b2.exe toolset=msvc-12.0 address-model=32 --build-dir=build\x86 install --prefix="C:\Program Files (x86)\Boost" -j8
X64:
b2.exe toolset=msvc-12.0 address-model=64 --build-dir=build\x64 install --prefix="C:\Program Files\Boost" -j8
```

* **编译Eigen库**

使用cmake，分别设置eigen的source和build路径（source路径是含有CMakeLists.txt的文件夹，其实就是source的根目录），如build不存在，点击Configure会提示新建build文件夹，选择vs编译器，这里注意Configure时有CMAKE\_INSTALL\_PREFIX这个选项，默认为C:\Program Files\Eigen．这里的路径即为该软件最后的安装路径(也是环境变量中要设置的EIGEN\_ROOT的路径，可设置为你想要的其它路径，后边的FLANN，QHULL，VTK也是一样道理)．然后Generate。 

**以管理员身份运行VS**（否则install时会失败），打开bulid文件夹下的eigen.sln工程，待加载完文件后，VS->生成->批生成->勾选ALL\_BUILD的Debug和 Release完成生成，完成后同理生成INSTALL(Debug and Release)。可以看到eigen安装路径中出现include文件夹。最后在环境变量中建立EIGEN\_ROOT变量，值为eigen的安装路径。

* **编译qhull**
使用cmake，分别设置qhull的source和build路径，如build不存在，点击Configure会提示新建build文件夹，选择vs编译器．注意根据需要修改CMAKE\_INSTALL\_PREFIX，然后添加一个entry：

>Name: CMAKE_DEBUG_POSTFIX
Type: STRING
Value: -d

修改完后再次点Configure，然后Generate。以管理员身份运行VS2013并打开qhull.sln工程文件，待加载完文件后，完成后生成ALL_BUILD（debug和release），然后生成INSTALL(Debugand Release)。完成后可以看到qhull安装路径中出现include和lib文件夹。最后在环境变量中建立QHULL_ROOT变量，值为qhull安装地址。



 
 