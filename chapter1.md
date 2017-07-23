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
>  options = <include>$(microsoft_mpi_sdk_path)/Include
          <address-model>64:<library-path>$(microsoft_mpi_sdk_path)/Lib/x64
          <library-path>$(microsoft_mpi_sdk_path)/Lib/x86
 
 