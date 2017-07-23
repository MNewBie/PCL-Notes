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






 
 