# NVIDIA PhysX SDK 4.1

Copyright (c) 2021 NVIDIA Corporation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
 * Neither the name of NVIDIA CORPORATION nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## Introduction

Welcome to the NVIDIA PhysX SDK source code repository. This depot includes the PhysX SDK and the Kapla Demo application.

The NVIDIA PhysX SDK is a scalable multi-platform physics solution supporting a wide range of devices, from smartphones to high-end multicore CPUs and GPUs. PhysX is already integrated into some of the most popular game engines, including Unreal Engine, and Unity3D. [PhysX SDK on developer.nvidia.com](https://developer.nvidia.com/physx-sdk).

## Documentation

Please see [Release Notes](http://gameworksdocs.nvidia.com/PhysX/4.1/release_notes.html) for updates pertaining to the latest version.

The full set of documentation can also be found in the repository under physx/documentation or online at http://gameworksdocs.nvidia.com/simulation.html 

Platform specific information can be found here:
* [Microsoft Windows](http://gameworksdocs.nvidia.com/PhysX/4.1/documentation/platformreadme/windows/readme_windows.html)
* [Linux](http://gameworksdocs.nvidia.com/PhysX/4.1/documentation/platformreadme/linux/readme_linux.html)
* [Google Android ARM](http://gameworksdocs.nvidia.com/PhysX/4.1/documentation/platformreadme/android/readme_android.html)
* [Apple macOS](http://gameworksdocs.nvidia.com/PhysX/4.1/documentation/platformreadme/mac/readme_mac.html)
* [Apple iOS](http://gameworksdocs.nvidia.com/PhysX/4.1/documentation/platformreadme/ios/readme_ios.html)
 

## Quick Start Instructions

Requirements:
* Python 2.7.6 or later
* CMake 3.12 or later

To begin, clone this repository onto your local drive.  Then change directory to physx/, run ./generate_projects.[bat|sh] and follow on-screen prompts.  This will let you select a platform specific solution to build.  You can then open the generated solution file with your IDE and kick off one or more configuration builds.

To build and run the Kapla Demo see [kaplademo/README.md](kaplademo/README.md).

## Acknowledgements

This depot contains external third party open source software copyright their respective owners.  See [kaplademo/README.md](kaplademo/README.md) and [externals/README.md](externals/README.md) for details.

## linux安装问题
### 环境
centOS7.9, x86_64, GCC8.3.1,  

### 编译问题
```
每次编译都会重编  
    ExternalProject_Add加上UPDATE_COMMAND ""  
    cmake_generate_projects.py中cleanupCompilerDir清理函数注释掉  
```

### 编译错误
```
GuBV4Build.cpp local_BuildHierarchy函数有restrict关键字限定，但调用处没有严格遵守。  
    去掉了restrict关键字。  

GCC8以上报错-Werror=class-memaccess，不知道为啥，在cmake_generate_projects.py:330 getCommonParams()中添加-Wno-error=class-memaccess也没用???  
    XnXmlSerialization.cpp:469 memset( &mScale, 0, sizeof( PxTolerancesScale ) );  对非凡类memset报错。  
    AcclaimLoader.cpp:677 memcpy(amcData.mFrameData, tempFrameData.begin(), sizeof(FrameData) * amcData.mNbFrames);  
    SampleNorthPoleBuilder.cpp:229 memset(samples,0,hfNumVerts*sizeof(PxHeightFieldSample));  

externals/glew-linux/include/GL/glew.h:1202:14: fatal error: GL/glu.h: 没有那个文件或目录  
    SamplePlatform.cmake添加包含头文件TARGET_INCLUDE_DIRECTORIES(SamplePlatfor PRIVATE ${PM_opengllinux_PATH}/include)  
        cmake_generate_projects.py getCommonParams()中要加上PM_opengllinux_PATH路径  

-Werror=stringop-overflow=  
    PhysXSampleApplication.cpp:841 error: ‘char* strncat(char*, const char*, size_t)’ specified bound 7 equals source length  
        改成了strcat  

SampleVehicle.cpp:1097:14: error: ‘physx::PxU32 GetFileSize(const char*)’ defined but not used [-Werror=unused-function] 
    SampleVehicle.cpp中的 GetFileSize, getPlatformName 函数注释掉  
    应该加-Wno-unused-function也可以的，但试了没用???
```

### 库问题
库找不到依赖，ldd查看，没有的要安装[Packages Search for Linux and Unix](https://pkgs.org/)  
```
externals/opengl-linux/lib64/libGL.so   
externals/opengl-linux/lib64/libXmu.so  
libX11-devel  
libXxf86vm-devel  
    /usr/include/X11/extensions/xf86vmode.h  
```

