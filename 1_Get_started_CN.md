# 1. 如何使用OpenCorr

OpenCorr面向的是具备一定C++程序开发基础的用户。这套程序开发库在微软Windows 10专业版上的Visual Studio 2019（VS2019 社区版）内开发和测试，目前开发环境已升级至Windows11专业版和VS2022，因此使用者需要掌握这一集成开发环境（Integrated development environment, IDE）的基本配置和使用技能。由于代码遵循ISO C++ 14标准开发，而且依赖的几个开源程序库也是类似的情况，因此OpenCorr可以在其它操作系统（诸如Linux或Unix）中或使用其它C++编译器（例如GCC）编译。我们在以下两小节中分别展示了Windows系统中运用Visual Studio和macOS系统中运用Visual Studio Code (VS Code)的作为IDE配置方法。

## 1.1 Windows中的IDE配置

编译和运行OpenCorr的程序和范例首先需要正确安装以下四个开源程序库：

- Eigen 3.4.0 (eigen.tuxfamily.org)，用于二维矩阵的基本操作；
- OpenCV 4.10.0 (opencv.org)，用于图像的读写，2D图像特征提取和匹配；
- FFTW 3.3.5 (fftw.org)，用来快速计算互相关判据；
- nanoflann 1.7.0 (https://github.com/jlblancoc/nanoflann)，用于快速估计点云中某一点的近邻点。

上述开发库大多在图像处理领域久负盛名，其官网提供非常详尽的指南和下载资源。在Windows 10系统上安装可以归纳为以下几个主要步骤：

（1）从官网上下载这些开发库的源代码（包括.h和.cpp等文件）以及相应的静态库（.lib文件）或动态库（.dll文件）。

（2）将这些文件放到合适的目录下，图1.1展示了我计算机上的目录结构，首先在Visual Studio中创建一个名为OpenCorr的解决方案及同名的项目，OpenCor源代码的目录（OpenCorr）与其它开发库的目录并列放在主目录（D:\OpenCorr\）下。

![](D:/OpenCorr/Release_GitHub/img/vs_solution.png)

图1.1 Visual Studio解决方案目录结构的示例



（3）在IDE中添加相应文件的路径，具体如图1.2所示，点击菜单 项目-->属性-->VC++目录，然后在“包含目录”里添加开发库源代码文件的目录，在“库目录”项里添加开发库静态库文件的目录。

![](D:/OpenCorr/Release_GitHub/img/vs_directories.png)

图1.2 Visual Studio内设置开发库目录示例



（4）设置链接器附加依赖项，打开 项目-->属性-->链接器-->输入-->附加依赖项对话框，如图1.3所示。

![image](D:/OpenCorr/Release_GitHub/img/vs_dependencies.png)

图1.3 Visual Studio内设置附加依赖项示例



将静态库文件（.lib）的文件名列表输入文本框后点击确定。对于Release模式，可以直接复制下面的列表粘贴至文本框。注意在Debug模式下，opencv库相应的文件名（不包括扩展名）以字母d结尾。

libfftw3-3.lib
libfftw3f-3.lib
libfftw3l-3.lib
opencv_world4100.lib

（5）对于OpenCV和FFTW，还需要把动态链接库文件放到合适的目录下，可以放在编译代码产生的可执行程序（.exe文件）目录下，例如图1.1中的x64\Release。这些文件也可以放在操作系统Path参量包含的目录里（可通过Windows 10 设置-->关于-->高级系统设置-->高级-->环境变量-->系统变量-->Path 查看和修改）。

（6）如果希望使用多核CPU加速程序运行，还需要在项目属性中开启OpenMP支持，如图1.4所示。点击菜单 项目-->属性-->C/C++-->语言，将OpenMP支持设置为“是”。

![image](D:/OpenCorr/Release_GitHub/img/vs_openmp.png)

图1.4 Visual Studio内设置OpenMP支持示例



为了方便初学者配置开发环境，我们将一个完整的Visual Studio解决方案做成zip压缩包，放在[opencorr.org](https://opencorr.org/download/)供大家下载。将压缩包解开后（可使用7-Zip），直接用VS 2022或更高版本的Visual Studio打开OpenCorr.sln文件，即可开始调试代码。注意根据解压缩文件的目录正确设置IDE的路径，具体可参考上面的安装说明。

配置好开发环境，就可以尝试使用OpenCorr的模块制作DIC或DVC程序了。我们在GitHub项目的"[examples](https://github.com/vincentjzy/OpenCorr/tree/main/examples)"目录下提供了一些程序范例和图像，供使用者尝试。在编译这些程序前，同样需要正确设置IDE和代码内的文件路径。

## 1.2 macOS中的IDE配置

OpenCorr所需的四个开源程序库（Eigen, OpenCV, FFTW, and nanoflann）可以通过homebrew ([brew.sh](https://brew.sh)) 安装。这是macOS上一个流行的软件包管理器。为了使程序支持OpenMP，还需要安装LLVM（也可以通过homebrew方便地安装）。VS Code可以直接从微软的网站 ([code.visualstudio.com](https://code.visualstudio.com)) 下载安装，注意安装后还需要在VS Code内安装扩展包C/C++ Extension Pack。

假设OpenCorr的代码存放在这个目录下（/Users/(your username)/Workshop/OpenCorr），在VS Code内打开这个目录（Menu->File->Open Folder..）。然后在命令面板（Menu->View->Command Palette）中选择“任务：配置默认生成任务”和“任务：配置默认任务”，这样会在当前目录的子目录（.vscode）下生成两个JSON文件，分别是 c_cpp_properties.json 和 tasks.json。我们可以在VS Code中直接打开这两个脚本文件，输入命令行完成编译器、路径和库选项等配置，如图1.5所示。

![image](D:/OpenCorr/Release_GitHub/img/vs_code_macos_configuration.png)

图1.5 Visual Studio Code内配置编译参数



我们在GitHub项目的 "[examples/ide_configuration](https://github.com/vincentjzy/OpenCorr/tree/main/examples/ide_configuration)" 目录下提供了c_cpp_properties.json and tasks.json的示例。开发者可以直接下载并覆盖自己目录下的同名文件。注意如果你的CPU型号不是Apple Silicon，上述程序库的安装目录可能会有所不同，这可以通过homebrew的指令来确定，例如"brew --prefix Eigen"，然后替换路径即可。配置完成后，就可以在VS Code中打开一个OpenCorr的示例（见Section 1.1），通过运行生成任务（Menu->Terminal->Run Build Task）编译生成一个可执行文件。同样要注意示例代码内打开图像的文件路径是否正确。

对于直接使用CMake编译代码的用户，我们在GitHub项目的 "[examples/ide_configuration](https://github.com/vincentjzy/OpenCorr/tree/main/examples/ide_configuration)" 目录下提供了一个CMake脚本的示例 CMakeLists.txt。
