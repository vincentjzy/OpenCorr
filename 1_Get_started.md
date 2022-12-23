# 1. Get started

OpenCorr is developed and tested in Microsoft Visual Studio 2019 (VS, community version) on Windows 10. The codes, following the standard of ISO C++ 14, can be compiled using other C++ compiler like GCC, or on other OS like Linux/Unix. Developers working on Linux/Unix may refer to a sample of CMake script ([https://github.com/vincentjzy/OpenCorr/examples/CMakeLists.txt](https://github.com/vincentjzy/OpenCorr/examples/CMakeLists.txt)). To work with this library, users are supposed to have basic knowledge and skill about integrated development environment like VS. The building environment requires four open source libraries:

- Eigen 3.4.0 ([eigen.tuxfamily.org](http://eigen.tuxfamily.org)), used for basic operations of 2D matrix.
- OpenCV 4.6.0 ([opencv.org](http://opencv.org)), used to read images, extract and match the 2D image features.
- FFTW 3.3.5 ([fftw.org](http://fftw.org)), used to speed up the calculation of cross correlation.
- nanoflann (https://github.com/jlblancoc/nanoflann), used to search for the nearest neighbors of a point among a point cloud.

These libraries provide comprehensive instructions for installation. The main procedure can be summarized as the following steps:

1. Download the source files (e.g., .h or .cpp), static library files (.lib), and dynamic link library files (.dll) from the websites;

2. Place them into proper folders in your computer. For example, I created a project of name OpenCorr in VS. The source codes are stored in folder "D:\OpenCorr\OpenCorr\", the files of the four libraries mentioned above are placed in folder "D:\OpenCorr\", as shown in Figure 1.1;

   ![image](./img/vs_solution.png)

   *Figure 1.1. An example of folder structure*

3. Set the paths of source files and static library files in VS through Project --> Properties, as illustrated in Figure 1.2;
   ![image](./img/vs_directories.png)
   *Figure 1.2. Illustration of setting directories of libraries in Visual Studio 2019*

4. Set the additional dependencies in VS through Project --> Properties. Open the Additional Dependencies dialog, as shown in Figure 1.3;
   ![image](./img/vs_dependencies.png)

   *Figure 1.3. Illustration of setting additional dependencies in Visual Studio 2019*

Then, add the name list of static library files into the edit box. Beware that the file name ends with "d" in Debug mode for opencv lib.

>libfftw3-3.lib
>libfftw3f-3.lib
>libfftw3l-3.lib
>opencv_world460.lib

5. Place the dynamic link library files (.dll) into the folder where the executable programs are built (e.g. "D:\OpenCorr\x64\Release\" in Figure 1.1), or the directories listed in system Path;

6. Set OpenMP support to enable the acceleration on multi-core CPU, as shown in Figure 1.4.
   ![image](./img/vs_openmp.png)
   *Figure 1.4. Illustration of setting OpenMP support in Visual Studio 2019*

To facilitate the configuration for beginners, we made a compressed package of Visual Studio solution and share it on  [opencorr.org](https://opencorr.org/Download). Users may download and unzip it (e.g. using 7-Zip), then open OpenCorr.sln in VS 2019 or higher version of Visual Studio, and start programming.

There are a few examples in folder "examples" of GitHub repository along with images, which demonstrate how to make a DIC  or DVC program by assembling the modules in OpenCorr. Before building the executables, make sure that the file paths in the codes are correctly set. 
