# OpenCorr
OpenCorr is an open source C++ library for development of 2D, 3D/stereo, and volumetric digital image correlation. It aims to provide a developer-friendly, lightweight, and efficient kit to the users who are willing to study the state-of-the-art DIC/DVC algorithms or to create DIC/DVC programs for their specific applications.

OpenCorr is under construction. More functions, including the GPU accelerated modules will be released soon. Thus, update of both codes and webpages will be frequent in the following months until we reach a stable version with relatively complete documentation.

Comments and suggestions are most welcome. You may reach us via
1. Email: jiangzhenyu (at) opencorr.com;
2. Discussion here (at GitHub);
3. QQ group: 597895040

># Important updates
>2021.04.30, 

# Get started

OpenCorr is developed and tested in Microsoft Visual Studio 2019 (VS, community version) on Windows 10. The codes follow the standard of ISO C++ 14, theoretically it can be compiled on other OS like Linux. To use this library, the users are supposed to have basic knowledge and skill about integrated development environment like VS. The building environment requires three freeware libraries:

- Eigen 3.3.9 ([eigen.tuxfamily.org](eigen.tuxfamily.org)), used for basic operations of matrix.
- OpenCV 3.4.5 ([opencv.org](opencv.org)), used to read images, and inthe  modules related with image feature and stereovision.
- FFTW 3.3.9 ([fftw.org](fftw.org)), used for cross correlation.

These libraries provide good instructions for installation. The main procedure can be summarized as the following steps:

1. Download the source files (e.g. .h or .cpp), static library files (.lib), and dynamic link library files (.dll) from the websites;
2. Place them into proper directories in your computer. For example, I created a solution in VS with name of OpenCorr and a project with same name. The source codes are stored in folder "D:\OpenCorr\OpenCorr\", the files of source codes and static libraries of the three libraries are also placed in folder "D\OpenCorr\", as shown in Figure 1;

![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/opencorr_folder_list.png)
*Figure 1. An example of directory structure*

3. Set the paths of source files and static library files in VS, as illustrated in Figure 2;
![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/configuration_path_vs.png)
*Figure 2. Illustration of setting paths in Visual Studio 2019*

4. Place the dynamic link library files (.dll) into the folder where the executable programs are built (e.g. "D\OpenCorx64\Debug\"), or the directories listed in system Path.

To facilitate the configuration for beginners, we made a zip package of the three libraries and share it on [pan.baidu.com](https://pan.baidu.com/s/17qdAhXJZPLWydYiowwEzig) (code: vyfy). Users may download it, unzip it and set the paths according to the instructions mentioned above.

There are some examples in folder "samples", which demonstrate how we use the modules in OpenCorr to build our own DIC programs. The DIC algorithms provided in this library include:

- Fast Fourier transform-based cross correlation (FFT-CC): Estimation of integer-pixel displacement.
- SIFT-aided DIC: Estimation of 1st order deformation
- Inverse compositional Gaussian-Newton (ICGN) algorithm: high accuracy estimation of deformation. Two versions are implemented, one for the 1st order shape function and the other for the 2nd order shape function.

Figure 3 shows the framework of OpenCorr, which gives a guide to understand the structure of OpenCorr.
![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/framework.png)
*Figure 3. Framework of OpenCorr*

# References
Users may refer to our papers for the details of the principle of the algorithms provided in OpenCorr.

[1] Z. Jiang, Q. Kemao, H. Miao, J. Yang, L. Tang, Path-independent digital image correlation with high accuracy, speed and robustness, Optics and Lasers in Engineering, 65 (2015) 93-102.

[2] L. Zhang, T. Wang, Z. Jiang, Q. Kemao, Y. Liu, Z. Liu, L. Tang, S. Dong, High accuracy digital image correlation powered by GPU-based parallel computing, Optics and Lasers in Engineering, 69 (2015) 7-12.

[3] T. Wang, Z. Jiang, Q. Kemao, F. Lin, S.H. Soon, GPU accelerated digital volume correlation, Experimental Mechanics, 56 (2016) 297-309.

[4] W. Chen, Z. Jiang, L. Tang, Y. Liu, Z. Liu, Equal noise resistance of two mainstream iterative sub-pixel registration algorithms in digital image correlation, Experimental Mechanics, 57 (2017) 979-996.

[5] J. Huang, L. Zhang, Z. Jiang, S. Dong, W. Chen, Y. Liu, Z. Liu, L. Zhou, L. Tang, Heterogeneous parallel computing accelerated iterative subpixel digital image correlation, Science China Technological Sciences, 61 (2018) 74-85.

[6] J. Yang, J. Huang, Z. Jiang, S. Dong, L. Tang, Y. Liu, Z. Liu, L. Zhou, SIFT-aided path-independent digital image correlation accelerated by parallel computing, Optics and Lasers in Engineering, 127 (2020) 105964.

[7] J. Yang, J. Huang, Z. Jiang, S. Dong, L. Tang, Y. Liu, Z. Liu, L. Zhou, 3D SIFT aided path independent digital volume correlation and its GPU acceleration, Optics and Lasers in Engineering, 136 (2021) 106323.
