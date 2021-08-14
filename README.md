![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/title_figure.png)
# OpenCorr
OpenCorr is an open source C++ library for development of 2D, 3D/stereo, and volumetric digital image correlation. It aims to provide a developer-friendly, lightweight, and efficient kit to the users who are willing to study the state-of-the-art DIC/DVC algorithms or to create DIC/DVC programs for their specific applications.

OpenCorr is under construction. More functions, including the GPU accelerated modules will be released soon. Thus, update of both codes and webpages will be frequent in the following months until we reach a stable version with relatively complete documentation.

Comments and suggestions are most welcome. You may reach us via
1. Email: zhenyujiang (at) scut.edu.cn;
2. Discussion here;
3. QQ group: 597895040

Users can also access the information of OpenCorr (in Chinese) via website [opencorr.org](http://opencorr.org) .

># Important updates
>2021.04.23, OpenCorr is released to public.
>
>2021.04.30, Modify structure of DIC module and stereovision module.
>
>2021.05.08, A brief instruction of framework is released.
>
>2021.05.17, Improve the adapbility for Linux and release a cool title figure.
>
>2021.06.12, Release an sample to demonstrate the calculation of strains, update the documentation.
>
>2021.08.14, Release the GPU accelerated module of ICGN algorithm and an example, instruction can be found in Section 3 (3. GPU accelerated modules).

# 1. Get started

OpenCorr is developed and tested in Microsoft Visual Studio 2019 (VS, community version) on Windows 10. The codes follow the standard of ISO C++ 14, theoretically it can be compiled on other OS like Linux. To use this library, the users are supposed to have basic knowledge and skill about integrated development environment like VS. The building environment requires three freeware libraries:

- Eigen 3.3.9 ([eigen.tuxfamily.org](http://eigen.tuxfamily.org)), used for basic operations of matrix.
- OpenCV 3.4.5 ([opencv.org](http://opencv.org)), used to read images, and inthe  modules related with image feature and stereovision.
- FFTW 3.3.9 ([fftw.org](http://fftw.org)), used for cross correlation.

These libraries provide excellent instructions for installation. The main procedure can be summarized as the following steps:

1. Download the source files (e.g. .h or .cpp), static library files (.lib), and dynamic link library files (.dll) from the websites;
2. Place them into proper directories in your computer. For example, I created a solution in VS with name of OpenCorr and a project with same name. The source codes are stored in folder "D:\OpenCorr\OpenCorr\", the files of source codes and static libraries of the three libraries are also placed in folder "D:\OpenCorr\", as shown in Figure 1.1;

![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/opencorr_folder_list.png)

*Figure 1.1. An example of directory structure*

3. Set the paths of source files and static library files in VS, as illustrated in Figure 1.2;
![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/vs_path_en.png)
*Figure 1.2. Illustration of setting paths in Visual Studio 2019*

4. Open the dialog of additional dependencies, as shown in Figure 1.3. 
![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/vs_lib_en.png)

*Figure 1.3. Illustration of setting additional dependencies in Visual Studio 2019*

Then add the name list of static library files into the editbox. Beware that the file names end with "d" in Debug mode.
>libfftw3-3.lib
>libfftw3f-3.lib
>libfftw3l-3.lib
>opencv_aruco345.lib
>opencv_bgsegm345.lib
>opencv_bioinspired345.lib
>opencv_calib3d345.lib
>opencv_ccalib345.lib
>opencv_core345.lib
>opencv_datasets345.lib
>opencv_dnn_objdetect345.lib
>opencv_dnn345.lib
>opencv_dpm345.lib
>opencv_face345.lib
>opencv_features2d345.lib
>opencv_flann345.lib
>opencv_fuzzy345.lib
>opencv_hfs345.lib
>opencv_highgui345.lib
>opencv_img_hash345.lib
>opencv_imgcodecs345.lib
>opencv_imgproc345.lib
>opencv_line_descriptor345.lib
>opencv_ml345.lib
>opencv_objdetect345.lib
>opencv_optflow345.lib
>opencv_phase_unwrapping345.lib
>opencv_photo345.lib
>opencv_plot345.lib
>opencv_reg345.lib
>opencv_rgbd345.lib
>opencv_saliency345.lib
>opencv_shape345.lib
>opencv_stereo345.lib
>opencv_stitching345.lib
>opencv_structured_light345.lib
>opencv_superres345.lib
>opencv_surface_matching345.lib
>opencv_text345.lib
>opencv_tracking345.lib
>opencv_video345.lib
>opencv_videoio345.lib
>opencv_videostab345.lib
>opencv_xfeatures2d345.lib
>opencv_ximgproc345.lib
>opencv_xobjdetect345.lib
>opencv_xphoto345.lib

5. Place the dynamic link library files (.dll) into the folder where the executable programs are built (e.g. "D:\OpenCorr\x64\Debug\", as shown in Figure 1.1), or the directories listed in system Path.

6. Set Open MP support to enable the acceleration on multi-core CPU, as shown in Figure 1.4.
![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/vs_openmp_en.png)
*Figure 1.4. Illustration of setting OpenMP support in Visual Studio 2019*

To facilitate the configuration for beginners, we made a zip package of the three libraries and share it on [pan.baidu.com](https://pan.baidu.com/s/17qdAhXJZPLWydYiowwEzig) (code: vyfy). Users may download it, unzip it and set the paths according to the instructions mentioned above.

There are a few examples in the folder "samples" along with images, which demonstrate how to make a DIC processing program using the modules in OpenCorr. Before building the executables, make sure that the file paths in the codes are correctly set. 

# 2. Framework
Figure 2.1 shows the framework of OpenCorr, which consists of four parts: (1) basic data objects; (2) DIC data objects; (3) basic processing methods; (4) DIC processing methods.

Figure 2.1 shows the framework of OpenCorr, which gives a guide to understand the structure of OpenCorr.
![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/framework.png)
*Figure 2.1. Framework of OpenCorr*

### Basic data objects:

(1) Point (oc_point.h and oc_point.cpp). Figure 2.2 shows the parameters and methods included in this object. The main parameter of point is its coordinate. It can also regarded as a vector, indicating the offset between two points. Thus, a function (vectorNorm()) is provided to calculate the length of the vector.  Moreover, operator "+" and "-" are overloaded to denote the superimposition of an offset on the coordinate of a point. Operator "*" and "/" for the coordinate of a point multiplied or divided by a scalar. Operator "<<" is also overloaded to out the coordinate of a point, in the form of  "x,y" (Point2D) or "x,y,z" (Point3D).

![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/oc_point.png)
*Figure 2.2. Parameters and methods included in Point object*

(2) Array/matrix (oc_array.h and oc_array.cpp). Figure 2.3 shows the parameters and methods included in this object. The object is simple, as the operations of matrix in this library call the functions of Eigen. It contains the functions of create and delete two dimension, three dimension and four dimension arrays, as well as specifically defined Eigen matrices.

![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/oc_array.png)
*Figure 2.3. Parameters and methods included in Array object*

(3) Image (oc_image.h and oc_image.cpp). Figure 2.4 shows the parameters and methods included in this object. In this object, OpenCV functions is called to read image file and get its dimension, as well as store the data into the Eigen matrices with same size. Caution: The users using OpenCV 4 may need to modify the codes of this object, as some basic functions in OpenCV 4 are different from the ones in OpenCV 3.

![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/oc_image.png)
*Figure 2.4. Parameters and methods included in Image object*

### DIC data objects:

(1) Subset (oc_subset.h and oc_subset.cpp). Figure 2.5 shows the parameters and methods included in this object. Subset can be regarded as a special matrix with its center located at a specific point. It height and width equals to two times of corresponding radius plus one. The constructor of this object sets the parameters mentioned above and initialize the Eigen matrix. Its menber function fill(Image2D* image) can be used to read grayscale data in specific region from an Image object and store the data into its Eigen matrix. Function zeroMeanNorm() performs the zero mean normalization of grayscale value at every point in a subset and return the normalization factor.

![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/oc_subset.png)
*Figure 2.5. Parameters and methods included in Subset object*

(2) Deformation (oc_deformation.h and oc_deformation.cpp). Figure 2.6 shows the parameters and methods included in this object. Deformation includes two dimensional one and three dimensional one, which can be described using the first order shape function and the second shape function. In 2D case, the 1st order shape function contains 6 elements (displacement u and v, as well as their gradients along x- and y-axis), its warp_matrix is a 3x3 matrix. The 2nd order shape function contains 12 elements (displacement u and v, as well as their first and second order gradients along x- and y-axis), its warp_matrix is a 6x6 matrix. Member functions include:

- setDeformation() without input, set the deformation elements according current warp_matrix;
- setDeformation() with input, set the deformation elements and update warp_matrix, according the input;
- setDeformation() with another Deformation object as input, set the deformation elements and update warp_matrix, according the input object;
- setWarp(), update warp_matrix according to the current deformation elements;
- Point2D warp(Point2D& point), Calculate the coordinates of a point after experiencing the deformation.

![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/oc_deformation.png)
*Figure 2.6. Parameters and methods included in Deformation object*

(3) POI (Point of interest, oc_poi.h and oc_poi.cpp). Figure 2.7 shows the parameters and methods included in this object. POI inherit the properties and methods from Point. In addition, it contains a deformation vector and a result vector. the former is used in DIC processsing, and the latter is used for output results. Its constructor set the coordinate, and clear the deformation vector and result vector meanwhile. Member functions include:

- clean(), set all the elements in deformation vector and result vector as zero;
- setIterationCriteria(float conv_criterion, float stop_condition, float neighbor_essential), set the convergence criterion and stop condition of iterative DIC methods and the minimum number required in the neighbor keypoints search when processing the POI.

![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/oc_poi.png)
*Figure 2.7. Parameters and methods included in POI object*

### Basic processing methods:

(1) Gradient (oc_gradient.h and oc_gradient.cpp). Figure 2.8 shows the parameters and methods included in this object. OpenCorr provides only one gradient calculation method, i.e. 4th-order central difference, which may be the most popular one. grad_img, points to the Image2D object to process, is initialized in constructor function. Member functions getGradientX() and getGradientY() are made to calculate the 1st-order gradient map along x and y direction, respectively. getGradientXY() calculates the mixed 2nd-order gradient map. The gradient maps are stored in Eigen matrices gradient_x, gradient_y, and gradient_xy, respectively.

![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/oc_gradient.png)
*Figure 2.8. Parameters and methods included in Gradient object*

(2) Interpolation (oc_interpolation.h and oc_interpolation.cpp). Figure 2.9 shows the parameters and methods included in this object. Interpolation is a base class which contains essential parameter interp_img pointing to the Image2D object to process. The derived class BicubicBspline (oc_bicubic_bspline.h and oc_bicubic_bspline.cpp) implemented the popular bicubic B-spline interpolation method. Our study indicates that the bicubic B-spline method show significantly improved accuracy and precision compared with bicubic method, at trial computational cost (Pan et al. Theo Appl Mech Lett, 2016, 6(3):126-130). Member function prepare() calculates the global lookup-table of interpolation coefficients of interp_image, while computer(Point2D& location) estimates the grayscale value at the input location according to the lookup-table.

![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/oc_interpolation.png)
*Figure 2.9. Parameters and methods included in Interpolation object*

(3) Feature (oc_feature.h and oc_feature.cpp). Figure 2.10 shows the parameters and methods included in this object. Feature is a base class which contains essential parameters ref_img and tar_img pointing to the Image2D objects (reference image and target image). Member function setImages(Image2D& ref_img, Image2D& tar_img) is used to update ref_img and tar_img. The derived class SIFT2D (oc_sift.h and oc_sift.cpp) provides the method to extract and match the SIFT features in the two images. Structure SIFT_config contains the main paramters in feature extraction. Users may refer to the relevent documents of OpenCV for their meanings. Parameter match_ratio is the threshold of ratio of the shortest distance betweeen the descriptors of reference feature and the target feature to the second shortest distance. The details of this parameter can be found in Lowe's famous paper (Lowe, Int J Comput Vis, 2004, 60(2):91-110). The extracted keypoints, after matching, are stored in vector ref_matched_kp and tar_matched_kp. The member functions include:

- prepare(), assign the address of matrices in ref_img and tar_img to OpenCV matrices ref_mat and tar_mat;
- compute(), extract the features in reference image and target image, and then match then;
- getSIFTconfig(), get current configuration of feature extraction;
- getMatchRatio(), get current threshold of feature matching;
- setExtraction(SIFTconfig SIFT_config), set configuration of feature extraction;
- setMatch(float match_ratio), set ratio threshold of feature matching.

![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/oc_feature.png)
*Figure 2.10. Parameters and methods included in Feature object*

(4) Calibration (oc_calibration.h and oc_calibration.cpp). Figure 2.11 shows the parameters and methods included in this object. The main parameters are

- Camera intrinsics, including fx, fy, fs, cx, cy, k1, k2, k3, k4, k5, k6, p1, p2;
- Camera extrinsics, including tx, ty, tz, pitch, roll, yaw;
- Convergence criterion and maximum iteration in iterative procedure of correction: convergence, iteration;
- Intrinsic matrix: intrinsic_matrix;
- Rotation matrix: rotation_matrix;
- Translation vector: translation_vector;
- Projection matrix: projection_matrix;

Member functions are

- updateIntrinsicMatrix(), update camera intrinsic matrix;
- updateRotationMatrix(), update rotation matrix according to rotation angles;
- updateTranslationVector(), update translation vector;
- updateProjectionMatrix(), update projection matrix according to the three matrices mentioned above;
- setCorrection(float convergence, int iteration), set parameters of iterative correction procedure;
- distort(Point2D& point), adjust the coordinate of input point according to the distortion model;
- correct(Point2D& point), correct the coordinate of input point through an iterative procedure.

![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/oc_calibration.png)
*Figure 2.11. Parameters and methods included in Calibration object*

(5) Stereovision (oc_stereovision.h and oc_stereovision.cpp). Figure 2.12 shows the parameters and methods included in this object. It helps to reconstruct the 3D coordinate of a point based on the two matched 2D points in left view and right view. The main parameters include:

- Intrinsics and extrinsics of left camera and right camera: left_cam 和 right_cam;
- Number of CPU threads in parallel processing: thread_number;
- 2D points in left view and right view: left_2d_pt 和 right_2d_pt;
- 3D point in space: space_pt.

Member functions include:

- updateCameraParameters(Calibration& left_cam, Calibration& right_cam), update parameters of cameras;
- setPointPair(Point2D& left_point, Point2D& right_point), set the matched point pair in left view and right view;
- prepare(), update the parameter matrices of the two cameras;
- reconstruct(Point2D& left_point, Point2D& right_point), reconstruct the coordinate of 3D point based on the 2D points in left view and right view.

![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/oc_stereovision.png)
*Figure 2.12. Parameters and methods included in Stereovision object*

### DIC processing methods:

The base class DIC (oc_dic.h and oc_dic.cpp) contains a few essential parameters:

- Pointer of reference image and target image: ref_img 和 tar_img;
- Subset radii (in x and y direction): subset_radius_x, subset_radius_y;
- Number of CPU threads in parallel processing: thread_number.

Member function setImages(Image2D& ref_img, Image2D& tar_img) is used to set the pointers of ref_img and tar_img, and setSubsetRadii(int subset_radius_x, int subset_radius_y) for setting subset radii. Three virtual member functions indicates the forms of main functions in this class: prepare() for preparation; compute(POI2D* POI) to process single POI, and compute(std::vector& POI_queue) to process a batch of POIs.

It is noteworthy that the methods in derive classes are designed for path-independent DIC, but they can also be used to realize the DIC methods with initial guess transfer mechanisms. For example, the popular reliability-guided DIC can be readily implemented by combining C++ vector and its sort functions with the DIC methods listed below.

(1) FFTCC2D (oc_fftcc.h and oc_fftcc.cpp), fast Fourier transform based cross correlation. Figure 2.13 shows the parameters and methods included in this object. The method calls FFTW library to perform FFT and inverse FFT calculation. Its principle can be found in our paper (Jiang et al. Opt Laser Eng, 2015, 65:93-102). An auxiliary class FFTW is made to facilitate parallel processing, as the procedure need allocate quite a lot of memory blocks dynamically. During the initialization of FFTCC2D, a few FFTW instances are created according to the set thread_number. Afterwards, they are called in compute(POI2D* POI) through getInstance(int tid). The POIs stored in container vector can be processed by compute(std::vector& POI_queue).

![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/oc_fftcc.png)
*Figure 2.13. Parameters and methods included in FFTCC2D object*

(2) FeatureAffine2D (oc_feature_affine.h and oc_feature_affine.cpp), feature aided affine estimation. Figure 2.14 shows the parameters and methods included in this object. The method estimate the affine matrix according to the keypoints around a POI in order to get the deformation at the POI. User may refer to our paper (Yang et al. Opt Laser Eng, 2020, 127:105964) for details of principle and implementation.

![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/oc_feature_affine.png)
*Figure 2.14. Parameters and methods included in FeatureAffine2D object*

(3) ICGN2D1 (ICGN algorithm with 1st-order shape function) and ICGN2D2 (ICGN algorithm with 2nd-order shape function), codes are stored in oc_icgn.h and oc_icgn.cpp. Figure 2.15 and Figure 2.16 show the parameters and methods included in the two objects. The principle and implementation of ICGN2D1 can be found in our paper (Jiang et al. Opt Laser Eng, 2015, 65:93-102). User may refer to the paper by Professor ZHANG Qingchuan's group (Gao et al. Opt Laser Eng, 2015, 65:73-80) for detailed information of ICGN2D2. Auxiliary classes ICGN2D1_ and ICGN2D2_ are made for parallel processing because the procedures also require a lot of dynamically allocated memory blocks. The implementation and usage are similar to the one in FFTCC2D.

![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/oc_icgn1.png)
*Figure 2.15. Parameters and methods included in ICGN2D1 object*

![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/oc_icgn2.png)
*Figure 2.16. Parameters and methods included in ICGN2D2 object*

(4) Strain2D (oc_strain.h and oc_strain.cpp), calculation of strains based on the DIC results. Figure 2.17 shows the parameters and methods included in this objects. The method first create local profiles of displacement components in POI-centered subregion through polynomial fitting, and then calculate the strains according to the first order derivatives of the profile. User may refer to the paper by Professor PAN Bing (Pan et al. Opt Eng, 2007, 46:033601) for the details of the principle. The main parameters include:

- Topleft corner of ROI: topleft_point
- Radius of subregion: subregion_radius
- Space between adjacent POIs: grid_space
- 2D matrices of displacement components: u_map, v_map

Main member functions include:

- setSubregionRadius(int subregion_radius), set the radius of the POI-centerd subregion for fitting of local displacement profiles
- setGridSpace(int grid_space), set space between the adjacent POIs, which should be identical to the value set in DIC computation
- setDisplacement(std::vector& POI_queue), create u_map and v_map according to the result of POI
- compute(POI2D* POI), calculate the strains at the POI

![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/oc_strain.png)
*Figure 2.17. Parameters and methods included in Strain2D object*

# 3. GPU accelerated modules

GPU accelerated modules currently include a module of ICGN algorithm with shape functions of the 1st and the 2nd order. Developers can use them through calling dynamic link libraries.
Requirements:
- Hardware: NVIDIA graphics card with memory greater than 2GB
- Software: Windows OS, CUDA ToolKit 11.4

GPU accelerated ICGN module consists of three files: head (opencorr_gpu.h), static link library (OpenCorrGPULib.lib), and dynamic link library (OpenCorrGPULib.dll). The configuration of IDE is similar to the one of FFTW, which can be summarized as four steps:

1. Put opencorr_gpu.h into the folder where opencorr.h is, as shown in Figure 3.1;
![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/gpu_head_en.png)
*Figure 3.1. Opencorr_gpu.h is placed in the same folder of opencorr.h*

2. Set the path of static library file in VS (for example, OpenCorrGPULib.lib is in folder opencorrGPU\lib in Figure 1.1), as illustrated in Figure 3.2;
![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/vs_gpu_path_en.png)
*Figure 3.2. Setting path of library in Visual Studio 2019*

3. Add OpenCorrGPULib.lib as the additional dependencies, as illustrated in Figure 3.3;
![image](https://github.com/vincentjzy/OpenCorr/blob/main/img/vs_gpu_lib_en.png)
*Figure 3.3. Setting additional dependencies of library in Visual Studio 2019*

4. Let OpenCorrGPULib.dll be in the same folder of the built executable program (.exe).

test_2d_dic_icgn_gpu.cpp in folder /samples demonstrates how to use the module. The data structures in this module are a little different from those in OpenCorr (CPU version) to achieve high computation efficiency. Thus, the reference image and the target image need to be converted to 1D arrays. An additional POI queue is generated and assigned with the location and initial guess for each POI. After the processing, the obtained results need to be transferred back to the original POI queue for the convenience of output by calling the OpenCorr functions.

# Acknowledgements

OpenCorr demonstrates our exploration of DIC/DVC methods in recent years, which got financial support from National Natural Science Foundation of China. I would like to give my special thanks to two collaborators for their continuous and strong supports: Professor QIAN Kemao at Nanyang Technological University and Professor DONG Shoubin at South China University of Technology.

# Developers

- Dr JIANG Zhenyu, Professor, South China University of Technology
- Mr ZHANG Lingqi, PhD candidate, Tokyo Institute of Technology
- Dr WANG Tianyi, PostDoc, BrookHaven Natinoal Lab
- Dr CHEN Wei, Research engineer, Midea
- Mr HUANG Jianwen, Software engineer, SenseTime
- Mr YANG Junrong, , Software engineer, Tencent
- Mr LIN Aoyu, Master student, South China University of Technology

# Related publication

Users may refer to our papers for the details of the principle of the algorithms provided in OpenCorr.

[1] Z. Jiang, Q. Kemao, H. Miao, J. Yang, L. Tang, Path-independent digital image correlation with high accuracy, speed and robustness, Optics and Lasers in Engineering, 65 (2015) 93-102.

[2] L. Zhang, T. Wang, Z. Jiang, Q. Kemao, Y. Liu, Z. Liu, L. Tang, S. Dong, High accuracy digital image correlation powered by GPU-based parallel computing, Optics and Lasers in Engineering, 69 (2015) 7-12.

[3] T. Wang, Z. Jiang, Q. Kemao, F. Lin, S.H. Soon, GPU accelerated digital volume correlation, Experimental Mechanics, 56 (2016) 297-309.

[4]. Z. Pan, W. Chen, Z. Jiang, L. Tang, Y. Liu, Z. Liu, Performance of global look-up table strategy in digital image correlation with cubic B-spline interpolation and bicubic interpolation, Theoretical and Applied Mechanics Letters, 6 (3) 126-130.

[5] W. Chen, Z. Jiang, L. Tang, Y. Liu, Z. Liu, Equal noise resistance of two mainstream iterative sub-pixel registration algorithms in digital image correlation, Experimental Mechanics, 57 (2017) 979-996.

[6] J. Huang, L. Zhang, Z. Jiang, S. Dong, W. Chen, Y. Liu, Z. Liu, L. Zhou, L. Tang, Heterogeneous parallel computing accelerated iterative subpixel digital image correlation, Science China Technological Sciences, 61 (2018) 74-85.

[7] J. Yang, J. Huang, Z. Jiang, S. Dong, L. Tang, Y. Liu, Z. Liu, L. Zhou, SIFT-aided path-independent digital image correlation accelerated by parallel computing, Optics and Lasers in Engineering, 127 (2020) 105964.

[8] J. Yang, J. Huang, Z. Jiang, S. Dong, L. Tang, Y. Liu, Z. Liu, L. Zhou, 3D SIFT aided path independent digital volume correlation and its GPU acceleration, Optics and Lasers in Engineering, 136 (2021) 106323.

[9] A. Lin, R. Li, Z. Jiang, S. Dong, Y. Liu, Z. Liu, L. Zhou, L. Tang, Path independent stereo digital image correlation with high speed and analysis resolution, Under review.
