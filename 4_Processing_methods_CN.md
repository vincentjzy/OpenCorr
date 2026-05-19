# 4. 处理方法

## 4.1. 基本处理

（1）Gradient（计算梯度），代码保存在oc_gradient.h和oc_gradient.cpp文件中。图4.1.1展示了Gradient对象的参数和方法。本程序开发库目前只提供常用的一阶微商的四阶中心差分方法。对二维图像，grad_img为指向待处理Image2D对象的指针，计算得到的梯度图保存在对应的Eigen矩阵gradient_x、gradient_y和gradient_xy里。对三维图像，grad_img为指向待处理Image3D对象的指针，计算得到的梯度图保存在对应的三维浮点数矩阵gradient_x、gradient_y和gradient_z里。

成员函数：

- getGradientX()，getGradientY()，getGradientZ()，计算x、y、z方向的一阶梯度图；
- getGradientXY()，计算二阶混合梯度图。

![image](D:/OpenCorr/Release_GitHub/img/oc_gradient.png)

图4.1.1 Gradient对象的参数和方法



（2）Interpolation（计算插值），代码保存在oc_interpolation.h文件中。图4.1.2展示了Interpolation对象的参数和方法。Interpolation是一个基类，包含了基本的参数，即指向待处理Image对象的指针*interp_img。派生类BicubicBspline和TricubicBspline（代码保存在oc_cubic_bspline.h和oc_cubic_bspline.cpp文件中）提供常用的双三次和三三次B样条插值方法。我们的研究表明，双三次B样条插值方法的精度明显优于双三次插值，但计算代价只有轻微的增加（Pan et al. Theo Appl Mech Lett, 2016, 6(3): 126-130）。三三次B样条插值的具体信息可参见我们的论文（Yang et al. Opt Laser Eng, 2021, 136: 106323）。

成员函数：

- prepare()，计算图像的全局插值系数矩阵，结果保存在矩阵interp_coefficient中；
- compute(Point2D& location)和compute(Point3D& location)，根据插值系数计算输入点坐标处的灰度值。

![image](D:/OpenCorr/Release_GitHub/img/oc_interpolation.png)

图4.1.2 Interpolation对象的参数和方法



（3）NearestNeighbor（近邻估计），代码保存在(oc_nearest_neighbor.h和oc_nearest_neighbor.cpp文件中。图4.1.3展示NearestNeighbor对象的参数和方法。该类调用了nanoflann(https://github.com/jlblancoc/nanoflann)估计三维点云中指定坐标的近邻点。快速近邻估计库（FLANN，Fast Library for Approximate Nearest Neighbors）提供的方法相比暴力搜索（brute force search）而言，效率上会有显著提升。nanoflann支持两种搜索模式：（i）搜索指定半径圆形区域内的近邻点或（ii）搜索最近邻的K个点。注意近邻估计偶尔无法准确估计所有符合条件的近邻点，因此发现获得的近邻点异常少时，可改用简单的暴力搜索再试一次。

成员函数：

- assignPoints(vector& point_queue)，根据赋予的Point2D或Point3D队列生成三维点云；
- constructKdTree()，构造用于快速搜索的KD树结构；
- radiusSearch(Point3D query_point)，估计三维点云中给定三维坐标周围指定半径圆形范围内的近邻点，返回近邻点的数目；
- knnSearch(Point3D query_point)，估计三维点云中给定三维坐标周围最近邻的K个点，返回近邻点的数目；
- getSearchRadius()，获取当前搜索半径；
- getSearchK()，获取当前搜索K值；
- setSearchRadius(float search_radius)，设定指定半径搜索的半径；
- setSearchK(int search_k)，设定K近邻搜索的K值。

![image](D:/OpenCorr/Release_GitHub/img/oc_nearest_neighbor.png)

图4.1.3 NearestNeighbor对象的参数和方法



（4）Feature（图象特征提取和匹配），代码保存在oc_feature.h文件中。图4.1.4展示了Feature对象的参数和方法。Feature2D和Feature3D是基类，包含了基本的参数，即指向参考图和目标图Image对象的指针（ref_img和tar_img）。派生类SIFT2D和SIFT3D（代码保存在oc_sift.h和oc_sift.cpp文件中）提供二维和三维的SIFT特征提取和匹配方法。sift_config结构包含了特征提取的主要参数，各参数的具体含义可参考OpenCV的相关文档，matching_ratio参数为特征匹配时两图中特征描述符最近距离和次近距离之比的阈值。提取的关键点经过匹配后分别存储在Point队列ref_matched_kp和tar_matched_kp中。

SIFT2D调用OpenCV中的SIFT类，其原理及参数说明可参见Lowe的论文（Lowe, Int J Comput Vis, 2004, 60(2): 91-110）。SIFT3D参考了Rister等人的论文（Rister et al, IEEE Trans Image Process, 2017, 26(10): 4900-4910），具体实现方法亦可参考我们的论文（Yang et al, Opt Lasers Eng, 2021, 136: 106323）。

主要成员函数包括：

- setImages(ref_img, tar_img)，设定ref_img和tar_img；
- prepare()，在SIFT2D中，将ref_img和tar_img地址赋予OpenCV矩阵ref_mat和tar_mat，在SIFT3D中，初始化用于建立特征描述符的正二十面体表面正三角片参数；
- compute()，提取参考图和目标图中的特征并进行匹配；
- getSiftConfig()，获取当前特征提取参数；
- getMatchingRatio()，获取当前特征匹配阈值；
- setExtraction(SIFTconfig sift_config)，设定特征提取参数；
- setMatching(float matching_ratio)，设定特征匹配阈值。

![image](D:/OpenCorr/Release_GitHub/img/oc_feature.png)

图4.1.4 Feature对象的参数和方法



（5）Calibration（相机标定），代码保存在oc_calibration.h和oc_calibration.cpp文件中。图4.1.5展示了Calibration对象的参数和方法。其主要功能是根据相机标定参数矫正传感器坐标系中点的像素坐标。

主要参数：

- 相机内参 intrinsics，内容为fx, fy, fs, cx, cy, k1, k2, k3, k4, k5, k6, p1, p2；
- 相机外参 extrinsics，内容为tx, ty, tz, rx, ry, rz；
- 内参矩阵 intrinsic_matrix；
- 旋转矩阵 rotation_matrix；
- 平移向量 translation_vector；
- 投影矩阵 projection_matrix；
- 建立图像畸变矫正映射图的收敛判据和迭代次数上限 convergence, iteration；
- 传感器坐标系中整像素位置对应的含畸变的图像坐标系中x和y坐标 map_x, map_y。

成员函数：

- updateIntrinsicMatrix()，更新相机内参矩阵；
- updateRotationMatrix()，更新旋转矩阵；
- updateTranslationVector()，更新平移矢量；
- updateProjectionMatrix()，根据上述三个矩阵更新投影矩阵；
- updateMatrices()，更新上述四个矩阵；
- clear(), 将内参和外参数组中所有元素设为零；
- Point2D image_to_sensor(Point2D& point), 将输入点的坐标从图像坐标系（物理单位）转换至传感器坐标（像素单位）;
- Point2D sensor_to_image(Point2D& point), 将输入点的坐标从传感器坐标（像素单位）转换至图像坐标系（物理单位）;
- float getConvergence()，获取当前的迭代收敛判据；
- int getIteration()，获取当前的迭代次数上限；
- setUndistortion(float convergence, int iteration)，设定建立畸变矫正映射图的迭代参数；
- prepare(int height, int width)，根据图像尺寸生成传感器坐标系中整像素位置对应的含畸变的物理坐标映射图；
- distort(Point2D& point)，根据畸变参数对输入点的物理坐标进行调节；
- undistort(Point2D& point)，根据畸变矫正映射图矫正输入点的像素坐标。

![image](D:/OpenCorr/Release_GitHub/img/oc_calibration.png)

图4.1.5 Calibration对象的参数和方法



（6）Stereovision（立体视觉），代码保存在oc_stereovision.h和oc_stereovision.cpp文件中。图4.1.6展示了Stereovision对象的参数和方法。其主要功能是根据左右视场中匹配的二维点坐标重构空间中的三维点坐标。

主要参数：

- Calibration view1_cam 和 view2_cam，相机对象，其中view1_cam为主相机，view2_cam为次相机；
- int thread_number，调用OpenMP进行并行处理的线程数；
- Eigen::Matrix3f fundamental_matrix，双目立体视觉模型中的基础矩阵。

成员函数：

- updateCameraParameters(Calibration* view1_cam, Calibration* view2_cam)，更新相机对象；
- void updateFundementalMatrix()，更新基础矩阵；
- prepare()，更新各相机中的参数矩阵以及基础矩阵；
- Point3D reconstruct(Point2D& view1_2d_point, Point2D& view2_2d_point)，根据主次视场匹配的点坐标重构对应的空间点坐标；
- reconstruct(vector<Point2D>& view1_2d_point_queue,vector<Point2D>& view2_2d_point_queue, vector<Point3D>& space_3d_point_queue)，处理一批点，结果保存在队列space_3d_point_queue中。

![image](D:/OpenCorr/Release_GitHub/img/oc_stereovision.png)

图4.1.6 Stereovision对象的参数和方法



（7）IO（文件输入和输出），代码保存在oc_io.h和oc_io.cpp文件中，图4.1.7展示了IO对象的参数和方法。该模块主要用于辅助程序测试，其主要功能是从文本格式的CSV数据表中读取POI的坐标和计算结果，或者将计算结果输出至文本格式的CSV数据表。

主要参数包括：

- 文件路径和文本分隔符：file_path，delimiter；
- 图像尺寸：2D（height，width），3D（dim_x，dim_y，dim_z）；

主要成员函数包括：

- setPath(string file_path)，设定CSV数据表路径；
- setDelimiter(string delimiter)，设定数据分隔符；
- loadPoint2D(string file_path)或loadPoint3D(string file_path)，从输入路径的CSV数据表中读取预设的POI坐标信息，生成Point2D或Point3D队列；
- savePoint2D(vector<Point2D>& point_queue, string file_path)或savePoint3D(vector<Point3D>& point_queue, string file_path)，将POI的坐标存入CSV数据表；
- loadCalibration(Calibration& calibration_cam1, Calibration& calibration_cam2, string file_path)，读入相机标定参数；
- loadTable2D()，loadTable2DS()或loadTable3D()，从CSV数据表中读取计算结果，生成POI2D，POI2DS或POI3D队列；
- saveTable2D(vector<POI2D>& POI_queue)，saveTable2DS(vector<POI2DS>& poi_queue)或saveTable3D(vector<POI3D>& poi_queue)，将POI队列的计算结果存入CSV数据表；
- saveDeformationTable2D(vector<POI2D>& POI_queue)，将POI2D队列中各POI的完整变形矢量信息存入数据表；
- saveMap2D(vector<POI2D>& POI_queue, OutputVariable variable)，saveMap2DS(vector<POI2DS>& poi_queue, OutputVariable variable)或saveMap3D(vector<POI3D>& POI_queue, OutputVariable variable)，将POI队列中各POI的指定variable信息（2D DIC, 3D/stereo DIC 或DVC结果）依照其坐标保存为二维或三维矩阵，variable参量可参考enum OutputVariable；
- saveMatrixBin(vector<POI3D>& poi_queue), 将POI队列的计算结果存入二进制文件，二进制文件的文件头为四个整数 (数组长度和图像的x，y及z方向尺寸)，紧接着是一个单精度浮点数一维矩阵（按先x，再y，然后z的顺序排列），数据顺序为x、y、z坐标，u、v、w位移、然后是ZNCC值和收敛时变形矢量增量的模;
- vector<POI3D> loadMatrixBin(), 从二进制数据文件中读取计算结果，保存为POI队列。二进制文件的文件头中图像的x，y及z方向尺寸分别存入IO对象的dim_x，dim_y和dim_z中。数据的排列规则与saveMatrixBin函数相同。

![image](D:/OpenCorr/Release_GitHub/img/oc_io.png)

图4.1.7 IO对象的参数和方法



## 4.2. DIC/DVC处理

图4.2.1展示了DIC基类的主要参数和方法，代码保存在oc_dic.h和oc_dic.cpp文件中。

主要参数：

- 指向参考图和目标图的指针，ref_img 和 tar_img；
- 子区半径（x，y和z方向），subset_radius_x, subset_radius_y, subset_radius_z；
- 调用OpenMP进行并行处理的线程数，thread_number；
- 自适应子区功能开关：self_adaptive。

成员函数：

- setImages(Image& ref_img, Image& tar_img)，设定参考图和目标图指针；
- setSubset(int subset_radius_x, int subset_radius_y)或setSubset(int subset_radius_x, int subset_radius_y, int subset_radius_z)，设定子区半径。
- setSelfAdaptive(bool is_self_adpative)，开启或关闭自适应子区功能。

其余三个虚函数展示了该类中的主要方法，即

- prepare()，准备工作；
- compute(POI2D* poi)或compute(POI3D* poi)，处理单个POI；
- compute(std::vector& poi_queue)，处理一批POI。

该基类下有数个派生类，实现了各种DIC方法。这些方法虽然针对path-independent DIC的策略开发，但是很容易使用它们实现初值传递的策略，例如目前较为流行的reliability-guided DIC方法，可以简捷地运用C++中的vector容器及其排序方法，结合下面提供的DIC方法加以实现。

![image](D:/OpenCorr/Release_GitHub/img/oc_dic.png)

图4.2.1 DIC基类的参数和方法



（1）FFTCC（快速傅里叶变换加速的互相关方法），代码保存在oc_fftcc.h和oc_fftcc.cpp中。图4.2.2展示了FFTCC对象的参数和方法。该方法调用FFTW库完成快速傅里叶变换及逆变换运算，其原理可参见我们的论文（Jiang et al. Opt Laser Eng, 2015, 65:93-102; Wang et al. Exp Mech, 2016, 56(2): 297-309）。由于处理过程中需要动态申请不少内存块，因而制作了一个FFTW类辅助并行计算。在FFTCC2D和FFTCC3D类初始化时，根据设定的线程数初始化相应数目的FFTW实例池，然后在compute(poi)中通过getInstance(int tid)根据当前的CPU线程号获取一个FFTW实例进行运算。对于装入vector容器的一系列POI，可调用compute(std::vector& poi_queue)进行批处理。FFTCC亦可用于计算自相关值，从而估计子区或全图内的散斑平均尺寸。

![image](D:/OpenCorr/Release_GitHub/img/oc_fftcc.png)

图4.2.2 FFTCC对象的参数和方法



（2）FeatureAffine（图像特征辅助的仿射变换方法），代码保存在oc_feature_affine.h和oc_feature_affine.cpp中。图4.2.3展示了FeatureAffine2D对象的参数和方法。该方法利用POI邻近的图像特征点拟合仿射变换矩阵，以此估计POI的变形，其原理可参见我们的论文（Yang et al. Opt Laser Eng, 2020, 127: 105964; Yang et al, Opt Lasers Eng, 2021, 136: 106323）。FeatureAffine使用NearestNeighbor加速POI附近特征点的搜索，因此在初始化时，根据设定的CPU线程数生成NearestNeighbor实例池，然后在compute(poi)中通过getInstance(int tid)根据当前的CPU线程号获取一个NearestNeighbor实例进行运算。

注意compute(poi)先调用NearestNeighbor中的指定半径范围搜索，若搜索到的特征点低于预设数量下限，则使用K近邻模式搜索。对于极少数POI周围特征点过少的情况，会采用brute force search模式距离最近的特征点，直至数目达到预设的数量下限。

![image](D:/OpenCorr/Release_GitHub/img/oc_feature_affine.png)

图4.2.3 FeatureAffine对象的参数和方法



（3）ICGN（Inverse compositional Gauss-Newton方法），代码保存在oc_icgn.h和oc_icgn.cpp中。图4.2.4展示了ICGN对象的参数和方法。ICGN2D1和ICGN3D1方法的原理可参见我们的论文（Jiang et al. Opt Laser Eng, 2015, 65: 93-102; Wang et al. Exp Mech, 2016, 56(2): 297-309），ICGN2D2方法的原理可参见张青川教授团队的论文（Gao et al. Opt Laser Eng, 2015, 65: 73-80）。由于ICGN方法处理过程中也需要动态申请大量内存块，因而制作了相应的辅助类 （ICGN2D1_ 、 ICGN2D2_ 和ICGN3D1_） 协助多线程并行计算。与FFTCC模块类似，在类初始化时，根据设定的线程数初始化相应数目的辅助类实例池，然后在compute(poi)中通过getInstance(int tid)根据当前的CPU线程号获取一个辅助类实例进行运算。对于装入vector容器的一系列POI，可调用compute(std::vector& poi_queue)进行批处理。

![image](D:/OpenCorr/Release_GitHub/img/oc_icgn.png)

图4.2.4 ICGN对象的参数和方法

(4) NR（Newton-Raphson方法），代码保存在oc_nr.h和oc_nr.cpp中。图4.2.5展示了NR对象的参数和方法。NR 曾是1990年代标志性的迭代DIC算法，但这一经典算法逐渐被ICGN取代，因为后者的精度相当而效率更高。因此，我们只制作了NR2D1 供对早期DIC算法感兴趣的朋友参考。关于该算法的原理可参见Hugh Bruck教授当年广为流传的论文（Bruck et al. Exp Mech, 1989, 29(3): 261-267）. NR和ICGN之间更细致的对比亦可参考我们的论文 （Chen et al. Exp Mech, 2017, 57(6): 979-996）。

![image](D:/OpenCorr/Release_GitHub/img/oc_nr.png)
图4.2.5 NR对象的参数和方法

（5）EpipolarSearch（极线约束辅助的搜索），代码保存在oc_epipolar_search.h和oc_epipolar_search.cpp文件中。图4.2.5展示了EpipolarSearch对象的参数和方法。该方法针对立体视觉所需的匹配，利用两个视角图像之间存在的极线约束，将搜索一个视图中的点在另一个视图中的对应点的范围缩小至极线的一部分。搜索范围的中心落在极线与其一条垂线的交点，该垂线同时通过一个根据初始位移和猜测的视差估计的点，其原理可参见我们的论文（Lin et al. Opt Laser Eng, 2022, 149: 106812）。搜索的步长限制在几个像素（低于ICGN算法的收敛半径），视差可设定为一个定值或视场内像素坐标的双线性函数。该方法中调用了ICGN2D1（收敛判据较为宽松，迭代次数较少）以确保搜索中匹配的精度，保留ZNCC值最高的结果。该结果可作为变形初值输送给ICGN2D2方法，进行高精度的匹配。 GitHub项目的examples 目录里的test_3d_reconstruction_epipolar.cpp展示了如何使用这一模块进行立体匹配，重构目标物表面点云坐标。另一个例子（test_3d_reconstruction_epipolar_sift.cpp）则展示了如何结合EpipolarSearch和SIFT特征引导的FeatureAffine来显著提高匹配效率。

主要参数包括：

- 相机对象: view1_cam (principal) 和 view2_cam (secondary);
- 步进搜索中参数, search_radius, search_step;
- 估算极线的基础矩阵，fundamental_matrix；
- 调用ICGN2D1的参数: icgn_rx, icgn_ry, icgn_conv, icgn_stop；
- 估算两个视场视差的参数：float parallax_x[3], parallax_y[3]，假设视差相对点坐标符合双线性分布，若假设全场视差恒定，可直接设定Point2D parallax。

![image](D:/OpenCorr/Release_GitHub/img/oc_epipolar_search.png)

图4.2.6 EpipolarSearch对象的参数和方法

(6) ICLM （Inverse compositional Levenberg-Marquardt方法），代码保存在oc_iclm.h和oc_iclm.cpp中。图4.2.7展示了ICGN对象的参数和方法。ICLM方法的原理可参见陈斌博士的论文 (Chen & Jungstedt, Opt Laser Eng, 2022, 151: 106930)，它的实现与ICGN非常相似，仅在求Hessian矩阵的逆时引入阻尼项。我们的测试表明，ICLM的性能（尤其在稳健性和计算效率方面）似乎弱于ICGN。

![image](D:/OpenCorr/Release_GitHub/img/oc_iclm.png)
图4.2.7 ICLM对象的参数和方法



图4.2.8展示了Strain（计算应变）对象的参数和方法，代码保存在oc_strain.h和oc_strain.cpp文件中。该方法先用多项式拟合以考察POI为中心的子区域（subregion）内的位移分量曲面，然后根据其一阶微商计算应变。原理可参考潘兵教授的论文（Pan et al. Opt Eng, 2007, 46: 033601）。具体实现中调用了NearestNeighbor模块加速POI周围的近邻POI搜索，其策略与FeatureAffine相似。注意计算应变时，缺省使用Cauchy应变的定义，用户可通过设定approximation参数，切换为Green应变的定义。

主要参数：

- 拟合局部位移分量曲面的子区域半径：subregion_radius；
- 考察POI附近参与位移场拟合的POI最小数目：neighbor_number_min；
- 参与位移场拟合POI的ZNCC下限阈值：zncc_threshold；
- 应变张量描述方式：description，1表示Lagrangian，2表示Eulaerian；
- 应变的近似定义：approximation，1表示Cauchy应变，2表示Green应变；
- OpenMP并行处理的线程数：thread_number。

成员函数：

- setSubregionRadius(int subregion_radius)，设置拟合局部位移分量曲面的子区域半径；
- setNeighborMin(int neighbor_number_min)，设置参与拟合曲面的最少邻近POI数目；
- setZnccthreshold(float zncc_threshold)，设置参与位移场拟合POI的ZNCC下限阈值；
- setDescription(int description)，设置应变张量描述方式：1表示Lagrangian，2表示Eulaerian；
- setApproximation(int approximation)，设置应变近似定义：1表示Cauchy应变，2表示Green应变；
- prepare(vector& poi_queue)，根据POI队列生成近邻搜索需要的KD树，vector的元素在2D DIC中为POI2D，在3D DIC中为POI2DS，在DVC中为POI3D；
- compute(*poi, vector& poi_queue)，计算POI的应变，2D DIC中POI类型为POI2D，3D DIC中POI类型为POI2DS，DVC中POI类型为POI3D；
- compute(std::vector& poi_queue)，处理一个POI队列。

![image](D:/OpenCorr/Release_GitHub/img/oc_strain.png)

图4.2.8 Strain对象的参数和方法
