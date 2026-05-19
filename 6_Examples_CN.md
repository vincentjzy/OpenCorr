# 6. 实例说明

我们提供了一些实例供大家参考，它们展示了如何使用OpenCorr开发DIC和DVC软件.  相关的代码、图像和计算结果等均陈列于example子目录内。OpenCorr中模块的使用大致包含几个步骤：（1）创建一个模块的实例；（2）设定参数；（3）准备必要的预计算数据；（4）计算；（5）删除模块的实例。

## 2D DIC

1. test_2d_dic_fftcc_icgn1.cpp

这个例子使用FFTCC模块估计每个POI的整像素位移，然后调用一阶形函数的ICGN模块获得高精度的变形结果。图像来自2D DIC challenge 1.0的Sample 12（由国际DIC协会提供）。这组图像记录了中间有圆孔的平板试样的单轴拉伸试验。

2. test_2d_dic_sift_icgn2.cpp

这个例子使用FeatureAffine模块大致估算每个POI的亚像素变形，然后调用二阶形函数的ICGN模块获取高精度的变形结果。参考图取自2D DIC challenge 1.0的Sample 9（由国际DIC协会提供）。原例子只包含10度以内的旋转，我们用参考图计算生成了一幅旋转170度的目标图，以展示FeatureAffine的强适应性。

3. test_2d_dic_fftcc_nr1.cpp

这个例子调用了FFTCC和一阶形函数的NR模块。它处理的图像与test_2d_dic_fftcc_icgn1.cpp相同，可用来比较两种经典的迭代DIC算法。

4. test_2d_dic_strain.cpp

这个例子使用Strain模块根据test_2d_dic_fftcc_icgn1.cpp得到的位移场计算应变场。Strain模块也可以直接在其他DIC模块后调用，如test_2d_dic_fftcc_nr1.cpp中展示的情况。

5. test_2d_dic_gpu_icgn.cpp

这个例子展示了GPU进行并行计算加速ICGN模块的运用，包括一阶和二阶形函数的ICGN。

6. test_2d_dic_self_adaptive_subset.cpp

这个例子展示了如何利用OpenCorr开发新算法。我们通过修改FeatureAffine模块和ICGN模块，实现了一种自适应DIC（Self-adaptive DIC）方法 。该方法可以根据每个POI附近图像特征的情况动态地优化子区尺寸和形状，以及POI的位置。我们的实验结果表明，该方法自动优化后的子区参数，与富有经验的实验人员基于一系列固定尺寸子区试算结果估计的子区参数基本一致 。散斑图像序列利用Sur等人的Boolean模型生成(Sur et al. J Math Imaging Vis, 2018, 60: 634-650)，模拟了平板试样承受较大拉伸应变（30%~45%）的试验。

7. test_2d_dic_fftcc_iclm1.cpp

这个例子使用FFTCC模块估计每个POI的整像素位移，然后调用一阶形函数的ICLM模块获得高精度的变形结果。图像来自2D DIC challenge 1.0的Sample 12（由国际DIC协会提供）。其性能可与test_2d_dic_fftcc_icgn1.cpp比较。

8. test_2d_dic_sift_iclm2.cpp

这个例子使用FeatureAffine模块大致估算每个POI的亚像素变形，然后调用二阶形函数的ICLM模块获取高精度的变形结果。其性能可与test_2d_dic_sift_icgn2.cpp比较。

## Stereo/3D DIC

1. test_3d_dic_epipolar_sift.cpp

这个例子使用SIFT、EpipolarSearch和二阶形函数的ICGN模块实现了3D DIC方法。图像来自Stereo DIC challenge的Stereo Sample 3（由国际DIC协会提供）。这组图像记录了一个D形钢制平板试样的单轴拉伸试验。

2. test_3d_dic_strain.cpp

这个例子使用Strain模块根据test_3d_dic_epipolar_sift.cpp得到的位移场计算应变场。

3. test_3d_reconstruction_epipolar.cpp

这个例子展示了如何使用EpipolarSearch、二阶形函数的ICGN和Stereovision模块测量物体的表面形貌。图像来自Stereo DIC challenge的Stereo Sample 1（由国际DIC协会提供）。这组图像为从左右两个视角拍摄的一个平板浮雕，包含半圆柱、三角棱柱和正方平台三种形状。注意这个例子中右图被设定为参考图。

4. test_3d_reconstruction_sift_epipolar.cpp

这个例子是test_3d_reconstruction_epipolar.cpp的一个改进版本，通过合理结合FeatureAffine和EpipolarSearch模块达到了效率和稳健性之间更好的平衡。该方法还利用图像特征估算了左右视场之间的视差，视差可用双线性空间分布近似。

## DVC

1. test_dvc_fftcc_icgn1.cpp

这个例子使用FFTCC模块估计每个POI的整像素位移，然后调用一阶形函数的ICGN模块获得高精度的变形结果。体图像为根据CT扫描的承受单轴压缩的泡沫铝试样。

2. test_dvc_sift_icgn1.cpp

这个例子使用FeatureAffine模块大致估算每个POI的亚像素变形，然后调用二阶形函数的ICGN模块获取高精度的变形结果。体图像由Correlated Solutions提供，记录了一个橡胶环承受侧向压缩的试验。图像文件可从以下两个网址下载：[opencorr.org](https://opencorr.org/download/) or [correlatedsolutions.com](https://downloads.correlatedsolutions.com/Torus.zip).

3. test_dvc_strain.cpp

这个例子使用Strain模块根据test_dvc_sift_icgn1.cpp得到的位移场计算应变场。

4. test_dvc_gpu_icgn.cpp

这个例子展示了GPU进行并行计算加速ICGN模块在DVC中的运用，其中ICGN算法使用了一阶形函数（ICGN3D1GPU），同时也比较了CPU版本和GPU的结果与性能。
