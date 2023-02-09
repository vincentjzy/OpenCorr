![](./img/title_figure.png)

# OpenCorr： An open source C++ library for digital image correlation 

OpenCorr is an open source C++ library for research and development of 2D, 3D/stereo, and volumetric digital image correlation (DIC). It aims to provide a developer-friendly, lightweight, and efficient kit to the users who are willing to study the state-of-the-art algorithms of DIC and DVC (digital volume correlation), or to create DIC and DVC programs for their specific applications.

Comments and suggestions are most welcome. You may reach us via

1. Email: zhenyujiang (at) scut.edu.cn;
2. Discussion in GitHub repository;
3. Tencent QQ group: 597895040

Users can also access the information of OpenCorr via website [opencorr.org](http://opencorr.org) .

# Important updates

>2021.04.23, OpenCorr is released to public.
>
>2021.04.30, Modify structure of DIC module and stereovision module.
>
>2021.05.08, A brief instruction of framework is released.
>
>2021.05.17, Improve the adaptability for Linux and release a cool title figure.
>
>2021.06.12, Release an example to demonstrate the calculation of strains, update the documentation.
>
>2021.08.14, Release the GPU accelerated module of ICGN algorithm and an example, instruction can be found in **Instructions** (5. GPU acceleration).
>
>2021.11.03, Release an example to implement stereo DIC (3D DIC), thoroughly improve the related modules and documentation.
>
>2021.11.16, Implement the calculation of 2D and 3D strains for surface measurement.
>
>2022.04.27, A major update, including (i) introduction of nanoflann to speed up the searching for nearest neighbors in Feature Affine method and strain calculation; (ii) update of the third party libraries (Eigen and OpenCV) to the latest stable version; (iii) regularization of the codes.
>
>2022.05.03, Estimation of parallax for epipolar constraint aided matching, and an example of stereo matching and reconstruction combining the methods using SIFT feature and epipolar constraint. 
>
>2022.06.23, Release DVC module, which includes 3D FFTCC and 3D ICGN algorithms. The related modules are expanded accordingly.
>
>2022.10.13, Fix the VRAM leak issue of GPU accelerated ICGN module.
>
>2022.10.21, Fix the conflict of calling NearestNeighbor instance by multiple threads in modules FeatureAffine and Strain.
>
>2022.12.23, Release of OpenCorr 1.0. Modules Feature and FeatureAffine are upgraded by introducing calsses SIFT3D and FeatureAffine3D, respectively. The codes, examples, and documents are thoroughly updated.
>
>2023.01.13, A regular update, including (i) adding a module of Newton-Raphson algorithm (NR) for 2D DIC; (ii) giving an example of self-adaptive DIC, which can dynamically optimize the size and shape of subset at each POI; (iii) fixing a potential bug in module Interpolation; (iv) updating the codes and documents.
>
>2023.01.18, Add description of examples.

# Instructions

1. [Get started](./1_Get_started.md)
2. [Framework](./2_Framework.md)
3. [Data structures](./3_Data_structures.md)
4. [Processing methods](./4_Processing_methods.md)
5. [GPU acceleration](./5_GPU_acceleration.md)
6. [Examples](./6_Examples.md)

# Developers

- Dr JIANG Zhenyu, Professor, South China University of Technology
- Mr ZHANG Lingqi, PhD candidate, Tokyo Institute of Technology
- Dr WANG Tianyi, PostDoc, BrookHaven Natinoal Lab
- Dr CHEN Wei, Chief research engineer, Midea
- Mr HUANG Jianwen, Software engineer, SenseTime
- Mr YANG Junrong, Software engineer, Tencent
- Mr LIN Aoyu, Engineer, China Southern Power Grid

# Acknowledgements

OpenCorr demonstrates our exploration of DIC and DVC methods in recent years, which got continuous financial support from National Natural Science Foundation of China. I would like to give my special thanks to two collaborators for their enthusiastic support: Professor QIAN Kemao (Nanyang Technological University) and Professor DONG Shoubin (South China University of Technology).

# Related publication

Users may refer to our papers for more information about the detailed principles and implementations of the algorithms in OpenCorr.

1. Z. Jiang, Q. Kemao, H. Miao, J. Yang, L. Tang, Path-independent digital image correlation with high accuracy, speed and robustness, Optics and Lasers in Engineering (2015) 65: 93-102. (https://doi.org/10.1016/j.optlaseng.2014.06.011)

2. L. Zhang, T. Wang, Z. Jiang, Q. Kemao, Y. Liu, Z. Liu, L. Tang, S. Dong, High accuracy digital image correlation powered by GPU-based parallel computing, Optics and Lasers in Engineering (2015) 69: 7-12. (https://doi.org/10.1016/j.optlaseng.2015.01.012)

3. T. Wang, Z. Jiang, Q. Kemao, F. Lin, S.H. Soon, GPU accelerated digital volume correlation, Experimental Mechanics (2016) 56(2): 297-309. (https://doi.org/10.1007/s11340-015-0091-4)

4. Z. Pan, W. Chen, Z. Jiang, L. Tang, Y. Liu, Z. Liu, Performance of global look-up table strategy in digital image correlation with cubic B-spline interpolation and bicubic interpolation, Theoretical and Applied Mechanics Letters (2016) 6(3): 126-130. (https://doi.org/10.1016/j.taml.2016.04.003)

5. W. Chen, Z. Jiang, L. Tang, Y. Liu, Z. Liu, Equal noise resistance of two mainstream iterative sub-pixel registration algorithms in digital image correlation, Experimental Mechanics (2017) 57(6): 979-996. (https://doi.org/10.1007/s11340-017-0294-y)

6. J. Huang, L. Zhang, Z. Jiang, S. Dong, W. Chen, Y. Liu, Z. Liu, L. Zhou, L. Tang, Heterogeneous parallel computing accelerated iterative subpixel digital image correlation, Science China Technological Sciences (2018) 61(1):74-85. (https://doi.org/10.1007/s11431-017-9168-0)

7. J. Yang, J. Huang, Z. Jiang, S. Dong, L. Tang, Y. Liu, Z. Liu, L. Zhou, SIFT-aided path-independent digital image correlation accelerated by parallel computing, Optics and Lasers in Engineering (2020) 127: 105964. (https://doi.org/10.1016/j.optlaseng.2019.105964)

8. J. Yang, J. Huang, Z. Jiang, S. Dong, L. Tang, Y. Liu, Z. Liu, L. Zhou, 3D SIFT aided path independent digital volume correlation and its GPU acceleration, Optics and Lasers in Engineering (2021) 136: 106323. (https://doi.org/10.1016/j.optlaseng.2020.106323)

9. L. Cai, J. Yang, S. Dong, Z. Jiang. GPU accelerated parallel reliability-guided digital volume correlation with automatic seed selection based on 3D SIFT. Parallel Computing (2021) 108: 102824. (https://doi.org/10.1016/j.parco.2021.102824)

10. A. Lin, R. Li, Z. Jiang, S. Dong, Y. Liu, Z. Liu, L. Zhou, L. Tang, Path independent stereo digital image correlation with high speed and analysis resolution, Optics and Lasers in Engineering (2022) 149: 106812. (https://doi.org/10.1016/j.optlaseng.2021.106812)

# Impact

We are jubilant that OpenCorr helps other colleagues in their study as a benchmark. We would appreciate it If anyone could let us know the work not yet included in this list.

1. Yuxi Chi, Bing Pan. Accelerating parallel digital image correlation computation with feature mesh interpolation. Measurement (2022) 199: 111554. (https://doi.org/10.1016/j.measurement.2022.111554)
2. Wang Lianpo. Super-robust digital image correlation based on learning template. Optics and Lasers in Engineering (2022) 158: 107164. (https://doi.org/10.1016/j.optlaseng.2022.107164)
