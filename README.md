![](./img/title_figure.png)

# OpenCorr： An open source C++ library for digital image correlation 

OpenCorr is an open source C++ library for research and development of 2D, 3D/stereo, and volumetric digital image correlation (DIC). It provides a developer-friendly, lightweight, and efficient kit to the users who are willing to study the state-of-the-art algorithms of DIC and DVC (digital volume correlation), or to create DIC and DVC programs for their specific applications. OpenCorr GUI is a shareware with graphical user interface developed based on OpenCorr, demonstrating main functions of this library.

Comments and suggestions are most welcome. You may reach us via

1. Email: zhenyujiang (at) scut.edu.cn;
2. Discussion in GitHub repository;
3. Tencent QQ group: 597895040

Users can also access the information of OpenCorr via website [opencorr.org](http://opencorr.org) .

![image](./img/gui_dvc_2.png)

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
>2022.04.27, Major update, including (i) introduction of nanoflann to speed up the searching for nearest neighbors in Feature Affine method and strain calculation; (ii) update of the third party libraries (Eigen and OpenCV) to the latest stable version; (iii) regularization of the codes.
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
>2023.01.13, Regular update, including (i) adding a module of Newton-Raphson algorithm (NR) for 2D DIC; (ii) giving an example of self-adaptive DIC, which can dynamically optimize the size and shape of subset at each POI; (iii) fixing a potential bug in module Interpolation; (iv) updating the codes and documents.
>
>2023.01.18, Add description of examples.
>
>2023.03.06, A research paper titled "OpenCorr: An open source library for research and development of digital image correlation" is published in Optics and Lasers in Engineering.
>
>2024.02.07, Major update of ICGN module: (i) GPU accelerated ICGN (ICGNGPU) has been completely reconstructed, adding the function of DVC. The calling method of ICGNGPU is now similar to the CPU version. Most of redundant data conversion and transfer are eliminated. Two examples are added to the folder /examples to demonstrate the use of ICGNGPU. (ii) CPU version is modified to improve the efficiency.
>
>2024.06.14, Release of OpenCorr GUI 1.0. Codes and examples are modified.
>
>2024.07.26, Release of OpenCorr GUI 2.0, which supports 2D, 3D/stereo DIC and DVC.
>
>2024.12.11, Regular update, including: (i) adding a module of inverse compositional Levenberg-Marquardt algorithm (ICLM); (ii) improving the codes according to the commits proposed by Czcibor Bohusz-Dobosz.

# Instructions

1. [Get started](./1_Get_started.md)
2. [Framework](./2_Framework.md)
3. [Data structures](./3_Data_structures.md)
4. [Processing methods](./4_Processing_methods.md)
5. [GPU acceleration](./5_GPU_acceleration.md)
6. [Examples](./6_Examples.md)
7. [Software with GUI](./7_Software_with_GUI.md)

# Developers

- Dr JIANG Zhenyu, Professor, South China University of Technology
- Dr ZHANG Lingqi, PostDoctoral researcher, RIKEN
- Dr WANG Tianyi, PostDoctoral researcher, BrookHaven National Laboratory
- Dr CHEN Wei, Chief research engineer, Midea
- Mr HUANG Jianwen, Software engineer, SenseTime
- Mr YANG Junrong, Software engineer, Tencent
- Mr LIN Aoyu, Engineer, China Southern Power Grid
- Mr LI Rui, PhD candidate, South China University of Technology
- Mr REN Haoqiang, PhD candidate, South China University of Technology

# Acknowledgements

OpenCorr demonstrates our exploration of DIC and DVC methods in recent years, which got continuous financial support from National Natural Science Foundation of China. I would like to give my special thanks to two collaborators for their enthusiastic support: Professor QIAN Kemao (Nanyang Technological University) and Professor DONG Shoubin (South China University of Technology).

# Related publication

Users may refer to our papers for more information about the detailed principles and implementations of the algorithms in OpenCorr. If you feel OpenCorr helps, please cite the following paper to make it known by more people.

```json
@article{jiang2023opencorr,
  title={OpenCorr: An open source library for research and development of digital image correlation},
  author={Jiang, Zhenyu},
  journal={Optics and Lasers in Engineering},
  volume={165},
  pages={107566},
  year={2023},
  publisher={Elsevier}
}
```

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
11. Z. Jiang, OpenCorr: An open source library for research and development of digital image correlation. Optics and Lasers in Engineering (2023) 165: 107566. (https://doi.org/10.1016/j.optlaseng.2023.107566)
12. W. Yin, Y. Ji, J. Chen, R. Li, S. Feng, Q. Chen, B. Pan, Z. Jiang, C. Zuo, Initializing and accelerating Stereo-DIC computation using semi-global matching with geometric constraints. Optics and Lasers in Engineering (2024) 172: 107879. (https://doi.org/10.1016/j.optlaseng.2023.107879)

# Impact

We are jubilant at that OpenCorr helps other colleagues in their study as a software development kit or testing benchmark. We would appreciate it if anyone could let us know the work not yet included in this list.

1. Yuxi Chi, Bing Pan. Accelerating parallel digital image correlation computation with feature mesh interpolation. Measurement (2022) 199: 111554. (https://doi.org/10.1016/j.measurement.2022.111554)
2. Wang Lianpo. Super-robust digital image correlation based on learning template. Optics and Lasers in Engineering (2022) 158: 107164. (https://doi.org/10.1016/j.optlaseng.2022.107164)
3. Y Li, L Wei, X Zhang. Measurement of nonuniform strain distribution in CORC cable due to bending process by a segmentation-aided stereo digital image correlation. Experimental Mechanics (2023) 63: 813-822. (https://doi.org/10.1007/s11340-023-00953-y)
4. Yusheng Wang, Zhixiang Huang, Pengfei Zhu, Rui Zhu, Tianci Hu, Dahai Zhang, Dong Jiang. Effects of compressed speckle image on digital image correlation for vibration measurement. Measurement (2023) 217: 113041. (https://doi.org/10.1016/j.measurement.2023.113041)
5. Yong Su. An analytical study on the low-pass filtering effect of digital image correlation caused by under-matched shape functions. Optics and Lasers in Engineering (2023) 168: 107679. (https://doi.org/10.1016/j.optlaseng.2023.107679)
6. Chuanguo Xiong , Yuhan Gao, Yuhua huang , Fulong Zhu. Specular surface deformation measurement based on projected-speckle deflectometry with digital image correlation. Optics and Lasers in Engineering (2023) 170: 107776. (https://doi.org/10.1016/j.optlaseng.2023.107776)
7. Xiao Hong, Li Chengnan, Feng Mingchi. Large deformation measurement method of speckle images based on deep learning. Acta Optica Sinica (2023) 43(14): 1412001. (https://doi.org/10.3788/AOS222084)
8. Jiaqiu Wang, Hao Wu, Zhengduo Zhu5, Hujin Xie, Han Yu, Qiuxiang Huang, Yuqiao Xiang, Phani Kumari Paritala, Jessica Benitez Mendieta, Haveena Anbananthan, Jorge Alberto Amaya Catano, Runxin Fang, Luping Wang, Zhiyong Li. Impact of Speckle Deformability on Digital Imaging Correlation. IEEE Access (2024) 12:66466-66477. (https://doi.org/10.1109/ACCESS.2024.3398786)
9. Derui Li, Bin Cheng, Sheng Xiang. Direct cubic B-spline interpolation: A fuzzy interpolating method for weightless, robust and accurate DVC computation. Optics and Lasers in Engineering (2024) 172: 107886. (https://doi.org/10.1016/j.optlaseng.2023.107886)
10. Datao Li, Xiahui Wei, Yingrong Gao, Jinsong Jiang, Wei Xia, Binhua Wang. Investigations on tensile mechanical properties of rigid insulation tile materials at elevated temperatures based on digital image correlation algorithm. Construction and Building Materials (2024) 413: 134925. (https://doi.org/10.1016/j.conbuildmat.2024.134925)
11. Jiashuai Yang, Kemao Qian, Lianpo Wang. R3-DICnet: an end-to-end recursive residual refinement DIC network for larger deformation measurement. Optics Express (2024) 32(1): 907-921. (https://doi.org/10.1364/OE.505655)
12. Longyong Tong, Hang Zhou, Brian Sheil. Multicore CPU-based parallel computing accelerated digital image correlation for large soil deformations measurement. Computers and Geotechnics (2024) 166: 106027. (https://doi.org/10.1016/j.compgeo.2023.106027)
13. Hengrui Cui, Zhoumo Zeng, Jian Li, Hui Zhang, Fenglong Yang, Shili Chen. The effect of error coefficient matrices and correlation criteria on dic computation errors. Optics and Lasers in Engineering (2024) 174: 107954. (https://doi.org/10.1016/j.optlaseng.2023.107954)
14. Siqi Wang, Zehui Zhu, Tao Ma, Jianwei Fan. Asphalt concrete characterization using digital image correlation: A systematic review of best practices, applications, and future vision. Journal of Testing and Evaluation (2024) 52(4): 20230485. (https://doi.org/10.1520/JTE20230485)
15. Yujia Cheng, Quanbao Wang, Lightweight digital image correlation network for stratospheric airship skin deformation measurement. Optical Engineering (2024) 63(7): 073103. (https://doi.org/10.1117/1.OE.63.7.073103)
16. Yahong Feng, Lianpo Wang. Stereo-DICNet: An efficient and unified speckle matching network for stereo digital image correlation measurement. Optics and Lasers in Engineering (2024) 179: 108267. (https://doi.org/10.1016/j.optlaseng.2024.108267)
17. Yong Su, Li Lao. Modeling the measurement accuracy of one-dimensional boundary subsets in digital image correlation. Optics and Lasers in Engineering (2024) 181: 108362. (https://doi.org/10.1016/j.optlaseng.2024.108362)
18. Han Yu, Zidun Wang, Hao Wu, Zhengduo Zhu, Jiaqiu Wang, Runxing Fang, Shanglin Wu, Hujin Xie, Xianjue Huang, Jessica Benitez Mendieta, Haveena Anbananthan, Zhiyong Li. In-vivo left atrial surface motion and strain measurement using novel mesh regularized image block matching method with 4D-CTA. Journal of Biomechanics (2024) 176: 112354. (https://doi.org/10.1016/j.jbiomech.2024.112354)
19. Peng Shi, Keming Zhang, Junxiang Peng. Method for luminescent digital image correlation deformation measurement in non-illuminated environments. Applied Optics (2024) 63(31): 8248-8256. (https://doi.org/10.1364/AO.540467)
20. Guowen Wang, Yuan Zhou, Zhiyuan Wang, Jian Zhou, Shanyong Xuan, Xuefeng Yao. StrainNet-LD: Large Displacement digital image correlation based on deep learning and displacement-field decomposition. Optics and Lasers in Engineering (2024) 183: 108502. (https://doi.org/10.1016/j.optlaseng.2024.108502)
21. Jiashuai Yang, Yahong Feng, Lianpo Wang. Efficient and robust deformation measurement based on unsupervised learning. Measurement (2024) 242 (Part A): 115908. (https://doi.org/10.1016/j.measurement.2024.115908)
22. Zhengduo Zhu, Jiaqiu Wang, Hao Wu, Minglong Chen, Zidun Wang, Runxin Fang, Xianjue Huang, Hujin Xie, Han Yu, Yuchu Tian, Zhiyong Li. Performance test of digital volume correlation on tracking left atrium motion from cardiac CT. Acta Mechanica Sinica (2024) 41(4): 624216. (https://doi.org/10.1007/s10409-024-24216-x)

Counting of visitors started from 03 Aug, 2023
![Visitor Count](https://profile-counter.glitch.me/vincentjzy/count.svg)
