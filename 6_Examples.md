# 6. Examples

A few examples are given to demonstrate how we work with OpenCorr.  The codes, images, and computation results can be found in the subdirectory (./examples). The usage of modules generally follows a simple procedure includes: (1) create an instance; (2) set parameters; (3) prepare data which need pre-computation; (4) compute; (5) delete the instance.

#### 2D DIC

1. test_2d_dic_fftcc_icgn1.cpp

This example uses module FFTCC to determine integral-pixel displacements at each POI, and then uses module ICGN with the 1st order shape function to get high accuracy deformation. The images come from Sample 12 of 2D DIC challenge 1.0, provided by courtesy of International DIC society. The image series records the uniaxial tension test of a plate with a hole in the middle.

2. test_2d_dic_sift_icgn2.cpp

This example uses module FeatureAffine to roughly estimate the sub-pixel deformation at each POI, and then uses module ICGN with the 2nd order shape function to get high accuracy results. The reference image comes from Sample 9 of 2D DIC challenge 1.0, provided by courtesy of International DIC society. We generate a target image by rotating the reference image for 170 degree, to show the outstanding robustness of FeatureAffine.

3. test_2d_dic_fftcc_nr1.cpp

This example combines modules FFTCC and NR with the 1st order shape function. It is nostalgia for the early stage of DIC. Users may compare it with test_2d_dic_fftcc_icgn1.cpp.

4. test_2d_dic_strain.cpp

This example uses module Strain to calculate strains based on the displacements determined by test_2d_dic_fftcc_icgn1.cpp. Module Strain can also be invoked right after other DIC modules, as demonstrated in test_2d_dic_fftcc_nr1.cpp.

5. test_2d_dic_gpu_icgn.cpp

This example demonstrates the use of GPU accelerated ICGN (ICGNGPU) with shape functions of the 1st and the 2nd order.

6. test_2d_dic_self_adaptive_subset.cpp

   This example provides an instance of how to develop new DIC algorithms. FeatureAffine and ICGN are modified to realize a self-adpative DIC method, in which the size and shape of subset, as well as location of POI are dynamically optimized at each POI according to the nearby image features. Our experiments show that the self-adaptive can reach the optimal configuration in a good agreement with the meticulous decision based on a series of trials. The Images are generated using a Boolean model (Sur et al. J Math Imaging Vis, 2018, 60: 634-650), simulating the tension of a plate with large strains (30%~45%).

7. test_2d_dic_fftcc_iclm1.cpp

   This example uses module FFTCC to determine integral-pixel displacements at each POI, and then uses module ICLM with the 1st order shape function to get high accuracy deformation. The images also come from Sample 12 of 2D DIC challenge 1.0. It could be compared with test_2d_dic_fftcc_icgn1.cpp.

8. test_2d_dic_sift_iclm2.cpp

   This example uses module FeatureAffine to roughly estimate the sub-pixel deformation at each POI, and then uses module ICLM with the 2nd order shape function to get high accuracy results. The performance of this example could be compared with test_2d_dic_sift_icgn2.cpp

#### Stereo/3D DIC

1. test_3d_dic_epipolar_sift.cpp

This example uses modules SIFT, EpipolarSearch, and ICGN with the 2nd order shape function to realize stereo DIC. The images come from Stereo Sample 3 of Stereo DIC challenge, provided by courtesy of International DIC society. The image series records the uniaxial tension test of a D-shape steel part.

2. test_3d_dic_strain.cpp

This example uses module Strain to calculate strains based on the displacements determined by test_3d_dic_epipolar_sift.cpp.

3. test_3d_reconstruction_epipolar.cpp

This example demonstrates the measurement of object profile, combining modules EpipolarSearch, ICGN with the 2nd order shape function, and Stereovision. The images come from Stereo Sample 1 of Stereo DIC challenge, provided by courtesy of International DIC society. The image pair is acquired from left and right view of a plate with three kinds of relief sculptures, such as semi-cylinder, triangular prism and square platform. It is noteworthy that the right image is set as the reference.

4. test_3d_reconstruction_sift_epipolar.cpp

This example is an improved version of test_3d_reconstruction_epipolar.cpp. It achieves a better balance between efficiency and robustness by combining FeatureAffine and EpipolarSearch. In this method, the parallax between left and right view are estimated assuming that it follows a bi-linear distribution.

#### DVC

1. test_dvc_fftcc_icgn1.cpp

This example uses module FFTCC to determine integral-pixel displacements at each POI, and then uses module ICGN with the 1st order shape function to get high accuracy deformation. The volumes are reconstructed from CT scan of an aluminum foam specimen subjected to uniaxial compression.

2. test_dvc_sift_icgn1.cpp

This example uses module FeatureAffine to roughly estimate the sub-pixel deformation at each POI, and then uses module ICGN with the 1st order shape function to get high accuracy results. The volume pair is provided by courtesy of Correlated Solutions, which records the lateral compression test of a rubber ring. The volumetric images can be downloaded from [opencorr.org](https://opencorr.org/download/) or [correlatedsolutions.com](https://downloads.correlatedsolutions.com/Torus.zip).

3. test_dvc_strain.cpp

This example uses module Strain to calculate strains based on the displacements determined by test_dvc_sift_icgn1.cpp.

4. test_dvc_gpu_icgn.cpp

This example demonstrates the use of GPU accelerated ICGN with shape functions of the 1st order (ICGN3D1GPU) for DVC. It also gives a comparison between the CPU version and GPU version.
