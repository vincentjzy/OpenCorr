# 3. Data structures

### 3.1. Basic data:

(1) Point (oc_point.h and oc_point.cpp). Figure 3.1.1 shows the parameters and methods included in this object. The main parameter of Point is its coordinates. It can also be regarded as a vector, indicating the offset from one point to another. Thus, a function (i.e. vectorNorm()) is provided to calculate the magnitude of the vector.  Moreover, operator "+" and "-" are overloaded to perform the superimposition of offsets on the coordinates of a point. Operator "\*" and "/" are used for the the cases that the coordinates of a point are multiplied or divided by a scalar. Moreover, Operator "\*" is also used to calculate the dot product of two point vectors, while operator "/" is used to calculate the cross product of two point vectors. Operator "<<" is overloaded to output the coordinate of a point, in the form of  "x, y" (Point2D) or "x, y, z" (Point3D).

![image](./img/oc_point.png)
*Figure 3.1.1. Parameters and methods included in Point object*

(2) Array (oc_array.h and oc_array.cpp). Figure 3.1.2 shows the parameters and methods included in this object. The object is simple, as the operations of 2D matrix in OpenCorr invoke the functions of Eigen. Array object only contains a few functions that create and delete 2D, 3D and 4D arrays, as well as the specifically defined Eigen matrices and vectors.

![image](./img/oc_array.png)
*Figure 3.1.2. Parameters and methods included in Array object*

(3) Image (oc_image.h and oc_image.cpp). Figure 3.1.3 shows the parameters and methods included in this object. In 2D case, OpenCV function is invoked to read image file and get its dimension, as well as store the data into the Eigen matrices with same size. In 3D case, the volumetric image is stored as a binary file, which includes a head of three integer (dimension x, y, and z) and a 3D float array. The 3D array can also be regarded as an 1D array, with the data arranged in the order of dimension: x, y, and then z. Another file format can be used to store volumetric image is TIFF image consisting of multiple pages, which can also be read using OpenCV function. In multi-page TIFF, each page is treated as a layer in x-y plane.

![image](./img/oc_image.png)
*Figure 3.1.3. Parameters and methods included in Image object*

### 3.2. DIC/DVC data:

(1) Subset (oc_subset.h and oc_subset.cpp). Figure 3.2.1 shows the parameters and methods included in this object. Subset can be regarded as a special matrix with its center located at a specific point. Each dimension of a subset equals to two times of corresponding radius plus one.

Member functions:

- fill(Image2D* image) or fill(Image3D* image), read grayscale data in a specific region from an Image object and feed the data into the subset.
- zeroMeanNorm(), perform the zero mean normalization of grayscale value at every point in the subset and return the normalization factor.

![image](./img/oc_subset.png)
*Figure 3.2.1. Parameters and methods included in Subset object*

(2) Deformation (oc_deformation.h and oc_deformation.cpp). Figure 3.2.2 shows the parameters and methods included in this object. Deformation can be described using the first order shape function and the second order shape function. In 2D case, the 1st order shape function contains 6 elements (displacements u and v, as well as their gradients along x- and y-axis), its warp_matrix is a 3x3 matrix. The 2nd order shape function contains 12 elements (displacements u and v, as well as their first and second order gradients along x- and y-axis), its warp_matrix is a 6x6 matrix. In 3D case, the 1st order shape function contains 12 elements (displacements u, v and w, as well as their gradients along x-, y- and z-axis), its warp_matrix is a 4x4 matrix.

Member functions:

- setDeformation() without input, set the deformation elements according to current warp_matrix;
- setDeformation() with input, set the deformation elements according to the input, and update warp_matrix;
- setDeformation() with another Deformation instance as input, set the deformation elements and update warp_matrix, according to the given instance;
- setWarp(), update warp_matrix according to current deformation elements;
- Point2D warp(Point2D& point) or Point3D warp(Point3D& point), calculate the coordinates of a point experienced the deformation.

![image](./img/oc_deformation.png)
*Figure 3.2.2. Parameters and methods included in Deformation object*

(3) POI (Point of interest, oc_poi.h and oc_poi.cpp). Figure 3.2.3 shows the parameters and methods included in this object. POI inherits the properties and methods from Point object. In addition, it contains a deformation vector, a result vector, a strain vector and a subset_radius of Point object. Vector deformation stores the calculated deformation or provides the initial guess. Vector result stores the parameters for data analysis (e.g. initial guess of displacement, final ZNCC value, maximum norm of deformation increment at the end of iterative procedure, numbers of iteration steps, and collected image features around the POI). Vector strain stores the calculated strains. Object subset_radius stores the subset radius in the processing of the POI. Especially, POI2DS is designed for 3D/stereo DIC, which inherits the properties from Point2D, but contains a three dimensional deformation vector of the zeroth order. Its result vector includes three ZNCC values in stereo and temporal matching, 2D coordinates of the POI in the two reference views and two target views, as well as the reconstructed 3D coordinates before and after deformation (ref_coor and tar_coor).

Member functions:

- clear(), set all the elements in vectors deformation, result, strain, and subset_radius to be zero.

![image](./img/oc_poi.png)
*Figure 3.2.3. Parameters and methods included in POI object*
