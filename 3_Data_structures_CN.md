# 3. 数据对象

## 3.1. 基本数据

（1）点（Point），代码保存在oc_point.h文件中。图3.1.1展示了point对象的参数和方法，Point的主要参数是它的坐标，构造函数用于生成指定坐标的point对象。由于point也可以视为矢量，表示点之间的偏移，因此我们制作了vectorNorm() 函数来计算该矢量的模长。同时，我们重载了基本运算符，其中“+”和“-”表示点的坐标叠加上一个正或负的偏移量，“\*”和“/”表示点的坐标分别乘以或除以一个标量。当运算符两侧均为point对象时，“\*”表示矢量的点积，而“/”表示矢量的叉积。“<<”被重载，用于直接输出point的坐标，格式为”x, y”（Point2D）或”x, y, z”(Point3D)。

![image](D:/OpenCorr/Release_GitHub/img/oc_point.png)

图3.1.1 Point对象的参数和方法



（2）数组（Array），代码保存在oc_array.h文件中。图3.1.2展示了Array对象的参数和方法。由于OpenCorr中的二维矩阵运算主要调用Eigen完成，该对象的定义较为简单。主要包括创建和删除二维、三维和四维数组的函数，以及自定义的Eigen矩阵类型。

![image](D:/OpenCorr/Release_GitHub/img/oc_array.png)

图3.1.2 Array对象的参数和方法



（3）图像（Image），代码保存在oc_image.h和oc_image.cpp文件中。图3.1.3展示了Image对象的参数和方法。对于二维图像，Image对象调用OpenCV的函数读取图像，获取其尺寸信息，并将其保存至尺寸相同的Eigen矩阵。三维体图像则保存在自定义二进制编码文件中，文件的格式为文件头（包含三个整型数，分别为体图像x，y和z方向的尺寸）和一个三维浮点数矩阵。注意三维矩阵亦可视为一维数组，存储的维度顺序为x，y和z。另一种体图像文件格式为包含多页的TIFF图像，可调用OpenCV的函数读取，其中每一页作为x-y平面的一层。

![image](D:/OpenCorr/Release_GitHub/img/oc_image.png)

图3.1.3 Image对象的参数和方法



## 3.2. DIC/DVC数据

（1）Subset（子区），代码保存在oc_subset.h和oc_subset.cpp文件中。图3.2.1展示了Subset对象的参数和方法。Subset可以视为一个特别的矩阵，其中心位于点center，其每个维度的长度分别等于对应的子区半径两倍再加一。

成员函数：

- fill(Image2D* image)或fill(Image3D* image)，从image对象中读取Subset参数定义位置的灰度数据并填入子区；
- zeroMeanNorm()，在完成子区所有点上灰度零均值归一化处理的同时，返回归一化系数。

![image](D:/OpenCorr/Release_GitHub/img/oc_subset.png)

图3.2.1 Subset对象的参数和方法



（2）Deformation（变形），代码保存在oc_deformation.h和oc_deformation.cpp文件中。图3.2.2展示了Deformation对象的参数和方法。Deformation包括二维变形和三维变形，变形可以通过一阶形函数和二阶形函数描述。在二维变形中，一阶形函数包括6个元素（位移u和v，及其分别在x和y方向的梯度），对应的变形矩阵warp_matrix的维度为3×3。二阶形函数包括12个元素（位移u和v，及其分别在x和y方向的一阶及二阶梯度），对应的变形矩阵warp_matrix的维度为6×6。在三维变形中，一阶形函数包括12个元素（位移u，v和w，及其分别在x，y和z方向的一阶梯度），对应的变形矩阵warp_matrix的维度为4×4。

成员函数：

- 无参数的setDeformation()，根据当前warp_matrix的内容设定变形分量；
- 有参数的setDeformation()，根据输入参数设定变形分量，更新相应的wrap_matrix；
- 用另一个变形对象作为参数的setDeformation()，根据给定变形对象设定变形分量，更新相应的wrap_matrix；
- setWarp()；根据当前变形分量计算warp_matrix；
- Point2D warp(Point2D& point)或Point3D warp(Point3D& point)，计算输入点经过变形后的新坐标。

![image](D:/OpenCorr/Release_GitHub/img/oc_deformation.png)

图3.2.2 Deformation对象的参数和方法



（3）POI（考察点，Point of interest），代码保存在oc_poi.h文件中。图3.2.3示了POI对象的参数和方法。POI对象继承point的属性和方法，在此基础上增加了一个变形矢量deformation、一个结果矢量result和一个应变矢量strain，以及一个Point对象subset_radius。其中deformation矢量用于保存DIC计算的变形或提供计算的变形初值，result矢量存储用于后续分析的计算参数（例如变形初值、最终的ZNCC、收敛时的最大位移增量、迭代次数和POI周围的近邻图像特征数），strain矢量存储计算的应变，subset_radius存储处理该考察点的所用的子区半径。POI2DS是针对3D/Stereo DIC设计的对象，它继承了Point2D，但包含的deformation矢量是三维零阶的，result矢量内则包括三个最终ZNCC值（分别为DIC运算和立体匹配的结果），POI分别在两帧参考视野和目标视野中的二维坐标，以及重构的变形前后三维点坐标（ref_coor和tar_coor）。

主要成员函数包括：

- clear()，将deformation、result、strain和subset_radius中所有元素清零。

![image](D:/OpenCorr/Release_GitHub/img/oc_poi.png)

图3.2.3 POI对象的参数和方法
