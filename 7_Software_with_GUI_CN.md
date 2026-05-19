# 7. 图形界面软件

为了方便大家使用OpenCorr处理自己的实验数据，我们基于Dear ImGui (https://github.com/ocornut/imgui) 制作了一个图形界面的软件OpenCorr。该软件可以从GitHub上的Releases页面 (https://github.com/vincentjzy/OpenCorr/releases) 或opencorr.org上的下载页面 (https://opencorr.org/Download) 获取。下载后解压缩，运行OpenCorr.exe即可。注意压缩包中的DLL文件要跟OpenCorr.exe放在同一目录下。我们提供了一些例子供使用者参考，它们可从GitHub项目的gui_soft目录或openocrr.org的下载页面获取。本软件由我的博士生（李睿、任豪强）和我一起开发完成。目前它的功能和使用体验还很有限，我们非常欢迎大家试用并提供改进建议。

OpenCorr GUI作为共享软件发布，供大家免费使用（非商业的用途）。目前网站上提供功能受限的2.0版本供大家自由下载试用。如需获得完整功能的软件，请通过Email联系我们。 我们主要是想结识使用这款软件的朋友们，请用科研机构的Email给我们发邮件，并在邮件中提供你的姓名、单位以及研究项目。如果你是研究生，请同时提供导师姓名。我们会发送软件并对这些信息保密。



图7.1展示了2D DIC的图形界面。用户可读入一幅参考图和一系列目标图，并将它们显示于右侧的子窗口。在文件对话框选择多个文件时，可通过按住CTRL或SHIFT键用鼠标点选。

![image](D:/OpenCorr/Release_GitHub/img/gui_dic_2d_1.png)

图7.1 2D DIC的图形界面（多视图）



用户也可以通过点击右侧窗口上方的标签切换到单视图，以便进行DIC参数设置，具体效果如图7.2所示。大家可以看到窗口中央有一个黄色矩形，这是一个子区的示意图，可通过点击左侧配置面板中的“Show subset”复选框展现。然后用户可以一边调节子区尺寸，一边观察子区的散斑图案是否具有足够的辨识度。

![image](D:/OpenCorr/Release_GitHub/img/gui_dic_2d_2.png)

图7.2 2D DIC的图形界面（单视图）



图7.3展示了3D/stereo DIC的图形界面。注意左侧一列的图像默认为主视角，右侧一列为次视角。

![image](D:/OpenCorr/Release_GitHub/img/gui_dic_3d_1.png)

图7.3 3D/stereo DIC的图形界面（多视图）



为了运行3D/stereo DIC，软件还需要从一个CSV文件中读入相机标定参数。我们提供了存放标定参数的CSV文件实例（这些*_Calibration.csv放在examples压缩包内）。

![image](D:/OpenCorr/Release_GitHub/img/gui_dic_3d_2.png)

图7.4 选择和读入3D/stereo DIC所需的相机标定参数



对于形状不规则的试样，我们可以通过画出考察区域ROI并设置其中的POI间距来获得规则排布的POI阵列，如图7.5所示。图中黄色区域表示ROI，蓝色区域表示从ROI中去除的部分。生成的POI阵列可以通过点击面板上的Export键导出为CSV文件，供后续处理中重复使用。

![image](D:/OpenCorr/Release_GitHub/img/gui_dic_3d_3.png)

图7.5 在单视图中画出考察区域ROI（黄色区域表示ROI，蓝色区域表示从ROI中去除的部分）



图7.6展示了立体重构的图形界面，用于测量物体的表面形貌。我们还提供了直接从预存的CSV文件中读入POI阵列的功能。

![image](D:/OpenCorr/Release_GitHub/img/gui_stereo_reconstruction.png)

图7.6 立体重构的图形界面（多视图）



图7.7展示了DVC处理的图形界面，目前支持读入多页TIFF文件以及我们自定义的bin文件。用户可拖动子窗口上方的滑块查看不同截面（绿色直线代表相应截面的位置），三个视图的相对关系可参考右下角的坐标系与视角示意图。

![image](D:/OpenCorr/Release_GitHub/img/gui_dvc_1.png)

图7.7 DVC的图形界面（多视图）



用户也可以选择合适的截面绘制ROI，然后在控制面板上调节垂直截面方向上ROI的范围，如图7.8所示。

![image](D:/OpenCorr/Release_GitHub/img/gui_dvc_2.png)

图7.8 在选定截面上画出考察区域ROI（黄色区域表示ROI，蓝色区域表示从ROI中去除的部分）



与DIC中类似，用户可以切换到单视图中，以更高的分辨率查看选定的截面，据此绘制ROI或设置subset尺寸或subregion范围，如图7.9所示。

![image](D:/OpenCorr/Release_GitHub/img/gui_dvc_3.png)

图7.9 在单视图中查看选定的截面，检查subset尺寸或subregion范围



图7.10展示了计算应变的图形界面。用户可根据获得的位移场计算应变，同时通过调节拟合局部位移场的参数（例如子域尺寸和ZNCC阈值）获得更加精准的结果。我们同样提供了图形辅助功能，用户可读入参考图，然后点选配置面板上“Show subregion”复选框，查看右侧窗口中子域尺寸于POI间距的情况。

![image](D:/OpenCorr/Release_GitHub/img/gui_strain_calculation.png)

图7.10 应变计算的图形界面（单视图），DIC计算结果可从CSV文件中读入
