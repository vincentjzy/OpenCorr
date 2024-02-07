/*
 This example demonstrates how to use OpenCorr with GPU acceleratd ICGN
 algorithms (with the 1st or the 2nd order shape function) to make a path
 independent DIC method. The initial guess at each POI is estimated using the
 SIFT feature guided method.
*/

#include <fstream>

#include "opencorr.h"
#include "opencorr_gpu.h"

using namespace opencorr;
using namespace std;

int main()
{
	//set files to process
	string ref_image_path = "d:/dic_tests/2d_dic/oht_cfrp_0.bmp"; //replace it with the path on your computer
	string tar_image_path = "d:/dic_tests/2d_dic/oht_cfrp_4.bmp"; //replace it with the path on your computer
	Image2D ref_img(ref_image_path);
	Image2D tar_img(tar_image_path);

	//initialize papameters for timing
	double timer_tic, timer_toc, consumed_time;
	vector<double> computation_time;

	//get the time of start
	timer_tic = omp_get_wtime();

	//create instances to read and write csv files
	string file_path;
	string delimiter = ",";
	ofstream csv_out; //instance for output calculation time
	IO2D in_out; //instance for input and output DIC data
	in_out.setDelimiter(delimiter);
	in_out.setHeight(ref_img.height);
	in_out.setWidth(ref_img.width);

	//set POIs
	Point2D upper_left_point(30, 30);
	vector<POI2D> poi_queue;
	int poi_number_x = 100;
	int poi_number_y = 300;
	int grid_space = 2;

	//store POIs in a queue
	for (int i = 0; i < poi_number_y; i++)
	{
		for (int j = 0; j < poi_number_x; j++)
		{
			Point2D offset(j * grid_space, i * grid_space);
			Point2D current_point = upper_left_point + offset;
			POI2D current_poi(current_point);
			poi_queue.push_back(current_poi);
		}
	}

	//set OpenMP parameters
	int cpu_thread_number = omp_get_num_procs() - 1;
	omp_set_num_threads(cpu_thread_number);

	//set DIC parameters
	int subset_radius_x = 16;
	int subset_radius_y = 16;
	int max_iteration = 10;
	float max_deformation_norm = 0.001f;

	//create instances of SIFT and FeatureAffine for initial guess estimation
	SIFT2D* sift = new SIFT2D();
	sift->setImages(ref_img, tar_img);
	FeatureAffine2D* feature_affine = new FeatureAffine2D(subset_radius_x, subset_radius_y, cpu_thread_number);
	feature_affine->setImages(ref_img, tar_img);

	//create instances of ICGN1 on CPU and GPU
	ICGN2D1GPU* icgn1 = new ICGN2D1GPU(subset_radius_x, subset_radius_y, max_deformation_norm, max_iteration);
	ICGN2D2GPU* icgn2 = new ICGN2D2GPU(subset_radius_x, subset_radius_y, max_deformation_norm, max_iteration);

	Img2D ref_img_gpu, tar_img_gpu;
	ref_img_gpu.width = ref_img.width;
	ref_img_gpu.height = ref_img.height;
	ref_img_gpu.data = new float[ref_img_gpu.width * ref_img_gpu.height];

	tar_img_gpu.width = tar_img.width;
	tar_img_gpu.height = tar_img.height;
	tar_img_gpu.data = new float[tar_img_gpu.width * tar_img_gpu.height];

#pragma omp parallel for
	for (int i = 0; i < ref_img.height; i++)
	{
		for (int j = 0; j < ref_img.width; j++)
		{
			ref_img_gpu.data[i * ref_img_gpu.width + j] = ref_img.eg_mat(i, j);
			tar_img_gpu.data[i * ref_img_gpu.width + j] = tar_img.eg_mat(i, j);
		}
	}

	icgn1->setImages(ref_img_gpu, tar_img_gpu);
	icgn2->setImages(ref_img_gpu, tar_img_gpu);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //0

	//display the time of initialization on screen
	cout << "Initialization with " << poi_queue.size() << " POIs takes " << consumed_time << " sec, " << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	//Extraction and matching of features
	sift->prepare();
	sift->compute();

	//FeatureAffine instance estimates the deformation at POIs according the neighbor features
	feature_affine->setKeypointPair(sift->ref_matched_kp, sift->tar_matched_kp);
	feature_affine->prepare();
	feature_affine->compute(poi_queue);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //1

	//display the time of processing on screen
	cout << "SIFT feature guided deformation estimation takes " << consumed_time << " sec." << std::endl;

	vector<POI2D> poi_queue1(poi_queue);
	vector<POI2D> poi_queue2(poi_queue);

	//get the time of start
	timer_tic = omp_get_wtime();

	icgn1->prepare();

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //2

	//display the time of processing on screen
	cout << "Preparation of ICGN1 on GPU takes " << consumed_time << " sec." << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	icgn1->compute(poi_queue1);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //3

	//display the time of processing on screen
	cout << "ICGN1 on GPU takes " << consumed_time << " sec." << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	icgn2->prepare();

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //2

	//display the time of processing on screen
	cout << "Preparation of ICGN2 on GPU takes " << consumed_time << " sec." << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	icgn2->compute(poi_queue2);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //3

	//display the time of processing on screen
	cout << "ICGN2 on GPU takes " << consumed_time << " sec." << std::endl;

	//save the calculated dispalcements
	file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_sift_icgn1(gpu)_r16.csv";
	in_out.setPath(file_path);
	in_out.saveTable2D(poi_queue1);

	file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_sift_icgn2(gpu)_r16.csv";
	in_out.setPath(file_path);
	in_out.saveTable2D(poi_queue2);

	//destroy the instances
	delete sift;
	delete feature_affine;
	delete icgn1;
	delete icgn2;

	cout << "Press any key to exit" << std::endl;
	cin.get();

	return 0;
}