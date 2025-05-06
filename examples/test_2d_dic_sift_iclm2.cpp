/*
 This example demonstrates how to use OpenCorr to realize a path-independent
 DIC method based on the SIFT feature guided deformation estimation and the
 IC-LM algorithm (with the 2nd order shape function). In FeatureAffine module,
 the searching of neighbor keypoints around a POI combines the FLANN and brute
 force searching.
*/

#include <fstream>
#include <omp.h>

#include "opencorr.h"

using namespace opencorr;
using namespace std;

int main()
{
	//set files to process
	string ref_image_path = "d:/dic_tests/2d_dic/rotation_000.tif"; //replace it with the path on your computer
	string tar_image_path = "d:/dic_tests/2d_dic/rotation_170.tif"; //replace it with the path on your computer
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

	//set OpenMP parameters
	int cpu_thread_number = omp_get_num_procs() - 1;
	omp_set_num_threads(cpu_thread_number);

	//set DIC parameters
	int subset_radius_x = 12;
	int subset_radius_y = 12;
	int max_iteration = 10;
	float max_deformation_norm = 0.001f;
	float lambda = 10.f;
	float alpha = 0.1f;
	float beta = 10.f;

	//create instances of SIFT and FeatureAffine for initial guess esitmation
	SIFT2D* sift = new SIFT2D();
	sift->setImages(ref_img, tar_img);
	FeatureAffine2D* feature_affine = new FeatureAffine2D(subset_radius_x, subset_radius_y, cpu_thread_number);
	feature_affine->setImages(ref_img, tar_img);

	//set POIs
	Point2D upper_left_point(50, 50);
	vector<POI2D> poi_queue;
	int poi_number_x = 205;
	int poi_number_y = 205;
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

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //0

	//display the time of initialization on screen
	cout << "Initialization with " << poi_queue.size() << " POIs takes " << consumed_time << " sec, " << cpu_thread_number << " CPU threads launched." << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	//Extraction and matching of SIFT features
	sift->prepare();
	sift->compute();

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //1

	//display the time of processing on screen
	int kp_amount = (int)sift->ref_matched_kp.size();
	cout << "Extraction and matching of " << kp_amount << " SIFT features takes " << consumed_time << " sec." << std::endl;

	//save the coordinates of matched keypoints
	file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_matched_kp.csv";
	csv_out.open(file_path);
	if (csv_out.is_open())
	{
		csv_out << "x_ref" << delimiter << "y_ref" << delimiter << "x_tar" << delimiter << "y_tar" << endl;
		for (int i = 0; i < kp_amount; i++)
		{
			csv_out << sift->ref_matched_kp[i] << delimiter << sift->tar_matched_kp[i] << endl;
		}
	}
	csv_out.close();

	//get the time of start
	timer_tic = omp_get_wtime();

	//FeatureAffine instance estimates the deformation at POI according the neighbor features
	feature_affine->setKeypointPair(sift->ref_matched_kp, sift->tar_matched_kp);
	feature_affine->prepare();
	feature_affine->compute(poi_queue);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //2

	//display the time of processing on screen
	cout << "SIFT feature guided deformation estimation takes " << consumed_time << " sec." << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	//ICLM with the 2nd order shape fucntion
	ICLM2D2* iclm2 = new ICLM2D2(subset_radius_x, subset_radius_y, max_deformation_norm, max_iteration, cpu_thread_number);
	iclm2->setImages(ref_img, tar_img);
	iclm2->setDamping(lambda, alpha, beta);
	iclm2->prepare();
	iclm2->compute(poi_queue);

	//get the time of end
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //3

	//display the time of processing on screen
	cout << "Deformation determination using ICGN takes " << consumed_time << " sec." << std::endl;

	//save the calculated dispalcements
	file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_sift_iclm2_r12.csv";
	in_out.setPath(file_path);
	in_out.saveTable2D(poi_queue);

	//save the computation time
	file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_sift_iclm2_r12_time.csv";
	csv_out.open(file_path);
	if (csv_out.is_open())
	{
		csv_out << "POI number" << delimiter << "Initialization" << delimiter << "SIFT" << delimiter << "FeatureAffine" << delimiter << "ICLM" << endl;
		csv_out << poi_queue.size() << delimiter << computation_time[0] << delimiter << computation_time[1] << delimiter << computation_time[2] << delimiter << computation_time[3] << endl;
	}
	csv_out.close();

	//destroy the instances
	delete sift;
	delete feature_affine;
	delete iclm2;

	cout << "Press any key to exit" << std::endl;
	cin.get();

	return 0;
}
