/*
 This example demonstrates a simple implementation of self-adaptive DIC, which
 automatically optimize the size, shape and position of subsets at each POI
 according to the nearby image features. The method invokes modules Feature and
 NearestNeighbor, and modifies slightly modules FeatureAffine and ICGN.
*/

#include <fstream>
#include <omp.h>

#include "opencorr.h"

using namespace opencorr;
using namespace std;

int main()
{
	//set files to process
	string ref_image_path = "d:/dic_tests/2d_dic/utn_00.bmp";
	string tar_image_path = "d:/dic_tests/2d_dic/utn_30.bmp";
	Image2D ref_img(ref_image_path);
	Image2D tar_img(tar_image_path);

	//create instances to read and write csv files
	string csv_file_path;
	string delimiter = ",";
	ofstream csv_out; //instance for output calculation time
	IO2D in_out; //instance for input and output DIC data
	in_out.setDelimiter(delimiter);
	in_out.setHeight(ref_img.height);
	in_out.setWidth(ref_img.width);

	//initialize papameters for timing
	double timer_tic, timer_toc, consumed_time;
	vector<double> computation_time;

	//get the time of start
	timer_tic = omp_get_wtime();

	//set DIC parameters
	int subset_radius_x = 30; //set for comparison with conventional DIC with fixed-size subset
	int subset_radius_y = 30; //set for comparison with conventional DIC with fixed-size subset
	int max_iteration = 10;
	float max_deformation_norm = 0.001f;

	//set POIs
	Point2D upper_left_point(50, 50);
	vector<POI2D> poi_queue;
	int poi_number_x = 480;
	int poi_number_y = 80;
	int grid_space = 5;

	//store POIs in a queue
	for (int i = 0; i < poi_number_y; i++) {
		for (int j = 0; j < poi_number_x; j++) {
			Point2D offset(j * grid_space, i * grid_space);
			Point2D current_point = upper_left_point + offset;
			POI2D current_poi(current_point);
			current_poi.subset_radius.x = subset_radius_x;
			current_poi.subset_radius.y = subset_radius_y;
			poi_queue.push_back(current_poi);
		}
	}

	//set OpenMP parameters
	int cpu_thread_number = omp_get_num_procs() - 1;
	omp_set_num_threads(cpu_thread_number);

	//create instances
	SIFT2D* sift = new SIFT2D();
	sift->setImages(ref_img, tar_img);

	FeatureAffine2D* feature_affine = new FeatureAffine2D(subset_radius_x, subset_radius_y, cpu_thread_number);
	int neighbor_num = 2 * feature_affine->getNeighborMin();
	int radius_min = 10;
	feature_affine->setSearch(sqrt(subset_radius_x * subset_radius_x + subset_radius_y * subset_radius_y), neighbor_num);
	feature_affine->setImages(ref_img, tar_img);
	feature_affine->setSelfAdaptive(true);
	feature_affine->setSubsetAdjustment(neighbor_num, radius_min);

	ICGN2D1* icgn1 = new ICGN2D1(subset_radius_x, subset_radius_y, max_deformation_norm, max_iteration, cpu_thread_number);
	icgn1->setImages(ref_img, tar_img);
	icgn1->setSelfAdaptive(true);

	float search_radius = 25;
	int min_neighbors = 5;
	Strain* strain = new Strain(search_radius, min_neighbors, cpu_thread_number);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //0

	//display the time of initialization on screen
	cout << "Initialization with " << poi_queue.size() << " POIs takes " << consumed_time << " sec, " << cpu_thread_number << " CPU threads launched." << std::endl;

	//get the time of start 
	timer_tic = omp_get_wtime();

	//extraction and matching of SIFT features
	sift->prepare();
	sift->compute();

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //1

	//display the time of processing on screen
	int kp_amount = (int)sift->ref_matched_kp.size();
	cout << "Extraction and matching of " << kp_amount << " SIFT features takes " << consumed_time << " sec." << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	//dynamic optimization of subset at each POI based on modified FeatureAffine
	feature_affine->setKeypointPair(sift->ref_matched_kp, sift->tar_matched_kp);
	feature_affine->prepare();
	feature_affine->compute(poi_queue);

	//conventional DIC method with fixed-size subset
	//feature_affine->compute(poi_queue);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //2

	//display the time of initialization on the screen
	cout << "Subset optimization and deformation estimation at " << poi_queue.size() << " POIs takes " << consumed_time << " sec" << std::endl;

	//get the time of start 
	timer_tic = omp_get_wtime();

	//ICGN with the 2nd order shape fucntion
	icgn1->prepare();
	icgn1->compute(poi_queue);

	//conventional DIC method with fixed-size subset
	//icgn1->compute(poi_queue);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //3

	//display the time of processing on screen
	cout << "Deformation determination using ICGN takes " << consumed_time << " sec." << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	//calculate strain
	strain->prepare(poi_queue);
	strain->compute(poi_queue);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //4

	//display the time of processing on screen
	cout << "Calculation of strain takes " << consumed_time << " sec" << std::endl;

	//save the calculated dispalcements
	csv_file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_self_adaptive.csv";
	//csv_file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_sift_icgn1_r30.csv";
	in_out.setPath(csv_file_path);
	in_out.saveTable2D(poi_queue);

	//save the computation time
	csv_file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_self_adaptive_time.csv";
	//csv_file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_sift_icgn1_r30_time.csv";
	csv_out.open(csv_file_path);
	if (csv_out.is_open())
	{
		csv_out << "POI number" << delimiter << "Initialization" << delimiter << "Feature extraction and matching" << delimiter << "Subset optimization" << delimiter << "ICGN" << delimiter << "Strain" << endl;
		csv_out << poi_queue.size() << delimiter << computation_time[0] << delimiter << computation_time[1] << delimiter << computation_time[2] << delimiter << computation_time[3] << delimiter << computation_time[4] << endl;
	}
	csv_out.close();

	cout << "Press any key to exit" << std::endl;
	cin.get();

	delete sift;
	delete feature_affine;
	delete icgn1;
	delete strain;

	return 0;
}