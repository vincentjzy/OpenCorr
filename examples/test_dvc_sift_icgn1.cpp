/*
 This example demonstrates how to use OpenCorr to realize a path-independent
 DVC method based on the 3D SIFT feature guided deformation estimation and the
 ICGN algorithm (with the 1st order shape function).
*/

#include <fstream>
#include <omp.h>

#include "opencorr.h"

using namespace opencorr;
using namespace std;

int main()
{
	//set files to process
	string ref_image_path = "d:/dic_tests/dvc/Torus_ref.tif"; //replace it with the path on your computer
	string tar_image_path = "d:/dic_tests/dvc/Torus_def.tif"; //replace it with the path on your computer
	Image3D ref_img(ref_image_path);
	Image3D tar_img(tar_image_path);

	//initialize papameters for timing
	double timer_tic, timer_toc, consumed_time;
	vector<double> computation_time;

	//get the time of start
	timer_tic = omp_get_wtime();

	//create instances to read and write csv files
	string file_path;
	string delimiter = ",";
	ofstream csv_out; //instance for output matched keypoints and calculation time
	IO3D in_out; //instance for input and output DIC data
	in_out.setDelimiter(delimiter);
	in_out.setDimX(ref_img.dim_x);
	in_out.setDimY(ref_img.dim_y);
	in_out.setDimZ(ref_img.dim_z);

	//set path of csv file that contains the coordinates of POIs
	file_path = "d:/dic_tests/dvc/Torus_POIs.csv"; //replace it with the path on your computer
	in_out.setPath(file_path);

	//load the coordinates of POIs
	vector<Point3D> point_queue = in_out.loadPoint3D(file_path);
	int queue_length = (int)point_queue.size();

	//set OpenMP parameters
	int cpu_thread_number = omp_get_num_procs() - 1;
	omp_set_num_threads(cpu_thread_number);

	//initialize a queue of POIs
	POI3D cur_poi(0, 0, 0);
	vector<POI3D> poi_queue(queue_length, cur_poi);
#pragma omp parallel for
	for (int i = 0; i < queue_length; i++)
	{
		poi_queue[i].x = point_queue[i].x;
		poi_queue[i].y = point_queue[i].y;
		poi_queue[i].z = point_queue[i].z;
	}

	//set DIC parameters
	int subset_radius_x = 16;
	int subset_radius_y = 16;
	int subset_radius_z = 16;
	int max_iteration = 10; //used in ICGN
	float max_deformation_norm = 0.001f; //used in ICGN

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //0

	//display the time of initialization on screen
	cout << "Initialization with " << queue_length << " POIs takes " << consumed_time << " sec, " << cpu_thread_number << " CPU threads launched." << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	//SIFT extraction and matching
	SIFT3D* sift = new SIFT3D();
	sift->setImages(ref_img, tar_img);
	sift->prepare();
	sift->compute();

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //1

	//display the time of processing on screen
	int kp_amount = (int)sift->ref_matched_kp.size();
	cout << "Extraction and matching of " << kp_amount << " 3D SIFT features takes " << consumed_time << " sec." << std::endl;

	//save the coordinates of matched keypoints
	file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_matched_kp.csv";
	csv_out.open(file_path);
	if (csv_out.is_open())
	{
		csv_out << "x_ref" << delimiter << "y_ref" << delimiter << "z_ref" << delimiter << "x_tar" << delimiter << "y_tar" << delimiter << "z_tar" << endl;
		for (int i = 0; i < kp_amount; i++)
		{
			csv_out << sift->ref_matched_kp[i] << delimiter << sift->tar_matched_kp[i] << endl;
		}
	}
	csv_out.close();

	//get the time of start
	timer_tic = omp_get_wtime();

	//FeatureAffine for deformation estimation according to matched feature pairs
	FeatureAffine3D* feature_affine = new FeatureAffine3D(subset_radius_x, subset_radius_y, subset_radius_z, cpu_thread_number);
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

	//ICGN with the 1st order shape function
	ICGN3D1* icgn1 = new ICGN3D1(subset_radius_x, subset_radius_y, subset_radius_z, max_deformation_norm, max_iteration, cpu_thread_number);
	icgn1->setImages(ref_img, tar_img);
	icgn1->prepare();
	icgn1->compute(poi_queue);

	//get the time of end
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //3

	//display the time of processing on screen
	cout << "Deformation determination using ICGN takes " << consumed_time << " sec." << std::endl;

	//save the calculated results
	file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_sift_icgn1_r16.csv";
	in_out.setPath(file_path);
	in_out.saveTable3D(poi_queue);

	//save the computation time
	file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_sift_icgn1_r16_time.csv";
	csv_out.open(file_path);
	if (csv_out.is_open())
	{
		csv_out << "POI number" << delimiter << "Initialization" << delimiter << "SIFT" << delimiter << "FeatureAffine" << delimiter << "ICGN" << endl;
		csv_out << poi_queue.size() << delimiter << computation_time[0] << delimiter << computation_time[1] << delimiter << computation_time[2] << delimiter << computation_time[3] << endl;
	}
	csv_out.close();

	//destroy the instances
	delete sift;
	delete feature_affine;
	delete icgn1;

	cout << "Press any key to exit..." << std::endl;
	cin.get();

	return 0;
}
