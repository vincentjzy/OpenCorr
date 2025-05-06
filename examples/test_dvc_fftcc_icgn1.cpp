/*
 This example demonstrates how to use OpenCorr to realize a path-independent
 DVC method based on the FFT-CC algorithm and the ICGN algorithm (with the 1st
 order shape function).
*/

#include <fstream>
#include <omp.h>

#include "opencorr.h"

using namespace opencorr;
using namespace std;

int main()
{
	//set files to process
	string ref_image_path = "d:/dic_tests/dvc/al_foam4_0.bin"; //replace it with the path on your computer
	string tar_image_path = "d:/dic_tests/dvc/al_foam4_1.bin"; //replace it with the path on your computer
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
	ofstream csv_out; //instance for output calculation time
	IO3D in_out; //instance for input and output DIC data
	in_out.setDelimiter(delimiter);
	in_out.setDimX(ref_img.dim_x);
	in_out.setDimY(ref_img.dim_y);
	in_out.setDimZ(ref_img.dim_z);

	//set OpenMP parameters
	int cpu_thread_number = omp_get_num_procs() - 1;
	omp_set_num_threads(cpu_thread_number);

	//set DIC parameters
	int subset_radius_x = 30;
	int subset_radius_y = 30;
	int subset_radius_z = 30;
	int max_iteration = 20;
	float max_deformation_norm = 0.001f;

	//set POIs
	Point3D upper_left_point(35, 35, 60);
	vector<POI3D> poi_queue;
	int poi_number_x = 7;
	int poi_number_y = 7;
	int poi_number_z = 117;
	int grid_space = 5;

	//store POIs in a queue
	for (int i = 0; i < poi_number_z; i++)
	{
		for (int j = 0; j < poi_number_y; j++)
		{
			for (int k = 0; k < poi_number_x; k++)
			{
				Point3D offset(k * grid_space, j * grid_space, i * grid_space);
				Point3D current_point = upper_left_point + offset;
				POI3D current_poi(current_point);
				poi_queue.push_back(current_poi);
			}
		}
	}
	int queue_length = (int)poi_queue.size();

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //0

	//display the time of initialization on screen
	cout << "Initialization with " << queue_length << " POIs takes " << consumed_time << " sec, " << cpu_thread_number << " CPU threads launched." << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	//FFTCC
	FFTCC3D* fftcc = new FFTCC3D(subset_radius_x, subset_radius_y, subset_radius_z, cpu_thread_number);
	fftcc->setImages(ref_img, tar_img);
	fftcc->compute(poi_queue);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //1

	//display the time of processing on the screen
	cout << "Displacement estimation using FFTCC takes " << consumed_time << " sec." << std::endl;

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
	computation_time.push_back(consumed_time); //2

	//display the time of processing on screen
	cout << "Deformation determination using ICGN takes " << consumed_time << " sec." << std::endl;

	//save the calculated results
	file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_fftcc_icgn1_r30.csv";
	in_out.setPath(file_path);
	in_out.saveTable3D(poi_queue);

	//save the computation time
	file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_fftcc_icgn1_r30_time.csv";
	csv_out.open(file_path);
	if (csv_out.is_open())
	{
		csv_out << "POI number" << delimiter << "Initialization" << delimiter << "FFTCC" << delimiter << "ICGN" << endl;
		csv_out << poi_queue.size() << delimiter << computation_time[0] << delimiter << computation_time[1] << delimiter << computation_time[2] << endl;
	}
	csv_out.close();

	//destroy the instances
	delete fftcc;
	delete icgn1;

	cout << "Press any key to exit..." << std::endl;
	cin.get();

	return 0;
}
