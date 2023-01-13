/*
 This example demonstrates how to use OpenCorr to realize a path-independent
 DIC method based on FFT-CC algorithm and NR algorithm (with the 1st order
 shape function).
*/

#include <fstream>

#include "opencorr.h"

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

	//set OpenMP parameters
	int cpu_thread_number = omp_get_num_procs() - 1;
	omp_set_num_threads(cpu_thread_number);

	//set DIC parameters
	int subset_radius_x = 16;
	int subset_radius_y = 16;
	int max_iteration = 10;
	float max_deformation_norm = 0.001f;

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

	//create instances
	FFTCC2D* fftcc = new FFTCC2D(subset_radius_x, subset_radius_y, cpu_thread_number);
	fftcc->setImages(ref_img, tar_img);

	NR2D1* nr1 = new NR2D1(subset_radius_x, subset_radius_y, max_deformation_norm, max_iteration, cpu_thread_number);
	nr1->setImages(ref_img, tar_img);

	float strain_radius = 20.f; //radius of subregion for polynomial fit of displacement field
	int min_neighbors = 5; //miminum number of neighbor POIs to perform fitting
	Strain* strain = new Strain(strain_radius, min_neighbors, cpu_thread_number);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //0

	//display the time of initialization on screen
	cout << "Initialization with " << poi_queue.size() << " POIs takes " << consumed_time << " sec, " << cpu_thread_number << " CPU threads launched." << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	//FFTCC
	fftcc->compute(poi_queue);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //1

	//display the time of processing on the screen
	cout << "Displacement estimation using FFTCC takes " << consumed_time << " sec." << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	//NR with the 1st order shape function
	nr1->prepare();
	nr1->compute(poi_queue);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //2

	//display the time of processing on screen
	cout << "Deformation determination using NR takes " << consumed_time << " sec." << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	//calculate strain
	strain->prepare(poi_queue);
	strain->compute(poi_queue);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //3

	//display the time of processing on screen
	cout << "Calculation of strain takes " << consumed_time << " sec" << std::endl;

	//save the calculated dispalcements
	file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_fftcc_nr1_r16.csv";
	in_out.setPath(file_path);
	in_out.saveTable2D(poi_queue);

	//save the computation time
	file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_fftcc_nr1_r16_time.csv";
	csv_out.open(file_path);
	if (csv_out.is_open())
	{
		csv_out << "POI number" << delimiter << "Initialization" << delimiter << "FFTCC" << delimiter << "NR" << delimiter << "Strain" << endl;
		csv_out << poi_queue.size() << delimiter << computation_time[0] << delimiter << computation_time[1] << delimiter << computation_time[2] << delimiter << computation_time[3] << endl;
	}
	csv_out.close();

	//destroy the instances
	delete fftcc;
	delete nr1;
	delete strain;

	cout << "Press any key to exit..." << std::endl;
	cin.get();

	return 0;
}
