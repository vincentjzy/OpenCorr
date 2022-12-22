/*
 This example demonstrates how to use OpenCorr to calculate the stains
 according to the polynomial fit fields of displacements u, v, and w obtained
 through 3D/stereo DIC methods. The searching of neighbor POIs around a POI 
 combines the FLANN and brute force searching.
*/

#include <fstream>

#include "opencorr.h"

using namespace opencorr;
using namespace std;

int main()
{
	//select the image file to get the file name to process
	string tar_image_path = "d:/dic_tests/3d_dic/GT4-0273_0.tif"; //replace it with the path on your computer
	Image2D tar_img(tar_image_path);

	//initialize papameters for timing
	double timer_tic, timer_toc, consumed_time;
	vector<double> computation_time;

	//get the time of start
	timer_tic = omp_get_wtime();

	//get the DIC results from csv file
	string file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_epipolar_sift_r16.csv";
	string delimiter = ",";
	ofstream csv_out; //instance for output calculation time
	IO2D in_out; //instance for input and output DIC data
	in_out.setPath(file_path);
	in_out.setHeight(tar_img.height);
	in_out.setWidth(tar_img.width);
	in_out.setDelimiter(delimiter);

	//load a queue of POIs
	vector<POI2DS> poi_queue = in_out.loadTable2DS();

	//set OpenMP parameters
	int cpu_thread_number = omp_get_num_procs() - 1;
	omp_set_num_threads(cpu_thread_number);

	//set the radius of subregion for polynomial fit of displacement field
	float strain_radius = 20.f;

	//set the miminum number of neighbor POIs to perform fitting
	int min_neighbors = 5;

	//initialize a object of stain calculation
	Strain* strain = new Strain(strain_radius, min_neighbors, cpu_thread_number);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //0

	//display the time of initialization on screen
	cout << "Initialization takes " << consumed_time << " sec, " << cpu_thread_number << " CPU threads launched." << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	//calculate the strain exx, eyy, ezz, exy, eyz, ezx
	strain->prepare(poi_queue);
	strain->compute(poi_queue);

	//get time of end
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //1

	cout << "Strain calculation of " << poi_queue.size() << " POIs takes " << consumed_time << " sec." << std::endl;

	//update the table of DIC results with calculated strains
	in_out.saveTable2DS(poi_queue);

	//save the computation time
	file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_strain_r20_time.csv";
	csv_out.open(file_path);
	if (csv_out.is_open())
	{
		csv_out << "POI number" << delimiter << "Initialization" << delimiter << "Strain calculation" << endl;
		csv_out << poi_queue.size() << delimiter << computation_time[0] << delimiter << computation_time[1] << endl;
	}
	csv_out.close();

	//destroy the instance
	delete strain;

	cout << "Press any key to exit..." << std::endl;
	cin.get();

	return 0;
}