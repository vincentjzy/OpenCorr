/*
 This example demonstrates how to use OpenCorr with GPU acceleratd ICGN
 algorithms (with the 1st order shape function) to make a path independent DVC
 method. The initial guess at each POI is estimated using the FFT-CC algorithm.
*/

#include <fstream>

#include "opencorr.h"
#include "opencorr_gpu.h"

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
	int max_iteration = 10;
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

	//FFTCC
	FFTCC3D* fftcc = new FFTCC3D(subset_radius_x, subset_radius_y, subset_radius_z, cpu_thread_number);
	fftcc->setImages(ref_img, tar_img);

	//ICGN1 on CPU and GPU
	ICGN3D1* icgn1_cpu = new ICGN3D1(subset_radius_x, subset_radius_y, subset_radius_z, max_deformation_norm, max_iteration, cpu_thread_number);
	ICGN3D1GPU* icgn1_gpu = new ICGN3D1GPU(subset_radius_x, subset_radius_y, subset_radius_z, max_deformation_norm, max_iteration);

	icgn1_cpu->setImages(ref_img, tar_img);

	Img3D ref_img_gpu, tar_img_gpu;
	ref_img_gpu.dim_x = ref_img.dim_x;
	ref_img_gpu.dim_y = ref_img.dim_y;
	ref_img_gpu.dim_z = ref_img.dim_z;
	ref_img_gpu.data = &ref_img.vol_mat[0][0][0];

	tar_img_gpu.dim_x = tar_img.dim_x;
	tar_img_gpu.dim_y = tar_img.dim_y;
	tar_img_gpu.dim_z = tar_img.dim_z;
	tar_img_gpu.data = &tar_img.vol_mat[0][0][0];

	icgn1_gpu->setImages(ref_img_gpu, tar_img_gpu);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //0

	//display the time of initialization on screen
	cout << "Initialization with " << poi_queue.size() << " POIs takes " << consumed_time << " sec, " << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	fftcc->compute(poi_queue);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //1

	//display the time of processing on the screen
	cout << "Displacement estimation using FFTCC takes " << consumed_time << " sec." << std::endl;

	vector<POI3D> poi_queue1(poi_queue);
	vector<POI3D> poi_queue2(poi_queue);

	//get the time of start
	timer_tic = omp_get_wtime();

	icgn1_cpu->prepare();

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //2

	//display the time of processing on screen
	cout << "Preparation of ICGN1 on CPU takes " << consumed_time << " sec." << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	icgn1_cpu->compute(poi_queue1);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //3

	//display the time of processing on screen
	cout << "ICGN1 on CPU with " << cpu_thread_number << " CPU threads takes " << consumed_time << " sec." << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	icgn1_gpu->prepare();

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //4

	//display the time of initialization on the screen
	cout << "Preparation of ICGN1 on GPU takes " << consumed_time << " sec." << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	icgn1_gpu->compute(poi_queue2);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //5

	//display the time of initialization on the screen
	cout << "ICGN1 on GPU takes " << consumed_time << " sec." << std::endl;

	//save the calculated dispalcements
	file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_fftcc_icgn1(cpu)_r30.csv";
	in_out.setPath(file_path);
	in_out.saveTable3D(poi_queue1);

	file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_fftcc_icgn1(gpu)_r30.csv";
	in_out.setPath(file_path);
	in_out.saveTable3D(poi_queue2);

	//save the computation time
	file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_fftcc_icgn1(cpu_vs_gpu)_r30_time.csv";
	csv_out.open(file_path);
	if (csv_out.is_open())
	{
		csv_out << "POI number" << delimiter << "Initialization" << delimiter << "Initial guess estimation" << delimiter << "Preparation of ICGN1 on CPU" << delimiter << "ICGN1 on CPU" << delimiter << "Preparation of ICGN1 on GPU" << delimiter << "ICGN1 on GPU" << endl;
		csv_out << poi_queue.size() << delimiter << computation_time[0] << delimiter << computation_time[1] << delimiter << computation_time[2] << delimiter << computation_time[3] << delimiter << computation_time[4] << delimiter << computation_time[5] << endl;
	}
	csv_out.close();

	//destroy the instances
	delete fftcc;
	delete icgn1_cpu;
	delete icgn1_gpu;

	cout << "Press any key to exit..." << std::endl;
	cin.get();

	return 0;
}
