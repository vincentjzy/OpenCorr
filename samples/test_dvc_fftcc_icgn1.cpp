/*
 This example demonstrates how to use OpenCorr to realize the path independent
 DVC based on FFT-CC algorithm and ICGN algorithm (with the 1st order shape
 function).
*/

#include "opencorr.h"

using namespace opencorr;
using namespace std;

int main() {
	//set files to process
	string ref_image_path = "../samples/al_foam4_0.bin";
	string tar_image_path = "../samples/al_foam4_1.bin";
	Image3D ref_img(ref_image_path);
	Image3D tar_img(tar_image_path);

	//get time of start
	double timer_tic = omp_get_wtime();

	//Initialization of data structure and parameters
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

	int queue_length = poi_number_x * poi_number_y * poi_number_z;

	//store POIs in a queue
	for (int i = 0; i < poi_number_z; i++) {
		for (int j = 0; j < poi_number_y; j++) {
			for (int k = 0; k < poi_number_x; k++) {
				Point3D offset(k * grid_space, j * grid_space, i * grid_space);
				Point3D current_point = upper_left_point + offset;
				POI3D current_poi(current_point);
				poi_queue.push_back(current_poi);
			}
		}
	}

	//get the time of end 
	double timer_toc = omp_get_wtime();
	double consumed_time = timer_toc - timer_tic;

	//display the time of initialization on the screen
	cout << "Initialization: " << consumed_time << " sec" << std::endl;

	//get time of start
	timer_tic = omp_get_wtime();

	//FFTCC
	FFTCC3D* fftcc = new FFTCC3D(subset_radius_x, subset_radius_y, subset_radius_z, cpu_thread_number);
	fftcc->setImages(ref_img, tar_img);
	fftcc->compute(poi_queue);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;

	//display the time of FFT-CC on the screen
	cout << "FFT-CC: " << consumed_time << " sec" << std::endl;

	//get time of start
	timer_tic = omp_get_wtime();

	//ICGN with the 1st order shape function
	ICGN3D1* icgn1 = new ICGN3D1(subset_radius_x, subset_radius_y, subset_radius_z, max_deformation_norm, max_iteration, cpu_thread_number);
	icgn1->setImages(ref_img, tar_img);
	icgn1->prepare();
	icgn1->compute(poi_queue);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;

	//display the time of processing on the screen
	cout << "ICGN: " << consumed_time << " sec" << std::endl;

	//output the results
	IO3D results_output;
	string file_path;
	results_output.setDimX(ref_img.dim_x);
	results_output.setDimY(ref_img.dim_y);
	results_output.setDimZ(ref_img.dim_z);
	results_output.setDelimiter(string(","));

	//save the calculated results
	file_path = tar_image_path + "_fftcc_icgn1_table_r30.csv";
	results_output.setPath(file_path);
	results_output.saveTable3D(poi_queue);

	//save the calculated results in a binary file
	file_path = tar_image_path + "_fftcc_icgn1_matrix_r30.bin";
	results_output.setPath(file_path);
	results_output.saveMatrixBin(poi_queue);

	delete icgn1;
	delete fftcc;

	cout << "Press any key to exit" << std::endl;
	cin.get();

	return 0;
}
