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
using namespace opencorr_gpu;
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

	//create instances of SIFT and FeatureAffine for initial guess estimation
	SIFT2D* sift = new SIFT2D();
	sift->setImages(ref_img, tar_img);
	FeatureAffine2D* feature_affine = new FeatureAffine2D(subset_radius_x, subset_radius_y, cpu_thread_number);
	feature_affine->setImages(ref_img, tar_img);

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

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //0

	//display the time of initialization on screen
	cout << "Initialization with " << poi_queue.size() << " POIs takes " << consumed_time << " sec, " << cpu_thread_number << " CPU threads launched." << std::endl;

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

	//get the time of start
	timer_tic = omp_get_wtime();

	//initialize the reference image and target image for GPU accelerated processing
	ICGNImage gpu_ref_img;
	gpu_ref_img.w = ref_img.width;
	gpu_ref_img.h = ref_img.height;
	gpu_ref_img.data = new float[ref_img.width * ref_img.height];

	ICGNImage gpu_tar_img;
	gpu_tar_img.w = tar_img.width;
	gpu_tar_img.h = tar_img.height;
	gpu_tar_img.data = new float[ref_img.width * ref_img.height];

	//fill the graysclae into the 1D arrays
	for (int r = 0; r < ref_img.height; r++)
	{
		for (int c = 0; c < ref_img.width; c++)
		{
			gpu_ref_img.data[r * ref_img.width + c] = (float)ref_img.eg_mat(r, c);
			gpu_tar_img.data[r * tar_img.width + c] = (float)tar_img.eg_mat(r, c);
		}
	}

	//initialize a POI queue for GPU accelerated ICGN
	vector<ICGNPOI> gpu_POI_queue;
	for (vector<POI2D>::iterator iter = poi_queue.begin(); iter != poi_queue.end(); iter++)
	{
		ICGNPOI gpu_POI(iter->x, iter->y);
		gpu_POI.initial.u = iter->deformation.u;
		gpu_POI.initial.ux = iter->deformation.ux;
		gpu_POI.initial.uy = iter->deformation.uy;
		gpu_POI.initial.v = iter->deformation.v;
		gpu_POI.initial.vx = iter->deformation.vx;
		gpu_POI.initial.vy = iter->deformation.vy;
		gpu_POI_queue.push_back(gpu_POI);

		//Save the initial guess
		iter->result.u0 = iter->deformation.u;
		iter->result.v0 = iter->deformation.v;
	}

	//choose the order of shape function
	int shape_order = 2; // 1 for the 1st order, 2 for the 2nd order

	//set ICGN parameters for GPU accelerated computation
	ICGNConfiguration icgn_conf;
	icgn_conf.subset_rx = subset_radius_x;
	icgn_conf.subset_ry = subset_radius_y;
	icgn_conf.convergence_criterion = max_deformation_norm;
	icgn_conf.stop_condtion = max_iteration;

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //2

	//display the time of initialization on the screen
	cout << "Preparation of GPU module takes " << consumed_time << " sec." << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	//run GPU accelerated ICGN
	if (shape_order == 1)
	{
		if (!ICGN2D1GPU(gpu_ref_img, gpu_tar_img, gpu_POI_queue, icgn_conf))
		{
			cerr << "ICGNbyGPU return false!" << endl;
			return -1;
		}
	}
	if (shape_order == 2)
	{
		if (!ICGN2D2GPU(gpu_ref_img, gpu_tar_img, gpu_POI_queue, icgn_conf))
		{
			cerr << "ICGNbyGPU return false!" << endl;
			return -1;
		}
	}

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //3

	//display the time of initialization on the screen
	cout << "ICGN2 on GPU takes " << consumed_time << " sec." << std::endl;

	//transfer the results to the queue of POIs
	for (int i = 0; i < poi_queue.size(); i++)
	{
		poi_queue[i].deformation.u = gpu_POI_queue[i].final.u;
		poi_queue[i].deformation.ux = gpu_POI_queue[i].final.ux;
		poi_queue[i].deformation.uy = gpu_POI_queue[i].final.uy;
		poi_queue[i].deformation.uxx = gpu_POI_queue[i].final.uxx;
		poi_queue[i].deformation.uxy = gpu_POI_queue[i].final.uxy;
		poi_queue[i].deformation.uyy = gpu_POI_queue[i].final.uyy;
		poi_queue[i].deformation.v = gpu_POI_queue[i].final.v;
		poi_queue[i].deformation.vx = gpu_POI_queue[i].final.vx;
		poi_queue[i].deformation.vy = gpu_POI_queue[i].final.vy;
		poi_queue[i].deformation.vxx = gpu_POI_queue[i].final.vxx;
		poi_queue[i].deformation.vxy = gpu_POI_queue[i].final.vxy;
		poi_queue[i].deformation.vyy = gpu_POI_queue[i].final.vyy;

		poi_queue[i].result.zncc = gpu_POI_queue[i].ZNCC;
		poi_queue[i].result.iteration = gpu_POI_queue[i].iteration;
		poi_queue[i].result.convergence = gpu_POI_queue[i].dpNorm;
	}

	//save the calculated dispalcements
	file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_sift_icgn2(gpu)_r16.csv";
	in_out.setPath(file_path);
	in_out.saveTable2D(poi_queue);

	//save the computation time
	file_path = tar_image_path.substr(0, tar_image_path.find_last_of(".")) + "_sift_icgn2(gpu)_r16_time.csv";
	csv_out.open(file_path);
	if (csv_out.is_open())
	{
		csv_out << "POI number" << delimiter << "Initialization" << delimiter << "Initiial guess estimation" << delimiter << "Preparation of GPU module" << delimiter << "ICGN on GPU" << endl;
		csv_out << poi_queue.size() << delimiter << computation_time[0] << delimiter << computation_time[1] << delimiter << computation_time[2] << delimiter << computation_time[3] << endl;
	}
	csv_out.close();

	//destroy the instances
	delete[] gpu_ref_img.data;
	delete[] gpu_tar_img.data;
	delete feature_affine;
	delete sift;

	cout << "Press any key to exit" << std::endl;
	cin.get();

	return 0;
}