/*
 This example demonstrates how to use the regional fitting strategy in OpenCorr
 to perform stereo matching of the points in two camera views. In this example,
 SIFT feature guided IC-GN algorithm is able to correctly process most points.
 The IC-GN alogorithm fails at some points due to poor initial guess. These
 points are then reprocessed using the regional fitting strategy. During the 
 reprocessing of an unreliable point, the neighboring points with highly 
 reliable results are searched and used to estimate the deformation vector, 
 which serves as the initial guess for the ICGN algorithm with the 2nd order 
 shape function. The remedial procedure can be repeated for several rounds.
*/

#include <fstream>
#include <omp.h>

#include "opencorr.h"

using namespace opencorr;
using namespace std;

int main()
{
	//set paths of images, in this example, the right image of initial state is used as the reference image
	string view1_image_path = "e:/dic_tests/3d_dic/Step18 00,00-0005_0.tif";  //replace it with the path on your computer
	string view2_image_path = "e:/dic_tests/3d_dic/Step18 00,00-0005_1.tif";  //replace it with the path on your computer

	//create the instances of images
	Image2D view1_img(view1_image_path);
	Image2D view2_img(view2_image_path);

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
	in_out.setHeight(view2_img.height);
	in_out.setWidth(view2_img.width);

	//set OpenMP parameters
	int cpu_thread_number = omp_get_num_procs() - 1;
	omp_set_num_threads(cpu_thread_number);

	//create the instances of camera parameters
	CameraIntrinsics view1_cam_intrinsics, view2_cam_intrinsics;
	CameraExtrinsics view1_cam_extrinsics, view2_cam_extrinsics;
	view1_cam_intrinsics.fx = 10664.80664f;
	view1_cam_intrinsics.fy = 10643.88965f;
	view1_cam_intrinsics.fs = 0.f;
	view1_cam_intrinsics.cx = 1176.03418f;
	view1_cam_intrinsics.cy = 914.7337036f;
	view1_cam_intrinsics.k1 = 0.030823536f;
	view1_cam_intrinsics.k2 = -1.350255132f;
	view1_cam_intrinsics.k3 = 74.21749878f;
	view1_cam_intrinsics.k4 = 0;
	view1_cam_intrinsics.k5 = 0;
	view1_cam_intrinsics.k6 = 0;
	view1_cam_intrinsics.p1 = 0;
	view1_cam_intrinsics.p2 = 0;

	view1_cam_extrinsics.tx = 0;
	view1_cam_extrinsics.ty = 0;
	view1_cam_extrinsics.tz = 0;
	view1_cam_extrinsics.rx = 0;
	view1_cam_extrinsics.ry = 0;
	view1_cam_extrinsics.rz = 0;

	view2_cam_intrinsics.fx = 10749.53223f;
	view2_cam_intrinsics.fy = 10726.52441f;
	view2_cam_intrinsics.fs = 0.f;
	view2_cam_intrinsics.cx = 1034.707886f;
	view2_cam_intrinsics.cy = 1062.162842f;
	view2_cam_intrinsics.k1 = 0.070953421f;
	view2_cam_intrinsics.k2 = -4.101067066f;
	view2_cam_intrinsics.k3 = 74.21749878f;
	view2_cam_intrinsics.k4 = 0;
	view2_cam_intrinsics.k5 = 0;
	view2_cam_intrinsics.k6 = 0;
	view2_cam_intrinsics.p1 = 0;
	view2_cam_intrinsics.p2 = 0;

	view2_cam_extrinsics.tx = 250.881488962793f;
	view2_cam_extrinsics.ty = -1.15469183120196f;
	view2_cam_extrinsics.tz = 37.4849858174401f;
	view2_cam_extrinsics.rx = 0.01450813f;
	view2_cam_extrinsics.ry = -0.39152833f;
	view2_cam_extrinsics.rz = 0.01064092f;

	//create the instances for stereovision
	Calibration cam_view1_calib(view1_cam_intrinsics, view1_cam_extrinsics);
	Calibration cam_view2_calib(view2_cam_intrinsics, view2_cam_extrinsics);
	cam_view1_calib.prepare(view1_img.height, view1_img.width);
	cam_view2_calib.prepare(view2_img.height, view2_img.width);
	Stereovision stereo_reconstruction(cam_view1_calib, cam_view2_calib, cpu_thread_number);

	//create queues of points and POIs for stereo matching and reconstruction
	vector<Point2D> view1_pt_queue; //points for stereo reconstruction
	vector<POI2D> poi_queue; //POI for matching
	vector<POI2DS> poi_result_queue; //POI used to store the results

	//set POIs
	Point2D upper_left_point(420, 250);
	int poi_number_x = 521;
	int poi_number_y = 521;
	int grid_space = 3;

	//store POIs in a queue
	for (int i = 0; i < poi_number_y; i++)
	{
		for (int j = 0; j < poi_number_x; j++)
		{
			Point2D offset(j * grid_space, i * grid_space);
			Point2D current_point = upper_left_point + offset;
			view1_pt_queue.push_back(current_point);

			POI2D current_poi(current_point);
			poi_queue.push_back(current_poi);

			POI2DS current_poi_2ds(current_point);
			poi_result_queue.push_back(current_poi_2ds);
		}
	}
	size_t queue_length = view1_pt_queue.size();

	//create a queue of 2D points for stereo matching
	Point2D point_2d;
	vector<Point2D> view2_pt_queue(queue_length, point_2d);

	//create a queue of 3D points for reconstruction
	Point3D point_3d;
	vector<Point3D> pt_3d_queue(queue_length, point_3d);

	//DIC parameter
	int subset_radius_x = 9;
	int subset_radius_y = 9;
	float conv_criterion = 0.005f;
	float stop_condition = 10.f;
	float zncc_threshold_high = 0.9f;
	float zncc_threshold_low = 0.5f;
	float search_radius_large = max(subset_radius_x, subset_radius_y) * 2.5f; //search radius for SIFT feature guided matching
	float search_radius_small = 12.f; //search radius for region fitting
	int neighbor_min = 9;
	int trial_rounds = 20;

	//create an instance of ICGN with the 2nd order shape function
	ICGN2D2* icgn2 = new ICGN2D2(subset_radius_x, subset_radius_y, conv_criterion, stop_condition, cpu_thread_number);

	//instance of RegionFit to process the POIs with ZNCC value below threshold
	RegionFit2D* region_fit = new RegionFit2D(search_radius_small, neighbor_min, cpu_thread_number);

	//create a FeatureAffine instance along with a SIFT instance
	SIFT2D* sift = new SIFT2D();
	FeatureAffine2D* feature_affine = new FeatureAffine2D(subset_radius_x, subset_radius_y, cpu_thread_number);

	//set search paramaters of FeatureAffine
	neighbor_min = 14;
	feature_affine->setSearch(search_radius_large, neighbor_min);

	//set RANSAC configuration in FeatureAffine
	RansacConfig ransac_config;
	ransac_config.trial_number = 10;
	ransac_config.sample_mumber = 5;
	ransac_config.error_threshold = 2;
	feature_affine->setRansacConfig(ransac_config);

	//assign the image pair to the instances
	icgn2->setImages(view1_img, view2_img);
	sift->setImages(view1_img, view2_img);
	feature_affine->setImages(view1_img, view2_img);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //0

	//display the time of initialization on screen
	cout << "Initialization with " << queue_length << " POIs takes " << consumed_time << " sec, " << cpu_thread_number << " CPU threads launched." << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	//SIFT feature extraction and matching
	sift->prepare();
	sift->compute();

	//assign keypoint pairs to FeatureAffine instance
	feature_affine->setKeypointPair(sift->ref_matched_kp, sift->tar_matched_kp);

	//feature aided matching
	feature_affine->prepare();
	feature_affine->compute(poi_queue);

	//refined registration
	icgn2->prepare();
	icgn2->compute(poi_queue);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //1

	//display the time of processing on the screen
	std::cout << "SIFT feature guided stereo matching takes " << consumed_time << " sec." << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	//select reliable POIs to estimate the deformation as initial guess for those unreliable POIs
	vector<POI2D> pois_reliable;
	vector<POI2D> pois_unreliable;
	vector<int> pois_unreliable_idx;
	for (int i = 0; i < queue_length; i++)
	{
		if (poi_queue[i].result.zncc < zncc_threshold_low || poi_queue[i].result.convergence>conv_criterion)
		{
			pois_unreliable.push_back(poi_queue[i]);
			pois_unreliable_idx.push_back(i);
		}
		else if(poi_queue[i].result.zncc >= zncc_threshold_high)
		{
			pois_reliable.push_back(poi_queue[i]);
		}
	}
	cout << pois_reliable.size() << " of " << queue_length << " POIs are highly reliable." << endl;

	//reprocessing using RegionFit
	for (int i = 0; i < trial_rounds; i++)
	{
		int success_counter = 0;
		region_fit->setNeighbor(pois_reliable);
		region_fit->prepare();
		region_fit->compute(pois_unreliable);
		icgn2->compute(pois_unreliable);

		//adjust the two queues of POIs
		size_t queue_size = pois_unreliable.size();
		if (queue_size > 0)
		{
			for (int j = 0; j < queue_size; j++)
			{
				if (pois_unreliable[j].result.zncc >= zncc_threshold_high && pois_unreliable[j].result.convergence <= conv_criterion)
				{
					poi_queue[pois_unreliable_idx[j]] = pois_unreliable[j];
					pois_reliable.push_back(pois_unreliable[j]);

					pois_unreliable.erase(pois_unreliable.begin() + j);
					pois_unreliable_idx.erase(pois_unreliable_idx.begin() + j);

					success_counter++;
				}
				queue_size = pois_unreliable.size();
			}
			cout << "Round" << i << " : " << success_counter << " POIs become reliable." << queue_size << " POIs remain unreliable." << endl;
			if (success_counter == 0)
			{
				break;
			}
		}
		else
		{
			cout << "Round" << i << " : " << queue_size << " POIs remain unreliable." << endl;
			break;
		}
	}

	//get the time of end
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //2

	//display the time of processing on screen
	cout << "Reprocessing with region fitting takes " << consumed_time << " sec. " << pois_unreliable.size() << " of " << queue_length << " POIs remain unreliable." << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	//store the results of stereo matching
	int counter_i = 0;
	for (int i = 0; i < queue_length; i++)
	{
		Point2D current_location(poi_queue[i].x, poi_queue[i].y);
		Point2D current_offset(poi_queue[i].deformation.u, poi_queue[i].deformation.v);
		view2_pt_queue[i] = current_location + current_offset;
		if (poi_queue[i].result.zncc >= zncc_threshold_high && poi_queue[i].result.convergence <= conv_criterion)
		{
			poi_result_queue[i].result.r2_x = view2_pt_queue[i].x;
			poi_result_queue[i].result.r2_y = view2_pt_queue[i].y;
			poi_result_queue[i].result.r1r2_zncc = poi_queue[i].result.zncc;
		}
		else
		{
			poi_result_queue[i].result.r2_x = view2_pt_queue[i].x;
			poi_result_queue[i].result.r2_y = view2_pt_queue[i].y;
			poi_result_queue[i].result.r1r2_zncc = -2.f + poi_queue[i].result.zncc;
			counter_i++;
		}
	}

	//reconstruct the coordinates in world coordinate system
	stereo_reconstruction.prepare();
	stereo_reconstruction.reconstruct(view1_pt_queue, view2_pt_queue, pt_3d_queue);

	//store the 3D coordinates for output
	for (int i = 0; i < queue_length; i++)
	{
		poi_result_queue[i].ref_coor = pt_3d_queue[i];
	}

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //3

	//display the time of processing on the screen
	std::cout << "Stereo reconstruction: " << consumed_time << " sec. POIs with low reliability: " << counter_i << " (" << counter_i * 100.f / queue_length << "%)" << std::endl;

	//save the calculated dispalcements
	file_path = view2_image_path.substr(0, view2_image_path.find_last_of(".")) + "_matching_sift_icgn2_regfit.csv";
	in_out.setPath(file_path);
	in_out.saveTable2D(poi_queue);

	file_path = view2_image_path.substr(0, view2_image_path.find_last_of(".")) + "_reconstruction_sift_icgn2_regfit.csv";
	in_out.setPath(file_path);
	in_out.saveTable2DS(poi_result_queue);

	//save the computation time
	file_path = view2_image_path.substr(0, view2_image_path.find_last_of(".")) + "_reconstruction_sift_icgn2_regfit_time.csv";
	csv_out.open(file_path);
	if (csv_out.is_open())
	{
		csv_out << "POI number" << delimiter << "Initialization" << delimiter << "Feature guided matching" << delimiter << "RegionFit" << delimiter << "reconstruction" << endl;
		csv_out << poi_queue.size() << delimiter << computation_time[0] << delimiter << computation_time[1] << delimiter << computation_time[2] << delimiter << computation_time[3] << endl;
	}
	csv_out.close();

	//destroy the instances
	delete icgn2;
	delete region_fit;
	delete sift;
	delete feature_affine;

	std::cout << "Press any key to exit..." << std::endl;
	std::cin.get();

	return 0;
}
