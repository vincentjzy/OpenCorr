/*
 This example demonstrates how to use OpenCorr to perform stereo matching of
 points in two camera views, using the epipolar constraint aided method and
 SIFT feature aided method to perform coarse search. The ICGN algorithm with
 the 2nd order shape function is used for refined search.
*/

#include "opencorr.h"
using namespace opencorr;
using namespace std;

int main() {
	//set paths of images
	//in this example, the right image of initial state is used as the reference image
	string view1_image_path = "../samples/Step18 00,00-0005_0.tif";
	string view2_image_path = "../samples/Step18 00,00-0005_1.tif";

	//create the instances of images
	Image2D view1_img(view1_image_path);
	Image2D view2_img(view2_image_path);

	//Initialization of data structure and parameters------------------------------------------
	//get time of start
	double timer_tic = omp_get_wtime();

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
	Stereovision stereo_reconstruction(&cam_view1_calib, &cam_view2_calib, cpu_thread_number);

	//create queues of points and POIs for stereo matching and reconstruction
	vector<Point2D> view1_pt_queue; //points for stereo reconstruction
	vector<POI2D> poi_queue; //POI for matching
	vector<POI2DS> poi_result_queue; //POI used to store the results

	//set POIs
	Point2D upper_left_point(420, 250);
	int poi_number_x = 313;
	int poi_number_y = 313;
	int grid_space = 5;

	//store POIs in a queue
	for (int i = 0; i < poi_number_y; i++) {
		for (int j = 0; j < poi_number_x; j++) {
			Point2D offset(j * grid_space, i * grid_space);
			Point2D current_point = upper_left_point + offset;
			view1_pt_queue.push_back(current_point);

			POI2D current_poi(current_point);
			poi_queue.push_back(current_poi);

			POI2DS current_poi_2ds(current_point);
			poi_result_queue.push_back(current_poi_2ds);
		}
	}

	//create the the queues of 2D points for stereo reconstruction
	int queue_size = (int)view1_pt_queue.size();
	Point2D point_2d;
	vector<Point2D> view2_pt_queue(queue_size, point_2d);

	//create the the queues of 3D points for stereo reconstruction
	Point3D point_3d;
	vector<Point3D> pt_3d_queue(queue_size, point_3d);

	//create an instance of ICGN with the 2nd order shape function
	int subset_radius_x = 9;
	int subset_radius_y = 9;
	float conv_criterion = 0.001f;
	float stop_condition = 10;
	ICGN2D2* icgn2 = new ICGN2D2(subset_radius_x, subset_radius_y, conv_criterion, stop_condition, cpu_thread_number);

	//create an instance for epipolar constraint aided matching
	EpipolarSearch* epipolar_search = new EpipolarSearch(cam_view1_calib, cam_view2_calib, cpu_thread_number);

	//set search parameters in epipolar constraint aided matching
	int search_radius = 90;
	int search_step = 3;
	epipolar_search->setSearch(search_radius, search_step);

	//initialize an ICGN2D1 instance in epipolar constraint aided matching
	subset_radius_x = 16;
	subset_radius_y = 16;
	conv_criterion = 0.05f;
	stop_condition = 9;
	epipolar_search->createICGN(subset_radius_x, subset_radius_y, conv_criterion, stop_condition);

	//create a FeatureAffine instance along with a SIFT instance to exstimate a map of parallax between the two views
	FeatureAffine2D* feature_affine = new FeatureAffine2D(subset_radius_x, subset_radius_y);
	SIFT2D* sift = new SIFT2D();

	//set search paramaters of Feature Affine
	int search_radius_x = 16;
	int search_radius_y = 16;
	float neighbor_search_radius = sqrtf(search_radius_x * search_radius_x + search_radius_y * search_radius_y);
	int min_neighbor_number = 14;
	feature_affine->setSearchParameters(neighbor_search_radius, min_neighbor_number);

	//set RANSAC configuration in Feature Affine
	RansacConfig ransac_config;
	ransac_config.trial_number = 10;
	ransac_config.sample_mumber = 5;
	ransac_config.error_threshold = 2;
	feature_affine->setRansacConfig(ransac_config);

	//assign the image pair to the instances
	icgn2->setImages(view1_img, view2_img);
	epipolar_search->setImages(view1_img, view2_img);
	feature_affine->setImages(view1_img, view2_img);
	sift->setImages(view1_img, view2_img);

	//get the time of end 
	double timer_toc = omp_get_wtime();
	double consumed_time = timer_toc - timer_tic;

	//display the time of initialization on the screen
	std::cout << "Initialization: " << consumed_time << " sec" << std::endl;

	//the first round: SIFT feature aided matching--------------------------------------------
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

	queue_size = int(poi_queue.size());
	int counter_i = 0;
	for (int i = 0; i < queue_size; i++) {
		if (poi_queue[i].result.zncc >= 0.9f) {
			counter_i++;
		}
	}
	//display the time of processing on the screen
	std::cout << "SIFT feature aided matching: " << consumed_time << " sec" << std::endl;
	std::cout << "POIs with ZNCC >= 0.9: " << counter_i * 100.f / queue_size << "%" << std::endl;

	//the second round: epipolar constraint aided matching-------------------------------------------
	//get the time of start
	timer_tic = omp_get_wtime();

	//estimate the parallax for epipolar constraint aided matching
	queue_size = int(poi_queue.size());
	std::vector<POI2D> parallax_queue;
	std::vector<POI2D> poi_low_zncc;
	std::vector<int> poi_lz_idx;
	for (int i = 0; i < queue_size; i++) {
		if (poi_queue[i].result.zncc >= 0.998f) {
			parallax_queue.push_back(poi_queue[i]);
		}
		else if (poi_queue[i].result.zncc < 0.9f) {
			poi_queue[i].deformation.u = 0.f;
			poi_queue[i].deformation.v = 0.f;
			poi_queue[i].result.zncc = 0.f;
			poi_low_zncc.push_back(poi_queue[i]);
			poi_lz_idx.push_back(i);
		}
	}

	//convert the coordinates to the coordinate system using image center as origin
	int set_size = int(parallax_queue.size());
	std::cout << "POIs used to estimate parallax: " << set_size << " of " << queue_size << std::endl;

	Point2D img_center(int(view1_img.width / 2), int(view1_img.height / 2));
	for (int i = 0; i < set_size; i++) {
		parallax_queue[i].x -= img_center.x;
		parallax_queue[i].y -= img_center.y;
	}

	//estimate the linear regression coefficients using least square metthod
	Eigen::MatrixXf point_coor(set_size, 3); //coordinates of points
	Eigen::MatrixXf parallax(set_size, 3); //parallax along x and y axes

	for (int i = 0; i < set_size; i++) {
		point_coor(i, 0) = parallax_queue[i].x;
		point_coor(i, 1) = parallax_queue[i].y;
		point_coor(i, 2) = 1.f;

		parallax(i, 0) = parallax_queue[i].deformation.u;
		parallax(i, 1) = parallax_queue[i].deformation.v;
		parallax(i, 2) = 0.f;
	}

	Eigen::Matrix3f lr_matrix = point_coor.colPivHouseholderQr().solve(parallax);

	float lr_x[3], lr_y[3];
	lr_x[0] = lr_matrix(0, 0);
	lr_x[1] = lr_matrix(1, 0);
	lr_x[2] = lr_matrix(2, 0);

	lr_y[0] = lr_matrix(0, 1);
	lr_y[1] = lr_matrix(1, 1);
	lr_y[2] = lr_matrix(2, 1);

	epipolar_search->setParallax(lr_x, lr_y);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	std::cout << "Estimate parallax: " << consumed_time << " sec" << std::endl;
	std::cout << "Parallax_x: " << lr_x[2] << " + " << lr_x[0] << "x + " << lr_x[1] << "y" << std::endl;
	std::cout << "Parallax_y: " << lr_y[2] << " + " << lr_y[0] << "x + " << lr_y[1] << "y" << std::endl;

	//coarse registration
	epipolar_search->prepare();
	epipolar_search->compute(poi_low_zncc);

	//refined registration
	conv_criterion = 0.001f;
	stop_condition = 10;
	icgn2->setIteration(conv_criterion, stop_condition);
	icgn2->compute(poi_low_zncc);

	set_size = int(poi_lz_idx.size());
	for (int i = 0; i < set_size; i++) {
		poi_queue[poi_lz_idx[i]] = poi_low_zncc[i];
	}
	std::cout << "POIs are processed using epipolar constraint aided mathcing: " << set_size << " of " << queue_size << std::endl;

	//store the results of stereo matching
	queue_size = int(poi_queue.size());
	counter_i = 0;
	for (int i = 0; i < queue_size; ++i) {
		Point2D current_location(poi_queue[i].x, poi_queue[i].y);
		Point2D current_offset(poi_queue[i].deformation.u, poi_queue[i].deformation.v);
		view2_pt_queue[i] = current_location + current_offset;
		if (isnan(poi_queue[i].result.zncc)) {
			poi_result_queue[i].result.r2_x = 0;
			poi_result_queue[i].result.r2_y = 0;
			poi_result_queue[i].result.r1r2_zncc = -2;
			counter_i++;
		}
		else {
			poi_result_queue[i].result.r2_x = view2_pt_queue[i].x;
			poi_result_queue[i].result.r2_y = view2_pt_queue[i].y;
			poi_result_queue[i].result.r1r2_zncc = poi_queue[i].result.zncc;
			if (poi_queue[i].result.zncc < 0.9f) {
				counter_i++;
			}
		}
	}

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;

	//display the time of processing on the screen
	std::cout << "Epipolar constraint aided matching: " << consumed_time << " sec" << std::endl;
	std::cout << "POIs with ZNCC < 0.9: " << counter_i * 100.f / queue_size << "%" << std::endl;

	//stereo reconstruction based on the matched point pairs-----------------------------------------
	//get the time of start
	timer_tic = omp_get_wtime();

	//reconstruct the coordinates in world coordinate system
	stereo_reconstruction.prepare();
	stereo_reconstruction.reconstruct(view1_pt_queue, view2_pt_queue, pt_3d_queue);

	//store the 3D coordinates for output
	queue_size = int(poi_queue.size());
	for (int i = 0; i < queue_size; i++) {
		poi_result_queue[i].ref_coor.x = pt_3d_queue[i].x;
		poi_result_queue[i].ref_coor.y = pt_3d_queue[i].y;
		poi_result_queue[i].ref_coor.z = pt_3d_queue[i].z;
	}

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;

	//display the time of processing on the screen
	std::cout << "Stereo reconstruction: " << consumed_time << " sec" << std::endl;

	//create an instance to read and write csv files
	IO2D in_out;
	in_out.setDelimiter(string(","));

	//output the results
	in_out.setHeight(view1_img.height);
	in_out.setWidth(view1_img.width);

	//save the calculated dispalcements
	string file_path = view2_image_path + "_reconstruction_epipolar_sift_table.csv";
	in_out.setPath(file_path);
	in_out.saveTable2DS(poi_result_queue);

	std::cout << "Press any key to exit" << std::endl;
	std::cin.get();

	delete icgn2;
	delete epipolar_search;
	delete feature_affine;
	delete sift;

	return 0;
}
