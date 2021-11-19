/*
 This example demonstrates how to use OpenCorr to perform stereo matching of
 points in two camera views, using the epipolar constraint aided method and
 the ICGN algorithm with the 2nd order shape function.
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

	//get time of start
	double timer_tic = omp_get_wtime();

	//Initialization of data structure and parameters
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

	//set POIs
	Point2D upper_left_point(420, 250);
	vector<Point2D> view1_pt_queue;
	vector<POI2D> poi_queue; //POI for matching
	vector<POI2DS> poi_result_queue; //POI used to store the results

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

	//create the the queues of 2D points for stereo matching and reconstruction
	int queue_length = (int)view1_pt_queue.size();
	Point2D point_2d;
	vector<Point2D> view2_pt_queue(queue_length, point_2d);

	//create the the queues of 3D points for stereo reconstruction
	Point3D point_3d;
	vector<Point3D> pt_3d_queue(queue_length, point_3d);

	//create the instances of ICGN
	int subset_radius_x = 9;
	int subset_radius_y = 9;
	float conv_criterion = 0.001f;
	int stop_condition = 10;

	//ICGN with the 2nd order shape function
	ICGN2D2* icgn2 = new ICGN2D2(subset_radius_x, subset_radius_y, conv_criterion, stop_condition, cpu_thread_number);

	//create an instance for epipolar constraint aided matching
	EpipolarSearch* epipolar_search = new EpipolarSearch(cam_view1_calib, cam_view2_calib, cpu_thread_number);

	//set search parameters in epipolar constraint aided matching
	Point2D parallax_guess(-30, -40);
	epipolar_search->setParallax(parallax_guess);
	int search_radius = 150;
	int search_step = 4;
	epipolar_search->setSearch(search_radius, search_step);

	//initialize an ICGN2D1 instance in epipolar constraint aided matching
	subset_radius_x = 15;
	subset_radius_y = 15;
	conv_criterion = 0.05f;
	stop_condition = 5;
	epipolar_search->setICGN(subset_radius_x, subset_radius_y, conv_criterion, stop_condition);

	//get the time of end 
	double timer_toc = omp_get_wtime();
	double consumed_time = timer_toc - timer_tic;

	//display the time of initialization on the screen
	cout << "Initialization: " << consumed_time << " sec" << std::endl;


	//get the time of start
	timer_tic = omp_get_wtime();

	//stereo matching between the two views
	epipolar_search->setImages(view1_img, view2_img);
	epipolar_search->prepare();
	//coarse registration
	epipolar_search->compute(poi_queue);
	//refined registration
	icgn2->setImages(view1_img, view2_img);
	icgn2->prepare();
	icgn2->compute(poi_queue);

	//store the results of stereo matching
#pragma omp parallel for
	for (int i = 0; i < poi_queue.size(); ++i) {
		Point2D current_location(poi_queue[i].x, poi_queue[i].y);
		Point2D current_offset(poi_queue[i].deformation.u, poi_queue[i].deformation.v);
		view2_pt_queue[i] = current_location + current_offset;
		poi_result_queue[i].result.r2_x = view2_pt_queue[i].x;
		poi_result_queue[i].result.r2_y = view2_pt_queue[i].y;
		poi_result_queue[i].result.r1r2_zncc = poi_queue[i].result.zncc;
	}


	//reconstruct the coordinates in world coordinate system
	stereo_reconstruction.prepare();
	stereo_reconstruction.reconstruct(view1_pt_queue, view2_pt_queue, pt_3d_queue);

	//store the 3D coordinates for output
#pragma omp parallel for
	for (int i = 0; i < poi_queue.size(); i++) {
		poi_result_queue[i].ref_coor.x = pt_3d_queue[i].x;
		poi_result_queue[i].ref_coor.y = pt_3d_queue[i].y;
		poi_result_queue[i].ref_coor.z = pt_3d_queue[i].z;
	}

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;

	//display the time of processing on the screen
	cout << "Stereo matching and reconstruction: " << consumed_time << " sec" << std::endl;


	//create an instance to read and write csv files
	IO2D in_out;
	in_out.setDelimiter(string(","));

	//output the results
	in_out.setHeight(view1_img.height);
	in_out.setWidth(view1_img.width);

	//save the calculated dispalcements
	string file_path = view2_image_path + "_reconstruction_table.csv";
	in_out.setPath(file_path);
	in_out.saveTable2DS(poi_result_queue);

	cout << "Press any key to exit" << std::endl;
	cin.get();

	delete icgn2;
	delete epipolar_search;

	return 0;
}

