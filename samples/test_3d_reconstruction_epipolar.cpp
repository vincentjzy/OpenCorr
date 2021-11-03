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
	string view1_image_path = "../samples/GT4-0273_0.tif";
	string view2_image_path = "../samples/GT4-0273_1.tif";

	//create the instances of images
	Image2D view1_img(view1_image_path);
	Image2D view2_img(view2_image_path);

	//set path of csv file that contains the coordinates of POIs
	string file_path = "../samples/GT4-POIs.csv";
	//create an instance to read and write csv files
	IO2D in_out;
	in_out.setPath(file_path);
	in_out.setDelimiter(string(","));

	//get time of start
	double timer_tic = omp_get_wtime();

	//Initialization of data structure and parameters
	//set OpenMP parameters
	int cpu_thread_number = omp_get_num_procs() - 1;
	omp_set_num_threads(cpu_thread_number);

	//create the instances of camera parameters
	CameraIntrinsics view1_cam_intrisics, view2_cam_intrisics;
	CameraExtrinsics view1_cam_extrisics, view2_cam_extrisics;
	view1_cam_intrisics.fx = 6673.3159f;
	view1_cam_intrisics.fy = 6669.3027f;
	view1_cam_intrisics.fs = 0.f;
	view1_cam_intrisics.cx = 872.15778f;
	view1_cam_intrisics.cy = 579.95532f;
	view1_cam_intrisics.k1 = 0.03225895f;
	view1_cam_intrisics.k2 = -1.0114142f;
	view1_cam_intrisics.k3 = 29.788389f;
	view1_cam_intrisics.k4 = 0;
	view1_cam_intrisics.k5 = 0;
	view1_cam_intrisics.k6 = 0;
	view1_cam_intrisics.p1 = 0;
	view1_cam_intrisics.p2 = 0;

	view1_cam_extrisics.tx = 0;
	view1_cam_extrisics.ty = 0;
	view1_cam_extrisics.tz = 0;
	view1_cam_extrisics.rx = 0;
	view1_cam_extrisics.ry = 0;
	view1_cam_extrisics.rz = 0;

	view2_cam_intrisics.fx = 6607.6182f;
	view2_cam_intrisics.fy = 6602.8574f;
	view2_cam_intrisics.fs = 0.f;
	view2_cam_intrisics.cx = 917.97339f;
	view2_cam_intrisics.cy = 531.63525f;
	view2_cam_intrisics.k1 = 0.06459849f;
	view2_cam_intrisics.k2 = -4.531374f;
	view2_cam_intrisics.k3 = 29.788389f;
	view2_cam_intrisics.k4 = 0;
	view2_cam_intrisics.k5 = 0;
	view2_cam_intrisics.k6 = 0;
	view2_cam_intrisics.p1 = 0;
	view2_cam_intrisics.p2 = 0;

	view2_cam_extrisics.tx = 122.24886f;
	view2_cam_extrisics.ty = 1.8488892f;
	view2_cam_extrisics.tz = 17.624638f;
	view2_cam_extrisics.rx = 0.00307711f;
	view2_cam_extrisics.ry = -0.33278773f;
	view2_cam_extrisics.rz = 0.00524556f;

	//create the instances for stereovision
	Calibration cam_view1_calib(view1_cam_intrisics, view1_cam_extrisics);
	Calibration cam_view2_calib(view2_cam_intrisics, view2_cam_extrisics);
	cam_view1_calib.prepare(view1_img.height, view1_img.width);
	cam_view2_calib.prepare(view2_img.height, view2_img.width);
	Stereovision stereo_reconstruction(&cam_view1_calib, &cam_view2_calib, cpu_thread_number);

	//load the coordinates of POIs in principal view
	vector<Point2D> view1_pt_queue = in_out.loadPOI2D();
	int queue_length = (int)view1_pt_queue.size();

	//create the the queues of 2D points for stereo matching and reconstruction
	Point2D point_2d;
	vector<Point2D> view2_pt_queue(queue_length, point_2d);

	//create the the queues of 3D points for stereo reconstruction
	Point3D point_3d;
	vector<Point3D> pt_3d_queue(queue_length, point_3d);
	//assign the coordinates to a queue of POIs

	//POI for matching
	vector<POI2D> poi_queue;
	//POI used to store the results
	vector<POI2DS> poi_result_queue;

	//assign the queues of POIs with coordinates
	for (int i = 0; i < queue_length; i++) {
		Point2D current_point = view1_pt_queue[i];

		POI2D current_POI(current_point);
		poi_queue.push_back(current_POI);

		POI2DS current_poi_2ds(current_point);
		poi_result_queue.push_back(current_poi_2ds);
	}

	//create the instances of ICGN
	int subset_radius_x = 16;
	int subset_radius_y = 16;
	float conv_criterion = 0.001f;
	int stop_condition = 10;

	//ICGN with the 2nd order shape function
	ICGN2D2* icgn2 = new ICGN2D2(subset_radius_x, subset_radius_y, conv_criterion, stop_condition, cpu_thread_number);

	//create an instance for epipolar constraint aided matching
	EpipolarSearch* epipolar_search = new EpipolarSearch(cam_view1_calib, cam_view2_calib, cpu_thread_number);

	//set search parameters in epipolar constraint aided matching
	Point2D parallax_guess(-30, -40);
	epipolar_search->setParallax(parallax_guess);
	int search_radius = 30;
	int search_step = 5;
	epipolar_search->setSearch(search_radius, search_step);

	//initialize an ICGN2D1 instance in epipolar constraint aided matching
	subset_radius_x = 20;
	subset_radius_y = 20;
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
		poi_result_queue[i].deformation.u = pt_3d_queue[i].x;
		poi_result_queue[i].deformation.v = pt_3d_queue[i].y;
		poi_result_queue[i].deformation.w = pt_3d_queue[i].z;
	}

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;

	//display the time of processing on the screen
	cout << "Stereo matching and reconstruction: " << consumed_time << " sec" << std::endl;


	//output the results
	in_out.setHeight(view1_img.height);
	in_out.setWidth(view1_img.width);

	//save the calculated dispalcements
	file_path = view2_image_path + "_reconstruction_table.csv";
	in_out.setPath(file_path);
	in_out.saveTable2DS(poi_result_queue);

	cout << "Press any key to exit" << std::endl;
	cin.get();

	delete icgn2;
	delete epipolar_search;

	return 0;
}

