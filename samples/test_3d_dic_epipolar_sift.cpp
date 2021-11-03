/*
 This example demonstrates how to use OpenCorr to realize stereo DIC.
 In the implementation, the stereo matching between the two views uses
 epipolar constraint aided method and the ICGN algorithm with the 2nd order
 shape function, while the matching between the same view before and after
 deformation uses the SIFT feature aided method and the ICGN algorithm with
 the 1st order shape function.
*/

#include "opencorr.h"
using namespace opencorr;
using namespace std;

int main() {
	//set paths of images
	//in this example, the right image of initial state is used as the reference image
	string ref_view1_image_path = "../samples/GT4-0000_0.tif";
	string ref_view2_image_path = "../samples/GT4-0000_1.tif";
	string tar_view1_image_path = "../samples/GT4-0273_0.tif";
	string tar_view2_image_path = "../samples/GT4-0273_1.tif";

	//set path of csv file that contains the coordinates of POIs
	string file_path = "../samples/GT4-POIs.csv";
	//create an instance to read and write csv files
	IO2D in_out;
	in_out.setPath(file_path);
	in_out.setDelimiter(string(","));

	//create the instances of images
	Image2D ref_view1_img(ref_view1_image_path);
	Image2D ref_view2_img(ref_view2_image_path);
	Image2D tar_view1_img(tar_view1_image_path);
	Image2D tar_view2_img(tar_view2_image_path);

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
	cam_view1_calib.prepare(ref_view1_img.height, ref_view1_img.width);
	cam_view2_calib.prepare(ref_view2_img.height, ref_view2_img.width);
	Stereovision stereo_reconstruction(&cam_view1_calib, &cam_view2_calib, cpu_thread_number);

	//load the coordinates of POIs in principal view
	vector<Point2D> ref_view1_pt_queue = in_out.loadPOI2D();
	int queue_length = (int)ref_view1_pt_queue.size();

	//create the the queues of 2D points for stereo matching and reconstruction
	Point2D point_2d;
	vector<Point2D> ref_view2_pt_queue(queue_length, point_2d);
	vector<Point2D> tar_view1_pt_queue(queue_length, point_2d);
	vector<Point2D> tar_view2_pt_queue(queue_length, point_2d);

	//create the the queues of 3D points for stereo reconstruction
	Point3D point_3d;
	vector<Point3D> ref_pt_3d_queue(queue_length, point_3d);
	vector<Point3D> tar_pt_3d_queue(queue_length, point_3d);

	//POI for matching
	vector<POI2D> poi_queue;

	POI2D poi_2d(point_2d);
	vector<POI2D> poi_round_queue(queue_length, poi_2d);

	//POI used to store the results
	vector<POI2DS> poi_result_queue;

	//assign the coordinates to a queue of POIs
	for (int i = 0; i < (int)queue_length; i++) {
		Point2D current_point = ref_view1_pt_queue[i];

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

	//ICGN with the 1st order shape function
	ICGN2D1* icgn1 = new ICGN2D1(subset_radius_x, subset_radius_y, conv_criterion, stop_condition, cpu_thread_number);
	//ICGN with the 2nd order shape function
	ICGN2D2* icgn2 = new ICGN2D2(subset_radius_x, subset_radius_y, conv_criterion, stop_condition, cpu_thread_number);

	//create the instances for SIFT feature aided matching
	SIFT2D* sift = new SIFT2D();
	FeatureAffine2D* feature_affine = new FeatureAffine2D(subset_radius_x, subset_radius_y);

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

	//stereo matching between the two views of reference
	epipolar_search->setImages(ref_view1_img, ref_view2_img);
	epipolar_search->prepare();
	//coarse registration
	epipolar_search->compute(poi_queue);
	//refined registration
	icgn2->setImages(ref_view1_img, ref_view2_img);
	icgn2->prepare();
	icgn2->compute(poi_queue);

	//store the results of stereo matching
#pragma omp parallel for
	for (int i = 0; i < poi_queue.size(); ++i) {
		Point2D current_location(poi_queue[i].x, poi_queue[i].y);
		Point2D current_offset(poi_queue[i].deformation.u, poi_queue[i].deformation.v);
		ref_view2_pt_queue[i] = current_location + current_offset;
		poi_result_queue[i].result.r2_x = ref_view2_pt_queue[i].x;
		poi_result_queue[i].result.r2_y = ref_view2_pt_queue[i].y;
		poi_result_queue[i].result.r1r2_zncc = poi_queue[i].result.zncc;
	}

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;

	//display the time of processing on the screen
	cout << "Stereo matching between reference view1 and view2: " << consumed_time << " sec" << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	//matching between the principal view of reference and that of target
	//extract and match the SIFT features
	sift->setImages(ref_view1_img, tar_view1_img);
	sift->prepare();
	sift->compute();

	//estimate the deformation according to the SIFT features around each POIs
	feature_affine->setImages(ref_view1_img, tar_view1_img);
	feature_affine->setKeypointPair(sift->ref_matched_kp, sift->tar_matched_kp);
	feature_affine->compute(poi_queue);

	//high accuracy registration
	icgn1->setImages(ref_view1_img, tar_view1_img);
	icgn1->prepare();
	icgn1->compute(poi_queue);

	//store the results of temporal matching
#pragma omp parallel for
	for (int i = 0; i < poi_queue.size(); ++i) {
		Point2D current_location(poi_queue[i].x, poi_queue[i].y);
		Point2D current_offset(poi_queue[i].deformation.u, poi_queue[i].deformation.v);
		tar_view1_pt_queue[i] = current_location + current_offset;
		poi_result_queue[i].result.t1_x = tar_view1_pt_queue[i].x;
		poi_result_queue[i].result.t1_y = tar_view1_pt_queue[i].y;
		poi_result_queue[i].result.r1t1_zncc = poi_queue[i].result.zncc;

		poi_round_queue[i].x = round(tar_view1_pt_queue[i].x);
		poi_round_queue[i].y = round(tar_view1_pt_queue[i].y);
	}

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;

	//display the time of processing on the screen
	cout << "Temporal matching between reference view1 and target view1: " << consumed_time << " sec" << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	//matching between the principal view of reference and the secondary view of target
	//coase matching between view1 and view2 of target
	parallax_guess.x = -30.f;
	parallax_guess.y = -40.f;
	epipolar_search->setParallax(parallax_guess);
	epipolar_search->setImages(tar_view1_img, tar_view2_img);
	epipolar_search->prepare();
	epipolar_search->compute(poi_round_queue);

	//combine the dispalcements obtained by stereo matching and temperoal matching
#pragma omp parallel for
	for (int i = 0; i < poi_queue.size(); ++i) {
		poi_queue[i].deformation.u += poi_round_queue[i].deformation.u;
		poi_queue[i].deformation.v += poi_round_queue[i].deformation.v;
	}

	//stereo matching between reference view1 and target view2
	icgn2->setImages(ref_view1_img, tar_view2_img);
	icgn2->prepare();
	icgn2->compute(poi_queue);

	//store the results of matching between reference view1 and target view2
#pragma omp parallel for
	for (int i = 0; i < poi_queue.size(); ++i) {
		Point2D current_location(poi_queue[i].x, poi_queue[i].y);
		Point2D current_offset(poi_queue[i].deformation.u, poi_queue[i].deformation.v);
		tar_view2_pt_queue[i] = current_location + current_offset;
		poi_result_queue[i].result.t2_x = tar_view2_pt_queue[i].x;
		poi_result_queue[i].result.t2_y = tar_view2_pt_queue[i].y;
		poi_result_queue[i].result.r1t2_zncc = poi_queue[i].result.zncc;
	}

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;

	//display the time of processing on the screen
	cout << "Stereo matching between reference view1 and target view2: " << consumed_time << " sec" << std::endl;

	//get the time of start
	timer_tic = omp_get_wtime();

	//reconstruct the coordinates in world coordinate system
	stereo_reconstruction.prepare();
	stereo_reconstruction.reconstruct(ref_view1_pt_queue, ref_view2_pt_queue, ref_pt_3d_queue);
	stereo_reconstruction.reconstruct(tar_view1_pt_queue, tar_view2_pt_queue, tar_pt_3d_queue);

	//calculate the 3D displacements of POIs
#pragma omp parallel for
	for (int i = 0; i < poi_queue.size(); i++) {
		poi_result_queue[i].deformation.u = tar_pt_3d_queue[i].x - ref_pt_3d_queue[i].x;
		poi_result_queue[i].deformation.v = tar_pt_3d_queue[i].y - ref_pt_3d_queue[i].y;
		poi_result_queue[i].deformation.w = tar_pt_3d_queue[i].z - ref_pt_3d_queue[i].z;
	}

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;

	//display the time of processing on the screen
	cout << "Stereo reconstruction: " << consumed_time << " sec" << std::endl;


	//output the results
	in_out.setHeight(ref_view1_img.height);
	in_out.setWidth(ref_view1_img.width);

	//save the results
	file_path = tar_view1_image_path + "_epipolar_sift_table.csv";
	in_out.setPath(file_path);
	in_out.saveTable2DS(poi_result_queue);

	cout << "Press any key to exit" << std::endl;
	cin.get();

	delete icgn1;
	delete icgn2;
	delete sift;
	delete feature_affine;
	delete epipolar_search;

	return 0;
}
