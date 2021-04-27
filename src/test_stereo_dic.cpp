#include "opencorr.h"

using namespace opencorr;
using namespace std;

int main() {
	string ref_left_image_path = "./samples/GT4-0000_0.tif";
	string ref_right_image_path = "./samples/GT4-0000_1.tif";
	string tar_left_image_path = "./samples/GT4-0066_0.tif";
	string tar_right_image_path = "./samples/GT4-0066_1.tif";

	double timer_tic = omp_get_wtime();

	Image2D ref_img(ref_left_image_path);
	Image2D tar_img(ref_right_image_path);

	Point2D ref_left_coordinate(697, 561);//corresponds to the location in right view near (673,517)
	POI2D left_POI(ref_left_coordinate);

	//set intrinsic and extrinsic parameters of cameras
	CameraIntrinsics left_cam_intrinsics, right_cam_intrinsics;
	CameraExtrinsics left_cam_extrinsics, right_cam_extrinsics;
	left_cam_intrinsics.fx = 6673.3159f;
	left_cam_intrinsics.fy = 6669.3027f;
	left_cam_intrinsics.fs = 0.f;
	left_cam_intrinsics.cx = 872.15778f;
	left_cam_intrinsics.cy = 579.95532f;
	left_cam_intrinsics.k1 = 0.03225895f;
	left_cam_intrinsics.k2 = -1.0114142f;
	left_cam_intrinsics.k3 = 29.788389f;
	left_cam_intrinsics.k4 = 0;
	left_cam_intrinsics.k5 = 0;
	left_cam_intrinsics.k6 = 0;
	left_cam_intrinsics.p1 = 0;
	left_cam_intrinsics.p2 = 0;

	left_cam_extrinsics.tx = 0;
	left_cam_extrinsics.ty = 0;
	left_cam_extrinsics.tz = 0;
	left_cam_extrinsics.pitch = 0;
	left_cam_extrinsics.roll = 0;
	left_cam_extrinsics.yaw = 0;

	right_cam_intrinsics.fx = 6607.6182f;
	right_cam_intrinsics.fy = 6602.8574f;
	right_cam_intrinsics.fs = 0.f;
	right_cam_intrinsics.cx = 917.97339f;
	right_cam_intrinsics.cy = 531.63525f;
	right_cam_intrinsics.k1 = 0.06459849f;
	right_cam_intrinsics.k2 = -4.531374f;
	right_cam_intrinsics.k3 = 29.788389f;
	right_cam_intrinsics.k4 = 0;
	right_cam_intrinsics.k5 = 0;
	right_cam_intrinsics.k6 = 0;
	right_cam_intrinsics.p1 = 0;
	right_cam_intrinsics.p2 = 0;

	right_cam_extrinsics.tx = 122.24886f;
	right_cam_extrinsics.ty = 1.8488892f;
	right_cam_extrinsics.tz = 17.624638f;
	right_cam_extrinsics.pitch = 0.00307711f;
	right_cam_extrinsics.roll = -0.33278773f;
	right_cam_extrinsics.yaw = 0.00524556f;

	//create the objects of camera calibration
	Calibration left_cam_calib(left_cam_intrinsics, left_cam_extrinsics);
	Calibration right_cam_calib(right_cam_intrinsics, right_cam_extrinsics);

	//create the object of stereo DIC
	Stereovision stereo_dic(left_cam_calib, right_cam_calib);
	stereo_dic.setImages(ref_img, tar_img);

	//setup search along epipolar for corresponding POI
	int search_radius = 50;
	int search_step = 5;
	stereo_dic.setSearch(search_radius, search_step);

	//setup coarse registration
	int subset_radius_x = 20;
	int subset_radius_y = 20;
	float convergence_criterion = 0.05f;
	float stop_condition = 5;
	stereo_dic.icgn1->setSubsetRadii(subset_radius_x, subset_radius_y);
	stereo_dic.icgn1->setIteration(convergence_criterion, stop_condition);

	//setup refined registration
	subset_radius_x = 16;
	subset_radius_y = 16;
	convergence_criterion = 0.001f;
	stop_condition = 10;
	stereo_dic.icgn2->setSubsetRadii(subset_radius_x, subset_radius_y);
	stereo_dic.icgn2->setIteration(convergence_criterion, stop_condition);

	//prepare matrices for reconstruct
	stereo_dic.prepare();
	stereo_dic.fundementalMatrix();

	//search corresponding POI in right view
	left_POI.clean();
	Point2D ref_right_coordinate = stereo_dic.epoipolarMatch(left_POI);

	//estimate the location in world coordinate system
	Point3D ref_world_coordinate = stereo_dic.reconstruct(ref_left_coordinate, ref_right_coordinate);

	//setup parameters of dic between left view before and after deformation
	stereo_dic.icgn1->setSubsetRadii(subset_radius_x, subset_radius_y);
	stereo_dic.icgn1->setIteration(convergence_criterion, stop_condition);

	SIFT2D* sift = new SIFT2D();
	FeatureAffine2D* feature_affine = new FeatureAffine2D(subset_radius_x, subset_radius_y);
	tar_img.load(tar_left_image_path);

	//extract and match features
	sift->setImages(ref_img, tar_img);
	sift->compute();

	//estimate deformation according the neighbor features
	float neighbor_search_radius = sqrtf(float(subset_radius_x * subset_radius_x + subset_radius_y * subset_radius_y));
	int essential_neighbors = 14;
	RANSACconfig ransac;
	ransac.error_threshold = 1.5f;
	ransac.sample_mumber = 5;
	ransac.trial_number = 10;

	feature_affine->setImages(ref_img, tar_img);
	feature_affine->setKeypointPair(sift->ref_matched_keypoints, sift->tar_matched_keypoints);
	feature_affine->setSearchParameters(neighbor_search_radius, essential_neighbors);
	feature_affine->setRANSAC(ransac);
	feature_affine->compute(&left_POI);

	//accurate registration
	stereo_dic.icgn1->compute(&left_POI);

	//search corresponding POI in right view
	Point2D tar_left_coordinate(left_POI.x + left_POI.deformation.u, left_POI.y + left_POI.deformation.v);

	tar_img.load(tar_right_image_path);
	stereo_dic.setImages(ref_img, tar_img);
	subset_radius_x = 20;
	subset_radius_y = 20;
	convergence_criterion = 0.05f;
	stop_condition = 5;
	stereo_dic.icgn1->setSubsetRadii(subset_radius_x, subset_radius_y);
	stereo_dic.icgn1->setIteration(convergence_criterion, stop_condition);

	subset_radius_x = 16;
	subset_radius_y = 16;
	convergence_criterion = 0.001f;
	stop_condition = 10;
	stereo_dic.icgn2->setSubsetRadii(subset_radius_x, subset_radius_y);
	stereo_dic.icgn2->setIteration(convergence_criterion, stop_condition);

	//recontruct 3D coordinate of the POI
	Point2D tar_right_coordinate = stereo_dic.epoipolarMatch(left_POI);
	Point3D tar_world_coordinate = stereo_dic.reconstruct(tar_left_coordinate, tar_right_coordinate);

	//calculate 3D displacement of the POI
	Point3D point_vector = tar_world_coordinate - ref_world_coordinate;
	Deformation3D1 deformation_3d;
	deformation_3d.u = point_vector.x;
	deformation_3d.v = point_vector.y;
	deformation_3d.w = point_vector.z;

	double timer_toc = omp_get_wtime();
	double consumed_time = timer_toc - timer_tic;

	cout << "ref_left:" << ref_left_coordinate << endl;
	cout << "ref_right:" << ref_right_coordinate << endl;
	cout << "ref_left:" << tar_left_coordinate << endl;
	cout << "ref_right:" << tar_right_coordinate << endl;
	cout << "displacement = (" << deformation_3d.u << ", " << deformation_3d.v << ", " << deformation_3d.w << ")" << endl;
	cout << consumed_time << " sec";
	cin.get();

	//delete objects and release memory
	delete sift;
	delete feature_affine;
	return 0;
}