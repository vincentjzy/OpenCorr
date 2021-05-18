/*
* This is an example demonstrating how to use OpenCorr to realize
* path-independent DIC based on SIFT-aided deformation estimation
* and ICGN algorithm (with 2nd order shape function).
*/

#include "opencorr.h"

using namespace opencorr;
using namespace std;

int main() {
	//set files to process
	string ref_image_path = "../samples/oht_cfrp_0.bmp";
	string tar_image_path = "../samples/oht_cfrp_4.bmp";
	Image2D ref_img(ref_image_path);
	Image2D tar_img(tar_image_path);

	//set DIC parameters
	int subset_radius_x = 16;
	int subset_radius_y = 16;
	int max_iteration = 10;
	float max_deformation_norm = 0.001f;

	//set POIs
	Point2D upper_left_point(30, 30);
	vector<POI2D> POI_queue;
	int POI_number_x = 100;
	int POI_number_y = 200;
	int grid_space = 2;

	//store POIs in a queue
	for (int i = 0; i < POI_number_y; i++) {
		for (int j = 0; j < POI_number_x; j++) {
			Point2D offset(j * grid_space, i * grid_space);
			Point2D current_point = upper_left_point + offset;
			POI2D current_POI(current_point);
			POI_queue.push_back(current_POI);
		}
	}
	//set OpenMP parameters
	int CPU_thread_number = omp_get_num_procs() - 1;
	omp_set_num_threads(CPU_thread_number);

	//get time of start
	double timer_tic = omp_get_wtime();

	//SIFT extraction and match
	SIFT2D* sift = new SIFT2D();
	sift->setImages(ref_img, tar_img);
	sift->prepare();
	sift->compute();

	//FeatureAffine instance estimates the deformation at POI according the neighbor features
	FeatureAffine2D* feature_affine = new FeatureAffine2D(subset_radius_x, subset_radius_y);
	feature_affine->setImages(ref_img, tar_img);
	feature_affine->setKeypointPair(sift->ref_matched_kp, sift->tar_matched_kp);
	feature_affine->compute(POI_queue);

	//ICGN 2nd
	ICGN2D2* icgn2 = new ICGN2D2(subset_radius_x, subset_radius_y, max_deformation_norm, max_iteration, CPU_thread_number);
	icgn2->setImages(ref_img, tar_img);
	icgn2->prepare();
	icgn2->compute(POI_queue);

	//get time of end
	double timer_toc = omp_get_wtime();
	double consumed_time = timer_toc - timer_tic;

	cout << consumed_time << " sec";

	//output the results
	IO2D results_output;
	string file_path;
	results_output.setHeight(ref_img.height);
	results_output.setWidth(ref_img.width);
	results_output.setDelimiter(string(","));

	//save the calculated dispalcements
	file_path = tar_image_path + "_table_SIFT.csv";
	results_output.setPath(file_path);
	results_output.saveTable2D(POI_queue);

	//save the full deformation vector
	file_path = tar_image_path + "_deformation_SIFT.csv";
	results_output.setPath(file_path);
	results_output.saveDeformationTable2D(POI_queue);

	//save the map of u-component
	file_path = tar_image_path + "_u_SIFT.csv";
	results_output.setPath(file_path);
	results_output.saveMap2D(POI_queue, 2);

	//save the map of v-component
	file_path = tar_image_path + "_v_SIFT.csv";
	results_output.setPath(file_path);
	results_output.saveMap2D(POI_queue, 3);

	cin.get();

	delete icgn2;
	delete feature_affine;
	delete sift;

	return 0;
}