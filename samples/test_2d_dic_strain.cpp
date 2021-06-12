/*
* This is an example demonstrating how to use OpenCorr to
* calculate the stain according to the polynomial fit displacement
* fields of u component and v component
*/

#include "opencorr.h"

using namespace opencorr;
using namespace std;

int main() {
	//select the image file to get the file name to process
	string tar_image_path = "../samples/oht_cfrp_4.bmp";
	Image2D tar_img(tar_image_path);

	//get the DIC results from csv file
	IO2D in_out;
	string file_path = tar_image_path + "_table.csv";
	in_out.setPath(file_path);
	in_out.setHeight(tar_img.height);
	in_out.setWidth(tar_img.width);
	in_out.setDelimiter(string(","));

	//load a queue of POIs
	vector<POI2D> POI_queue = in_out.loadTable2D();

	//get time of start
	double timer_tic = omp_get_wtime();

	//set OpenMP parameters
	int CPU_thread_number = omp_get_num_procs() - 1;
	omp_set_num_threads(CPU_thread_number);

	//set the radius of subregion for polynomial fit of displacement field
	int strain_radius = 15;

	//set the grid space between POIs
	//it should identical to the value set in DIC computation
	int grid_space = 2;

	//initialize a object of stain calculation
	LSFitting* strain = new LSFitting(strain_radius, grid_space);

	//create the maps of u component and v component
	strain->setDisplacement(POI_queue);

	//calculate the strain exx, exy, eyy
	strain->compute(POI_queue);

	//get time of end
	double timer_toc = omp_get_wtime();
	double consumed_time = timer_toc - timer_tic;

	cout << consumed_time << " sec";

	//update the table of DIC results with calculated strains
	in_out.saveTable2D(POI_queue);

	//save eyy
	file_path = tar_image_path + "_eyy.csv";
	in_out.setPath(file_path);

	//use the index in array of Result2D to choose variable for output
	//the order: u0, v0, u, v, exx, eyy, exy, ZNCC, iteration, convergence, feature
	in_out.saveMap2D(POI_queue, 5);

	cin.get();

	delete strain;

	return 0;
}