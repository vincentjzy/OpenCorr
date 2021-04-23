/*
 * This file is part of OpenCorr, an open source C++ library for
 * study and development of 2D, 3D/stereo and volumetric
 * digital image correlation.
 *
 * Copyright (C) 2021, Zhenyu Jiang <zhenyujiang@scut.edu.cn>
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * More information about OpenCorr can be found at https://www.opencorr.org/
 */

#include "oc_strain.h"

namespace opencorr
{
	Strain2D::Strain2D(int subregion_radius, int grid_space) {
		this->subregion_radius = subregion_radius;
		this->grid_space = grid_space;
	}

	Strain2D::~Strain2D() {
	}

	int Strain2D::getSubregionRadius() const {
		return this->subregion_radius;
	}

	int Strain2D::getGridSpace() const {
		return this->grid_space;
	}

	void Strain2D::setSubregionRadius(int subresionRadius) {
		this->subregion_radius = subregion_radius;
	}

	void Strain2D::setGridSpace(int grid) {
		this->grid_space = grid;
	}

	void Strain2D::setDisplacement(std::vector<POI2D>& POI_queue) {
		size_t POI_number = POI_queue.size();

		//get boundary of ROI along x-axis
		std::sort(POI_queue.begin(), POI_queue.end(), sortByX);
		int x_min = (int)POI_queue[0].x;
		int x_max = (int)POI_queue[POI_number - 1].x;
		//calculate the width of POI matrix
		int width = (int)((x_max - x_min) / this->grid_space + 1);

		//get boundary of ROI along y-axis
		std::sort(POI_queue.begin(), POI_queue.end(), sortByY);
		int y_min = (int)POI_queue[0].y;
		int y_max = (int)POI_queue[POI_number - 1].y;
		//calculate the height of POI matrix
		int height = (int)((y_max - y_min) / this->grid_space + 1);

		this->topleft_point.x = (float)x_min;
		this->topleft_point.y = (float)y_min;

		//create component matrices of POIs;
		this->u_map = Eigen::MatrixXf::Zero(height, width);
		this->v_map = Eigen::MatrixXf::Zero(height, width);

		//fill the matrices with the components;
		for (int i = 0; i < POI_number; ++i) {
			int col = (int)((POI_queue[i].x - x_min) / this->grid_space);
			int row = (int)((POI_queue[i].y - y_min) / this->grid_space);
			this->u_map(row, col) = POI_queue[i].result.u;
			this->v_map(row, col) = POI_queue[i].result.v;
		}
	}

	void Strain2D::prepare() {
	}

	SGFilter::SGFilter(int radius, int grid) : Strain2D(radius, grid) {
	}

	SGFilter::~SGFilter() {
	}

	void SGFilter::compute(POI2D* POI) {
		//create Savitzky-Golay filter
		int filter_size = 2 * this->subregion_radius + 1;
		Eigen::MatrixXf coefficient_matrix = Eigen::MatrixXf::Zero(filter_size, filter_size);
		float matrix_factor = 3.f / (float)(filter_size * filter_size * (this->subregion_radius + 1.f) * this->subregion_radius * this->grid_space);
		for (int r = 0; r < filter_size; r++) {
			for (int c = 0; c < filter_size; c++) {
				coefficient_matrix(r, c) = matrix_factor * (c - this->subregion_radius);
			}
		}

		//get location of POI in u_map and v_map
		Point2D relative_location = *POI - this->topleft_point;
		int map_x = (int)(relative_location.x / this->grid_space);
		int map_y = (int)(relative_location.y / this->grid_space);

		//deal with the POI close to boundary of ROI
		int height = this->v_map.rows();
		int width = this->u_map.cols();
		if (map_y < this->subregion_radius)
			map_y = this->subregion_radius;
		if (map_y >= height - this->subregion_radius)
			map_y = height - this->subregion_radius - 1;
		if (map_x < this->subregion_radius)
			map_x = this->subregion_radius;
		if (map_x >= width - this->subregion_radius)
			map_x = width - this->subregion_radius - 1;

		//create a transposed matrix of filter
		Eigen::VectorXf coefficient_matrix_transpose = coefficient_matrix.transpose();

		//calculate the coefficients
		float ux = 0;
		float uy = 0;
		float vx = 0;
		float vy = 0;
		for (int r = 0; r < filter_size; r++) {
			int map_r = map_y + r - this->subregion_radius;
			for (int c = 0; c < filter_size; c++) {
				int map_c = map_x + c - this->subregion_radius;
				ux += coefficient_matrix(r, c) * this->u_map(map_r, map_c);
				vx += coefficient_matrix(r, c) * this->v_map(map_r, map_c);

				uy += coefficient_matrix_transpose(r, c) * this->u_map(map_r, map_c);
				vy += coefficient_matrix_transpose(r, c) * this->v_map(map_r, map_c);
			}
		}

		//calculate the strains and save them for output
		POI->result.exx = ux + 0.5f * (ux * ux + vx * vx);
		POI->result.eyy = vy + 0.5f * (uy * uy + vy * vy);
		POI->result.exy = 0.5f * (uy + vx + ux * uy + vx * vy);

	}

	LSFitting::LSFitting(int radius, int grid) :Strain2D(radius, grid) {
	}

	LSFitting::~LSFitting() {
	}

	void LSFitting::compute(POI2D* POI) {
		//get location of POI in u_map and v_map
		Point2D relative_location = *POI - this->topleft_point;

		//calculate the location of POI in u_map and v_map 
		int map_x = (int)(relative_location.x / this->grid_space);
		int map_y = (int)(relative_location.y / this->grid_space);

		//deal with the POI close to the boundary of ROI
		int height = this->v_map.rows();
		int width = this->u_map.cols();
		int subregion_x1 = map_x - this->subregion_radius;
		int	subregion_x2 = map_x + this->subregion_radius;
		int subregion_y1 = map_y - this->subregion_radius;
		int subregion_y2 = map_y + this->subregion_radius;
		if (map_x < this->subregion_radius)
			subregion_x1 = 0;
		if (map_x >= width - this->subregion_radius)
			subregion_x2 = width - 1;
		if (map_y < this->subregion_radius)
			subregion_y1 = 0;
		if (map_y >= height - this->subregion_radius)
			subregion_y2 = height - 1;

		int subregion_size_c = (subregion_x2 - subregion_x1) + 1;
		int subregion_size_r = (subregion_y2 - subregion_y1) + 1;
		int subregion_size = subregion_size_r * subregion_size_c;

		//create two matrices of displacments
		Eigen::VectorXf u_vector(subregion_size);
		Eigen::VectorXf v_vector(subregion_size);

		//construct coefficient matrix
		Eigen::MatrixXf coefficient_matrix = Eigen::MatrixXf::Zero(subregion_size, 3);

		//fill coefficient matrix, u_vector and v_vector
		int counter = 0;
		for (int r = subregion_y1; r <= subregion_y2; r++) {
			for (int c = subregion_x1; c <= subregion_x2; c++) {
				coefficient_matrix(counter, 0) = 1.f;
				coefficient_matrix(counter, 1) = (float)(c - map_x);
				coefficient_matrix(counter, 2) = (float)(r - map_y);

				u_vector(counter) = this->u_map(r, c);
				v_vector(counter) = this->v_map(r, c);
				counter++;
			}
		}

		//solve the equations to obtain gradients of u and v
		Eigen::VectorXf u_gradient = coefficient_matrix.colPivHouseholderQr().solve(u_vector);
		Eigen::VectorXf v_gradient = coefficient_matrix.colPivHouseholderQr().solve(v_vector);
		float ux = u_gradient(1, 0) / this->grid_space;
		float uy = u_gradient(2, 0) / this->grid_space;
		float vx = v_gradient(1, 0) / this->grid_space;
		float vy = v_gradient(2, 0) / this->grid_space;

		//calculate the strains and save them for output
		POI->result.exx = ux + 0.5f * (ux * ux + vx * vx);
		POI->result.eyy = vy + 0.5f * (uy * uy + vy * vy);
		POI->result.exy = 0.5f * (uy + vx + ux * uy + vx * vy);

	}

	bool sortByX(const POI2D& p1, const POI2D& p2) {
		return p1.x < p2.x;
	}

	bool sortByY(const POI2D& p1, const POI2D& p2) {
		return p1.y < p2.y;
	}

}//namespace opencorr
