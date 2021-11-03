/*
 * This file is part of OpenCorr, an open source C++ library for
 * study and development of 2D, 3D/stereo and volumetric
 * digital image correlation.
 *
 * Copyright (C) 2021, Zhenyu Jiang <zhenyujiang@scut.edu.cn>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * More information about OpenCorr can be found at https://www.opencorr.org/
 */

#include "oc_icgn.h"

namespace opencorr
{
	//the 1st order shape function--------------------------------------------
	ICGN2D1_* ICGN2D1_::allocate(int subset_radius_x, int subset_radius_y) {
		int subset_width = 2 * subset_radius_x + 1;
		int subset_height = 2 * subset_radius_y + 1;
		Point2D subset_center(0, 0);

		ICGN2D1_* ICGN_instance = new ICGN2D1_;
		ICGN_instance->ref_subset = new Subset2D(subset_center, subset_radius_x, subset_radius_y);
		ICGN_instance->tar_subset = new Subset2D(subset_center, subset_radius_x, subset_radius_y);
		ICGN_instance->error_img = Eigen::MatrixXf::Zero(subset_height, subset_width);
		ICGN_instance->sd_img = new3D(subset_height, subset_width, 6);

		return ICGN_instance;
	}

	void ICGN2D1_::release(ICGN2D1_* instance) {
		delete instance->ref_subset;
		delete instance->tar_subset;
		delete3D(instance->sd_img);
	}

	ICGN2D1::ICGN2D1(int subset_radius_x, int subset_radius_y, float conv_criterion, float stop_condition, int thread_number)
		: ref_gradient(nullptr), tar_interp(nullptr) {
		this->subset_radius_x = subset_radius_x;
		this->subset_radius_y = subset_radius_y;
		this->conv_criterion = conv_criterion;
		this->stop_condition = stop_condition;
		this->thread_number = thread_number;

		for (int i = 0; i < thread_number; i++) {
			ICGN2D1_* instance = ICGN2D1_::allocate(subset_radius_x, subset_radius_y);
			instance_pool.push_back(instance);
		}
	}

	ICGN2D1::~ICGN2D1() {
		delete ref_gradient;
		delete tar_interp;

		for (auto& instance : instance_pool) {
			ICGN2D1_::release(instance);
			delete instance;
		}
		instance_pool.clear();
	}

	void ICGN2D1::setIteration(float conv_criterion, float stop_condition) {
		this->conv_criterion = conv_criterion;
		this->stop_condition = stop_condition;
	}

	void ICGN2D1::setIteration(POI2D* poi) {
		conv_criterion = poi->result.convergence;
		stop_condition = (int)poi->result.iteration;
	}

	ICGN2D1_* ICGN2D1::getInstance(int tid)
	{
		if (tid >= (int)instance_pool.size()) {
			throw std::string("CPU thread ID over limit");
		}

		return instance_pool[tid];
	}

	void ICGN2D1::prepareRef() {
		if (ref_gradient != nullptr) {
			delete ref_gradient;
			ref_gradient = nullptr;
		}

		ref_gradient = new Gradient2D4(*ref_img);
		ref_gradient->getGradientX();
		ref_gradient->getGradientY();
	}

	void ICGN2D1::prepareTar() {
		if (tar_interp != nullptr) {
			delete tar_interp;
			tar_interp = nullptr;
		}

		tar_interp = new BicubicBspline(*tar_img);
		tar_interp->prepare();
	}

	void ICGN2D1::prepare() {
		prepareRef();
		prepareTar();
	}

	void ICGN2D1::compute(POI2D* poi) {
		//set instance w.r.t. thread id 
		ICGN2D1_* cur_instance = getInstance(omp_get_thread_num());

		if (poi->y - subset_radius_y < 0 || poi->x - subset_radius_x < 0
			|| poi->y + subset_radius_y > ref_img->height - 1 || poi->x + subset_radius_x > ref_img->width - 1) {
			//std::cerr << "POI is too close to boundary" << poi->x << ", " << poi->y << std::endl;
			poi->result.zncc = 0;
		}
		else {
			int subset_width = 2 * subset_radius_x + 1;
			int subset_height = 2 * subset_radius_y + 1;

			//set reference subset
			cur_instance->ref_subset->center = (Point2D)*poi;
			cur_instance->ref_subset->fill(ref_img);
			float ref_mean_norm = cur_instance->ref_subset->zeroMeanNorm();

			//build the hessian matrix
			cur_instance->hessian.setZero();
			for (int r = 0; r < subset_height; r++) {
				for (int c = 0; c < subset_width; c++) {
					int x_local = c - subset_radius_x;
					int y_local = r - subset_radius_y;
					int x_global = poi->x + x_local;
					int y_global = poi->y + y_local;
					float ref_gradient_x = ref_gradient->gradient_x(y_global, x_global);
					float ref_gradient_y = ref_gradient->gradient_y(y_global, x_global);

					cur_instance->sd_img[r][c][0] = ref_gradient_x;
					cur_instance->sd_img[r][c][1] = ref_gradient_x * x_local;
					cur_instance->sd_img[r][c][2] = ref_gradient_x * y_local;
					cur_instance->sd_img[r][c][3] = ref_gradient_y;
					cur_instance->sd_img[r][c][4] = ref_gradient_y * x_local;
					cur_instance->sd_img[r][c][5] = ref_gradient_y * y_local;

					for (int i = 0; i < 6; i++) {
						float sum = 0;
						for (int j = 0; j < 6; j++) {
							sum = cur_instance->hessian(i, j) + cur_instance->sd_img[r][c][i] * cur_instance->sd_img[r][c][j];
							cur_instance->hessian(i, j) = sum;
						}
					}
				}
			}

			//compute inversed hessian matrix
			cur_instance->inv_hessian = cur_instance->hessian.inverse();

			//set target subset
			cur_instance->tar_subset->center = (Point2D)*poi;

			//get initial guess
			Deformation2D1 p_initial(poi->deformation.u, poi->deformation.ux, poi->deformation.uy,
				poi->deformation.v, poi->deformation.vx, poi->deformation.vy);

			//IC-GN iteration
			int iteration = 0; //initialize iteration counter
			Deformation2D1 p_current, p_increment;
			p_current.setDeformation(p_initial);
			float dp_norm_max, znssd;
			Point2D local_coor, warped_coor, global_coor;
			do {
				iteration++;
				//reconstruct target subset
				for (int r = 0; r < subset_height; r++) {
					for (int c = 0; c < subset_width; c++) {
						int x_local = c - subset_radius_x;
						int y_local = r - subset_radius_y;
						local_coor.x = x_local;
						local_coor.y = y_local;
						warped_coor = p_current.warp(local_coor);
						global_coor = (Point2D)*poi + warped_coor;
						cur_instance->tar_subset->eg_mat(r, c) = tar_interp->compute(global_coor);
					}
				}
				float tar_mean_norm = cur_instance->tar_subset->zeroMeanNorm();

				//compute error image
				cur_instance->error_img = cur_instance->tar_subset->eg_mat * (ref_mean_norm / tar_mean_norm)
					- (cur_instance->ref_subset->eg_mat);

				//calculate ZNSSD
				znssd = cur_instance->error_img.squaredNorm() / (ref_mean_norm * ref_mean_norm);

				//compute numerator
				float numerator[6] = { 0 };
				for (int r = 0; r < subset_height; r++) {
					for (int c = 0; c < subset_width; c++) {
						for (int i = 0; i < 6; i++) {
							numerator[i] += cur_instance->sd_img[r][c][i] * cur_instance->error_img(r, c);
						}
					}
				}

				//compute dp
				float dp[6] = { 0 };
				for (int i = 0; i < 6; i++) {
					for (int j = 0; j < 6; j++) {
						dp[i] += cur_instance->inv_hessian(i, j) * numerator[j];
					}
				}
				p_increment.setDeformation(dp);

				//update warp
				p_current.warp_matrix = p_current.warp_matrix * p_increment.warp_matrix.inverse();

				//update p
				p_current.setDeformation();

				//check convergence
				int subset_radius_x2 = subset_radius_x * subset_radius_x;
				int subset_radius_y2 = subset_radius_y * subset_radius_y;

				dp_norm_max = 0.f;
				dp_norm_max += p_increment.u * p_increment.u;
				dp_norm_max += p_increment.ux * p_increment.ux * subset_radius_x2;
				dp_norm_max += p_increment.uy * p_increment.uy * subset_radius_y2;
				dp_norm_max += p_increment.v * p_increment.v;
				dp_norm_max += p_increment.vx * p_increment.vx * subset_radius_x2;
				dp_norm_max += p_increment.vy * p_increment.vy * subset_radius_y2;

				dp_norm_max = sqrtf(dp_norm_max);
			} while (iteration < stop_condition && dp_norm_max >= conv_criterion);

			//store the final result
			poi->deformation.u = p_current.u;
			poi->deformation.ux = p_current.ux;
			poi->deformation.uy = p_current.uy;
			poi->deformation.v = p_current.v;
			poi->deformation.vx = p_current.vx;
			poi->deformation.vy = p_current.vy;

			//save the results for output
			poi->result.u0 = p_initial.u;
			poi->result.v0 = p_initial.v;
			poi->result.zncc = 0.5f * (2 - znssd);
			poi->result.iteration = (float)iteration;
			poi->result.convergence = dp_norm_max;
		}
	}

	void ICGN2D1::compute(std::vector<POI2D>& poi_queue) {
#pragma omp parallel for
		for (int i = 0; i < poi_queue.size(); ++i) {
			compute(&poi_queue[i]);
		}
	}



	//the 2nd order shape function--------------------------------------------
	ICGN2D2_* ICGN2D2_::allocate(int subset_radius_x, int subset_radius_y) {
		int subset_width = 2 * subset_radius_x + 1;
		int subset_height = 2 * subset_radius_y + 1;
		Point2D subset_center(0, 0);

		ICGN2D2_* ICGN_instance = new ICGN2D2_;
		ICGN_instance->ref_subset = new Subset2D(subset_center, subset_radius_x, subset_radius_y);
		ICGN_instance->tar_subset = new Subset2D(subset_center, subset_radius_x, subset_radius_y);
		ICGN_instance->error_img = Eigen::MatrixXf::Zero(subset_height, subset_width);
		ICGN_instance->sd_img = new3D(subset_height, subset_width, 12);

		return ICGN_instance;
	}

	void ICGN2D2_::release(ICGN2D2_* instance) {
		delete instance->ref_subset;
		delete instance->tar_subset;
		delete3D(instance->sd_img);
	}

	ICGN2D2::ICGN2D2(int thread_number) :ref_gradient(nullptr), tar_interp(nullptr) {
		this->thread_number = thread_number;
		for (int i = 0; i < thread_number; i++) {
			ICGN2D2_* instance = ICGN2D2_::allocate(subset_radius_x, subset_radius_y);
			instance_pool.push_back(instance);
		}
	}

	ICGN2D2::ICGN2D2(int subset_radius_x, int subset_radius_y, float conv_criterion, float stop_condition, int thread_number)
		: ref_gradient(nullptr), tar_interp(nullptr) {
		this->subset_radius_x = subset_radius_x;
		this->subset_radius_y = subset_radius_y;
		this->conv_criterion = conv_criterion;
		this->stop_condition = stop_condition;
		this->thread_number = thread_number;

		for (int i = 0; i < thread_number; i++) {
			ICGN2D2_* instance = ICGN2D2_::allocate(subset_radius_x, subset_radius_y);
			instance_pool.push_back(instance);
		}
	}

	ICGN2D2::~ICGN2D2() {
		delete ref_gradient;
		delete tar_interp;

		for (auto& instance : instance_pool) {
			ICGN2D2_::release(instance);
			delete instance;
		}
		instance_pool.clear();

	}

	void ICGN2D2::setIteration(float conv_criterion, float stop_condition) {
		this->conv_criterion = conv_criterion;
		this->stop_condition = stop_condition;
	}

	void ICGN2D2::setIteration(POI2D* poi) {
		conv_criterion = poi->result.convergence;
		stop_condition = poi->result.iteration;
	}

	ICGN2D2_* ICGN2D2::getInstance(int tid)
	{
		if (tid >= (int)instance_pool.size()) {
			throw std::string("CPU thread ID over limit");
		}

		return instance_pool[tid];
	}

	void ICGN2D2::prepareRef() {
		if (ref_gradient != nullptr) {
			delete ref_gradient;
			ref_gradient = nullptr;
		}

		ref_gradient = new Gradient2D4(*ref_img);
		ref_gradient->getGradientX();
		ref_gradient->getGradientY();
	}

	void ICGN2D2::prepareTar() {
		if (tar_interp != nullptr) {
			delete tar_interp;
			tar_interp = nullptr;
		}

		tar_interp = new BicubicBspline(*tar_img);
		tar_interp->prepare();
	}

	void ICGN2D2::prepare() {
		prepareRef();
		prepareTar();
	}

	void ICGN2D2::compute(POI2D* poi) {
		//set instance w.r.t. thread id 
		ICGN2D2_* cur_instance = getInstance(omp_get_thread_num());

		if (poi->y - subset_radius_y < 0 || poi->x - subset_radius_x < 0
			|| poi->y + subset_radius_y > ref_img->height - 1 || poi->x + subset_radius_x > ref_img->width - 1) {
			//std::cerr << "POI is too close to boundary" << poi->x << ", " << poi->y << std::endl;
			poi->result.zncc = 0;
		}
		else {
			int subset_width = 2 * subset_radius_x + 1;
			int subset_height = 2 * subset_radius_y + 1;

			//set reference subset
			cur_instance->ref_subset->center = (Point2D)*poi;
			cur_instance->ref_subset->fill(ref_img);
			float ref_mean_norm = cur_instance->ref_subset->zeroMeanNorm();

			//build the hessian matrix
			cur_instance->hessian.setZero();
			for (int r = 0; r < subset_height; r++) {
				for (int c = 0; c < subset_width; c++) {
					int x_local = c - subset_radius_x;
					int y_local = r - subset_radius_y;
					float xx_local = (x_local * x_local) * 0.5f;
					float xy_local = (float)(x_local * y_local);
					float yy_local = (y_local * y_local) * 0.5f;
					int x_global = poi->x + x_local;
					int y_global = poi->y + y_local;
					float ref_gradient_x = ref_gradient->gradient_x(y_global, x_global);
					float ref_gradient_y = ref_gradient->gradient_y(y_global, x_global);

					cur_instance->sd_img[r][c][0] = ref_gradient_x;
					cur_instance->sd_img[r][c][1] = ref_gradient_x * x_local;
					cur_instance->sd_img[r][c][2] = ref_gradient_x * y_local;
					cur_instance->sd_img[r][c][3] = ref_gradient_x * xx_local;
					cur_instance->sd_img[r][c][4] = ref_gradient_x * xy_local;
					cur_instance->sd_img[r][c][5] = ref_gradient_x * yy_local;

					cur_instance->sd_img[r][c][6] = ref_gradient_y;
					cur_instance->sd_img[r][c][7] = ref_gradient_y * x_local;
					cur_instance->sd_img[r][c][8] = ref_gradient_y * y_local;
					cur_instance->sd_img[r][c][9] = ref_gradient_y * xx_local;
					cur_instance->sd_img[r][c][10] = ref_gradient_y * xy_local;
					cur_instance->sd_img[r][c][11] = ref_gradient_y * yy_local;

					for (int i = 0; i < 12; i++) {
						float sum = 0;
						for (int j = 0; j < 12; j++) {
							sum = cur_instance->hessian(i, j) + cur_instance->sd_img[r][c][i] * cur_instance->sd_img[r][c][j];
							cur_instance->hessian(i, j) = sum;
						}
					}
				}
			}

			//compute inversed hessian matrix
			cur_instance->inv_hessian = cur_instance->hessian.inverse();

			//set target subset
			cur_instance->tar_subset->center = (Point2D)*poi;

			//get initial guess
			Deformation2D1 p_initial(poi->deformation.u, poi->deformation.ux, poi->deformation.uy,
				poi->deformation.v, poi->deformation.vx, poi->deformation.vy);

			//IC-GN iteration
			int iteration = 0; //initialize iteration counter
			Deformation2D2 p_current, p_increment;
			p_current.setDeformation(p_initial);
			float dp_norm_max, znssd;
			Point2D local_coor, warped_coor, global_coor;
			do {
				iteration++;
				//reconstruct target subset
				for (int r = 0; r < subset_height; r++) {
					for (int c = 0; c < subset_width; c++) {
						int x_local = c - subset_radius_x;
						int y_local = r - subset_radius_y;
						local_coor.x = x_local;
						local_coor.y = y_local;
						warped_coor = p_current.warp(local_coor);
						global_coor = (Point2D)*poi + warped_coor;
						cur_instance->tar_subset->eg_mat(r, c) = tar_interp->compute(global_coor);
					}
				}
				float tar_mean_norm = cur_instance->tar_subset->zeroMeanNorm();

				//compute error image
				cur_instance->error_img = cur_instance->tar_subset->eg_mat * (ref_mean_norm / tar_mean_norm)
					- (cur_instance->ref_subset->eg_mat);

				//calculate ZNSSD
				znssd = cur_instance->error_img.squaredNorm() / (ref_mean_norm * ref_mean_norm);

				//compute numerator
				float numerator[12] = { 0 };
				for (int r = 0; r < subset_height; r++) {
					for (int c = 0; c < subset_width; c++) {
						for (int i = 0; i < 12; i++) {
							numerator[i] += cur_instance->sd_img[r][c][i] * cur_instance->error_img(r, c);
						}
					}
				}

				//compute dp
				float dp[12] = { 0 };
				for (int i = 0; i < 12; i++) {
					for (int j = 0; j < 12; j++) {
						dp[i] += cur_instance->inv_hessian(i, j) * numerator[j];
					}
				}
				p_increment.setDeformation(dp);

				//update warp
				p_current.warp_matrix = p_current.warp_matrix * p_increment.warp_matrix.inverse();

				//update p
				p_current.setDeformation();

				//check convergence
				int subset_radius_x2 = subset_radius_x * subset_radius_x;
				int subset_radius_y2 = subset_radius_y * subset_radius_y;
				int subset_radius_xy = subset_radius_x2 * subset_radius_y2;

				dp_norm_max = 0.f;
				dp_norm_max += p_increment.u * p_increment.u;
				dp_norm_max += p_increment.ux * p_increment.ux * subset_radius_x2;
				dp_norm_max += p_increment.uy * p_increment.uy * subset_radius_y2;
				dp_norm_max += p_increment.uxx * p_increment.uxx * subset_radius_x2 * subset_radius_x2 / 4.f;
				dp_norm_max += p_increment.uyy * p_increment.uyy * subset_radius_y2 * subset_radius_y2 / 4.f;
				dp_norm_max += p_increment.uxy * p_increment.uxy * subset_radius_xy;

				dp_norm_max += p_increment.v * p_increment.v;
				dp_norm_max += p_increment.vx * p_increment.vx * subset_radius_x2;
				dp_norm_max += p_increment.vy * p_increment.vy * subset_radius_y2;
				dp_norm_max += p_increment.vxx * p_increment.vxx * subset_radius_x2 * subset_radius_x2 / 4.f;
				dp_norm_max += p_increment.vyy * p_increment.vyy * subset_radius_y2 * subset_radius_y2 / 4.f;
				dp_norm_max += p_increment.vxy * p_increment.vxy * subset_radius_xy;

				dp_norm_max = sqrtf(dp_norm_max);
			} while (iteration < stop_condition && dp_norm_max >= conv_criterion);

			//store the final result
			poi->deformation.u = p_current.u;
			poi->deformation.ux = p_current.ux;
			poi->deformation.uy = p_current.uy;
			poi->deformation.uxx = p_current.uxx;
			poi->deformation.uyy = p_current.uyy;
			poi->deformation.uxy = p_current.uxy;

			poi->deformation.v = p_current.v;
			poi->deformation.vx = p_current.vx;
			poi->deformation.vy = p_current.vy;
			poi->deformation.vxx = p_current.vxx;
			poi->deformation.vyy = p_current.vyy;
			poi->deformation.vxy = p_current.vxy;

			//save the results for output
			poi->result.u0 = p_initial.u;
			poi->result.v0 = p_initial.v;
			poi->result.zncc = 0.5f * (2 - znssd);
			poi->result.iteration = (float)iteration;
			poi->result.convergence = dp_norm_max;
		}
	}

	void ICGN2D2::compute(std::vector<POI2D>& poi_queue) {
#pragma omp parallel for
		for (int i = 0; i < poi_queue.size(); ++i) {
			compute(&poi_queue[i]);
		}
	}

}//namespace opencorr