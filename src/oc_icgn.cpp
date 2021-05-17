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

#include "oc_icgn.h"

namespace opencorr
{
	//1st order shape function
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
			ICGN2D1_* instance = ICGN2D1_::allocate(this->subset_radius_x, this->subset_radius_y);
			this->instance_pool.push_back(instance);
		}
	}

	ICGN2D1::~ICGN2D1() {
		delete this->ref_gradient;
		delete this->tar_interp;

		for (auto& instance : this->instance_pool) {
			ICGN2D1_::release(instance);
			delete instance;
		}
		this->instance_pool.clear();
	}

	void ICGN2D1::setIteration(float conv_criterion, float stop_condition) {
		this->conv_criterion = conv_criterion;
		this->stop_condition = stop_condition;
	}

	void ICGN2D1::setIteration(POI2D* POI) {
		this->conv_criterion = POI->result.convergence;
		this->stop_condition = (int)POI->result.iteration;
	}

	ICGN2D1_* ICGN2D1::getInstance(int tid)
	{
		if (tid >= (int)this->instance_pool.size()) {
			throw std::string("CPU thread ID over limit");
		}

		return this->instance_pool[tid];
	}

	void ICGN2D1::prepareRef() {
		if (this->ref_gradient != nullptr) {
			delete this->ref_gradient;
			this->ref_gradient = nullptr;
		}

		this->ref_gradient = new Gradient2D4(*this->ref_img);
		this->ref_gradient->getGradientX();
		this->ref_gradient->getGradientY();
	}

	void ICGN2D1::prepareTar() {
		if (this->tar_interp != nullptr) {
			delete this->tar_interp;
			this->tar_interp = nullptr;
		}

		this->tar_interp = new BicubicBspline(*this->tar_img);
		this->tar_interp->prepare();
	}

	void ICGN2D1::prepare() {
		this->prepareRef();
		this->prepareTar();
	}

	void ICGN2D1::compute(POI2D* POI) {
		int subset_width = 2 * this->subset_radius_x + 1;
		int subset_height = 2 * this->subset_radius_y + 1;

		//set instance w.r.t. thread id 
		ICGN2D1_* cur_instance = this->getInstance(omp_get_thread_num());

		//set reference subset
		cur_instance->ref_subset->center = (Point2D)*POI;
		cur_instance->ref_subset->fill(this->ref_img);
		float ref_mean_norm = cur_instance->ref_subset->zeroMeanNorm();

		//build the hessian matrix
		cur_instance->hessian.setZero();
		for (int r = 0; r < subset_height; r++) {
			for (int c = 0; c < subset_width; c++) {
				int x_local = c - this->subset_radius_x;
				int y_local = r - this->subset_radius_y;
				int x_global = POI->x + x_local;
				int y_global = POI->y + y_local;
				float ref_gradient_x = this->ref_gradient->gradient_x(y_global, x_global);
				float ref_gradient_y = this->ref_gradient->gradient_y(y_global, x_global);

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
		cur_instance->tar_subset->center = (Point2D)*POI;

		//get initial guess
		Deformation2D1 p_initial(POI->deformation.u, POI->deformation.ux, POI->deformation.uy,
			POI->deformation.v, POI->deformation.vx, POI->deformation.vy);

		//IC-GN iteration
		int iteration = 0; //initialize iteration counter
		Deformation2D1 p_current, p_increment;
		p_current.setDeformation(p_initial);
		float dp_norm_max, ZNSSD;
		Point2D local_coor, warped_coor, global_coor;
		do {
			iteration++;
			//reconstruct target subset
			for (int r = 0; r < subset_height; r++) {
				for (int c = 0; c < subset_width; c++) {
					int x_local = c - this->subset_radius_x;
					int y_local = r - this->subset_radius_y;
					local_coor.x = x_local;
					local_coor.y = y_local;
					warped_coor = p_current.warp(local_coor);
					global_coor = (Point2D)*POI + warped_coor;
					cur_instance->tar_subset->eg_mat(r, c) = this->tar_interp->compute(global_coor);
				}
			}
			float tar_mean_norm = cur_instance->tar_subset->zeroMeanNorm();

			//compute error image
			cur_instance->error_img = cur_instance->tar_subset->eg_mat * (ref_mean_norm / tar_mean_norm)
				- (cur_instance->ref_subset->eg_mat);

			//calculate ZNSSD
			ZNSSD = cur_instance->error_img.squaredNorm() / (ref_mean_norm * ref_mean_norm);

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
			int subset_radius_x2 = this->subset_radius_x * this->subset_radius_x;
			int subset_radius_y2 = this->subset_radius_y * this->subset_radius_y;

			dp_norm_max = 0.f;
			dp_norm_max += p_increment.u * p_increment.u;
			dp_norm_max += p_increment.ux * p_increment.ux * subset_radius_x2;
			dp_norm_max += p_increment.uy * p_increment.uy * subset_radius_y2;
			dp_norm_max += p_increment.v * p_increment.v;
			dp_norm_max += p_increment.vx * p_increment.vx * subset_radius_x2;
			dp_norm_max += p_increment.vy * p_increment.vy * subset_radius_y2;

			dp_norm_max = sqrtf(dp_norm_max);
		} while (iteration < this->stop_condition && dp_norm_max >= this->conv_criterion);

		//store the final result
		POI->deformation.u = p_current.u;
		POI->deformation.ux = p_current.ux;
		POI->deformation.uy = p_current.uy;
		POI->deformation.v = p_current.v;
		POI->deformation.vx = p_current.vx;
		POI->deformation.vy = p_current.vy;

		//save the results for output
		POI->result.u0 = p_initial.u;
		POI->result.v0 = p_initial.v;
		POI->result.u = p_current.u;
		POI->result.v = p_current.v;
		POI->result.ZNCC = 0.5f * (2 - ZNSSD);
		POI->result.iteration = (float)iteration;
		POI->result.convergence = dp_norm_max;
	}

	void ICGN2D1::compute(std::vector<POI2D>& POI_queue) {
#pragma omp parallel for
		for (int i = 0; i < POI_queue.size(); ++i) {
			this->compute(&POI_queue[i]);
		}
	}



	//2nd order shape function////////////////////////////////////////////////////////////////////////////////////////
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
			ICGN2D2_* instance = ICGN2D2_::allocate(this->subset_radius_x, this->subset_radius_y);
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
			ICGN2D2_* instance = ICGN2D2_::allocate(this->subset_radius_x, this->subset_radius_y);
			instance_pool.push_back(instance);
		}
	}

	ICGN2D2::~ICGN2D2() {
		delete this->ref_gradient;
		delete this->tar_interp;

		for (auto& instance : this->instance_pool) {
			ICGN2D2_::release(instance);
			delete instance;
		}
		instance_pool.clear();

	}

	void ICGN2D2::setIteration(float conv_criterion, float stop_condition) {
		this->conv_criterion = conv_criterion;
		this->stop_condition = stop_condition;
	}

	void ICGN2D2::setIteration(POI2D* POI) {
		this->conv_criterion = POI->result.convergence;
		this->stop_condition = POI->result.iteration;
	}

	ICGN2D2_* ICGN2D2::getInstance(int tid)
	{
		if (tid >= (int)this->instance_pool.size()) {
			throw std::string("CPU thread ID over limit");
		}

		return this->instance_pool[tid];
	}

	void ICGN2D2::prepareRef() {
		if (this->ref_gradient != nullptr) {
			delete this->ref_gradient;
			this->ref_gradient = nullptr;
		}

		this->ref_gradient = new Gradient2D4(*this->ref_img);
		this->ref_gradient->getGradientX();
		this->ref_gradient->getGradientY();
	}

	void ICGN2D2::prepareTar() {
		if (this->tar_interp != nullptr) {
			delete this->tar_interp;
			this->tar_interp = nullptr;
		}

		this->tar_interp = new BicubicBspline(*this->tar_img);
		this->tar_interp->prepare();
	}

	void ICGN2D2::prepare() {
		this->prepareRef();
		this->prepareTar();
	}

	void ICGN2D2::compute(POI2D* POI) {
		int subset_width = 2 * this->subset_radius_x + 1;
		int subset_height = 2 * this->subset_radius_y + 1;

		//set instance w.r.t. thread id 
		ICGN2D2_* cur_instance = this->getInstance(omp_get_thread_num());

		//set reference subset
		cur_instance->ref_subset->center = (Point2D)*POI;
		cur_instance->ref_subset->fill(this->ref_img);
		float ref_mean_norm = cur_instance->ref_subset->zeroMeanNorm();

		//build the hessian matrix
		cur_instance->hessian.setZero();
		for (int r = 0; r < subset_height; r++) {
			for (int c = 0; c < subset_width; c++) {
				int x_local = c - this->subset_radius_x;
				int y_local = r - this->subset_radius_y;
				float xx_local = (x_local * x_local) * 0.5f;
				float xy_local = (float)(x_local * y_local);
				float yy_local = (y_local * y_local) * 0.5f;
				int x_global = POI->x + x_local;
				int y_global = POI->y + y_local;
				float ref_gradient_x = this->ref_gradient->gradient_x(y_global, x_global);
				float ref_gradient_y = this->ref_gradient->gradient_y(y_global, x_global);

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
		cur_instance->tar_subset->center = (Point2D)*POI;

		//get initial guess
		Deformation2D1 p_initial(POI->deformation.u, POI->deformation.ux, POI->deformation.uy,
			POI->deformation.v, POI->deformation.vx, POI->deformation.vy);

		//IC-GN iteration
		int iteration = 0; //initialize iteration counter
		Deformation2D2 p_current, p_increment;
		p_current.setDeformation(p_initial);
		float dp_norm_max, ZNSSD;
		Point2D local_coor, warped_coor, global_coor;
		do {
			iteration++;
			//reconstruct target subset
			for (int r = 0; r < subset_height; r++) {
				for (int c = 0; c < subset_width; c++) {
					int x_local = c - this->subset_radius_x;
					int y_local = r - this->subset_radius_y;
					local_coor.x = x_local;
					local_coor.y = y_local;
					warped_coor = p_current.warp(local_coor);
					global_coor = (Point2D)*POI + warped_coor;
					cur_instance->tar_subset->eg_mat(r, c) = this->tar_interp->compute(global_coor);
				}
			}
			float tar_mean_norm = cur_instance->tar_subset->zeroMeanNorm();

			//compute error image
			cur_instance->error_img = cur_instance->tar_subset->eg_mat * (ref_mean_norm / tar_mean_norm)
				- (cur_instance->ref_subset->eg_mat);

			//calculate ZNSSD
			ZNSSD = cur_instance->error_img.squaredNorm() / (ref_mean_norm * ref_mean_norm);

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
			int subset_radius_x2 = this->subset_radius_x * this->subset_radius_x;
			int subset_radius_y2 = this->subset_radius_y * this->subset_radius_y;
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
		} while (iteration < this->stop_condition && dp_norm_max >= this->conv_criterion);

		//store the final result
		POI->deformation.u = p_current.u;
		POI->deformation.ux = p_current.ux;
		POI->deformation.uy = p_current.uy;
		POI->deformation.uxx = p_current.uxx;
		POI->deformation.uyy = p_current.uyy;
		POI->deformation.uxy = p_current.uxy;

		POI->deformation.v = p_current.v;
		POI->deformation.vx = p_current.vx;
		POI->deformation.vy = p_current.vy;
		POI->deformation.vxx = p_current.vxx;
		POI->deformation.vyy = p_current.vyy;
		POI->deformation.vxy = p_current.vxy;

		//save the results for output
		POI->result.u0 = p_initial.u;
		POI->result.v0 = p_initial.v;
		POI->result.u = p_current.u;
		POI->result.v = p_current.v;
		POI->result.ZNCC = 0.5f * (2 - ZNSSD);
		POI->result.iteration = (float)iteration;
		POI->result.convergence = dp_norm_max;
	}

	void ICGN2D2::compute(std::vector<POI2D>& POI_queue) {
#pragma omp parallel for
		for (int i = 0; i < POI_queue.size(); ++i) {
			this->compute(&POI_queue[i]);
		}
	}

}//namespace opencorr