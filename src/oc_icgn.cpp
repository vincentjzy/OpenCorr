/*
 * This file is part of OpenCorr, an open-source C++ library for
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
		ICGN_instance->steepest_descent_img = new3D(subset_height, subset_width, 6);

		return ICGN_instance;
	}

	void ICGN2D1_::release(ICGN2D1_* instance) {
		delete instance->ref_subset;
		delete instance->tar_subset;
		delete3D(instance->steepest_descent_img);
	}

	ICGN2D1::ICGN2D1() :ref_gradient(nullptr), tar_interp(nullptr) {
	}

	ICGN2D1::ICGN2D1(int subset_radius_x, int subset_radius_y, float convergence_criterion, float stop_condition)
		: ref_gradient(nullptr), tar_interp(nullptr) {

		this->subset_radius_x = subset_radius_x;
		this->subset_radius_y = subset_radius_y;
		this->convergence_criterion = convergence_criterion;
		this->stop_condition = stop_condition;
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

	void ICGN2D1::setSubsetRadii(int subset_radius_x, int subset_radius_y) {
		this->subset_radius_x = subset_radius_x;
		this->subset_radius_y = subset_radius_y;
	}

	void ICGN2D1::setIteration(float convergence_criterion, float stop_condition) {
		this->convergence_criterion = convergence_criterion;
		this->stop_condition = stop_condition;
	}

	void ICGN2D1::setIteration(POI2D* POI) {
		this->convergence_criterion = POI->result.convergence;
		this->stop_condition = (int)POI->result.iteration;
	}

	ICGN2D1_* ICGN2D1::getInstance(int tid)
	{
		if (tid >= (int)this->instance_pool.size()) {
			throw std::exception(std::string("CPU thread ID over limit").data());
		}

		return this->instance_pool[tid];
	}

	void ICGN2D1::prepare() {
		if (this->ref_gradient != nullptr) {
			delete this->ref_gradient;
			this->ref_gradient = nullptr;
		}
		if (this->tar_interp != nullptr) {
			delete this->tar_interp;
			this->tar_interp = nullptr;
		}

		this->ref_gradient = new Gradient2D4(*this->ref_img);
		this->ref_gradient->getGradientX();
		this->ref_gradient->getGradientY();

		this->tar_interp = new BicubicBspline(*this->tar_img);
		this->tar_interp->prepare();

		for (int i = 0; i < this->CPU_thread_number; i++) {
			ICGN2D1_* instance = ICGN2D1_::allocate(this->subset_radius_x, this->subset_radius_y);
			this->instance_pool.push_back(instance);
		}

	}

	void ICGN2D1::compute(POI2D* POI) {
		int subset_width = 2 * this->subset_radius_x + 1;
		int subset_height = 2 * this->subset_radius_y + 1;

		//set instance w.r.t. thread id 
		ICGN2D1_* current_instance = this->getInstance(omp_get_thread_num());

		//set reference subset
		current_instance->ref_subset->center = (Point2D)*POI;
		current_instance->ref_subset->fill(this->ref_img);
		float ref_mean_norm = current_instance->ref_subset->zeroMeanNorm();

		//build the hessian matrix
		current_instance->hessian.setZero();
		for (int r = 0; r < subset_height; r++) {
			for (int c = 0; c < subset_width; c++) {
				int x_local = c - this->subset_radius_x;
				int y_local = r - this->subset_radius_y;
				int x_global = POI->x + x_local;
				int y_global = POI->y + y_local;
				float ref_gradient_x = this->ref_gradient->gradient_x(y_global, x_global);
				float ref_gradient_y = this->ref_gradient->gradient_y(y_global, x_global);

				current_instance->steepest_descent_img[r][c][0] = ref_gradient_x;
				current_instance->steepest_descent_img[r][c][1] = ref_gradient_x * x_local;
				current_instance->steepest_descent_img[r][c][2] = ref_gradient_x * y_local;
				current_instance->steepest_descent_img[r][c][3] = ref_gradient_y;
				current_instance->steepest_descent_img[r][c][4] = ref_gradient_y * x_local;
				current_instance->steepest_descent_img[r][c][5] = ref_gradient_y * y_local;

				for (int i = 0; i < 6; i++) {
					float sum = 0;
					for (int j = 0; j < 6; j++) {
						sum = current_instance->hessian(i, j) + current_instance->steepest_descent_img[r][c][i] * current_instance->steepest_descent_img[r][c][j];
						current_instance->hessian(i, j) = sum;
					}
				}
			}
		}

		//compute inversed hessian matrix
		current_instance->inv_hessian = current_instance->hessian.inverse();

		//set target subset
		current_instance->tar_subset->center = (Point2D)*POI;

		//get initial guess
		current_instance->p_initial.setDeformation(POI->deformation.u, POI->deformation.ux, POI->deformation.uy,
			POI->deformation.v, POI->deformation.vx, POI->deformation.vy);

		//IC-GN iteration
		int iteration = 0; //initialize iteration counter
		current_instance->p_current.setDeformation(current_instance->p_initial);
		float max_deformation_norm, ZNSSD;
		Point2D local_coordinate, warped_coordinate, global_coordinate;
		do {
			iteration++;
			//reconstruct target subset
			for (int r = 0; r < subset_height; r++) {
				for (int c = 0; c < subset_width; c++) {
					int x_local = c - this->subset_radius_x;
					int y_local = r - this->subset_radius_y;
					local_coordinate.x = x_local;
					local_coordinate.y = y_local;
					warped_coordinate = current_instance->p_current.warp(local_coordinate);
					global_coordinate = (Point2D)*POI + warped_coordinate;
					current_instance->tar_subset->eg_mat(r, c) = this->tar_interp->compute(global_coordinate);
				}
			}
			float tar_mean_norm = current_instance->tar_subset->zeroMeanNorm();

			//compute error image
			current_instance->error_img = current_instance->tar_subset->eg_mat * (ref_mean_norm / tar_mean_norm)
				- (current_instance->ref_subset->eg_mat);

			//calculate ZNSSD
			ZNSSD = current_instance->error_img.squaredNorm() / (ref_mean_norm * ref_mean_norm);

			//compute numerator
			float numerator[6] = { 0 };
			for (int r = 0; r < subset_height; r++) {
				for (int c = 0; c < subset_width; c++) {
					for (int i = 0; i < 6; i++) {
						numerator[i] += current_instance->steepest_descent_img[r][c][i] * current_instance->error_img(r, c);
					}
				}
			}

			//compute dp
			float dp[6] = { 0 };
			for (int i = 0; i < 6; i++) {
				for (int j = 0; j < 6; j++) {
					dp[i] += current_instance->inv_hessian(i, j) * numerator[j];
				}
			}
			current_instance->p_increment.setDeformation(dp);

			//update warp
			current_instance->p_current.warp_matrix = current_instance->p_current.warp_matrix * current_instance->p_increment.warp_matrix.inverse();

			//update p
			current_instance->p_current.setDeformation();

			//check convergence
			int subset_radius_x2 = this->subset_radius_x * this->subset_radius_x;
			int subset_radius_y2 = this->subset_radius_y * this->subset_radius_y;

			max_deformation_norm = 0.f;
			max_deformation_norm += current_instance->p_increment.u * current_instance->p_increment.u;
			max_deformation_norm += current_instance->p_increment.ux * current_instance->p_increment.ux * subset_radius_x2;
			max_deformation_norm += current_instance->p_increment.uy * current_instance->p_increment.uy * subset_radius_y2;
			max_deformation_norm += current_instance->p_increment.v * current_instance->p_increment.v;
			max_deformation_norm += current_instance->p_increment.vx * current_instance->p_increment.vx * subset_radius_x2;
			max_deformation_norm += current_instance->p_increment.vy * current_instance->p_increment.vy * subset_radius_y2;

			max_deformation_norm = sqrtf(max_deformation_norm);
		} while (iteration < this->stop_condition && max_deformation_norm >= this->convergence_criterion);

		//store the final result
		POI->deformation.u = current_instance->p_current.u;
		POI->deformation.ux = current_instance->p_current.ux;
		POI->deformation.uy = current_instance->p_current.uy;
		POI->deformation.v = current_instance->p_current.v;
		POI->deformation.vx = current_instance->p_current.vx;
		POI->deformation.vy = current_instance->p_current.vy;

		//save the results for output
		POI->result.u0 = current_instance->p_initial.u;
		POI->result.v0 = current_instance->p_initial.v;
		POI->result.u = current_instance->p_current.u;
		POI->result.v = current_instance->p_current.v;
		POI->result.ZNCC = 0.5f * (2 - ZNSSD);
		POI->result.iteration = (float)iteration;
		POI->result.convergence = max_deformation_norm;

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
		ICGN_instance->steepest_descent_img = new3D(subset_height, subset_width, 12);

		return ICGN_instance;
	}

	void ICGN2D2_::release(ICGN2D2_* instance) {
		delete instance->ref_subset;
		delete instance->tar_subset;
		delete3D(instance->steepest_descent_img);
	}

	ICGN2D2::ICGN2D2() :ref_gradient(nullptr), tar_interp(nullptr) {
	}

	ICGN2D2::ICGN2D2(int subset_radius_x, int subset_radius_y, float convergence_criterion, float stop_condition)
		: ref_gradient(nullptr), tar_interp(nullptr) {
		this->subset_radius_x = subset_radius_x;
		this->subset_radius_y = subset_radius_y;
		this->convergence_criterion = convergence_criterion;
		this->stop_condition = stop_condition;
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

	void ICGN2D2::setSubsetRadii(int subset_radius_x, int subset_radius_y) {
		this->subset_radius_x = subset_radius_x;
		this->subset_radius_y = subset_radius_y;
	}

	void ICGN2D2::setIteration(float convergence_criterion, float stop_condition) {
		this->convergence_criterion = convergence_criterion;
		this->stop_condition = stop_condition;
	}

	void ICGN2D2::setIteration(POI2D* POI) {
		this->convergence_criterion = POI->result.convergence;
		this->stop_condition = POI->result.iteration;
	}

	ICGN2D2_* ICGN2D2::getInstance(int tid)
	{
		if (tid >= (int)this->instance_pool.size()) {
			throw std::exception(std::string("CPU thread ID over limit").data());
		}

		return this->instance_pool[tid];
	}

	void ICGN2D2::prepare() {
		if (this->ref_gradient != nullptr) {
			delete this->ref_gradient;
			this->ref_gradient = nullptr;
		}
		if (this->tar_interp != nullptr) {
			delete this->tar_interp;
			this->tar_interp = nullptr;
		}

		this->ref_gradient = new Gradient2D4(*this->ref_img);
		this->ref_gradient->getGradientX();
		this->ref_gradient->getGradientY();

		this->tar_interp = new BicubicBspline(*this->tar_img);
		this->tar_interp->prepare();

		for (int i = 0; i < this->CPU_thread_number; i++) {
			ICGN2D2_* instance = ICGN2D2_::allocate(this->subset_radius_x, this->subset_radius_y);
			instance_pool.push_back(instance);
		}
	}

	void ICGN2D2::compute(POI2D* POI) {
		int subset_width = 2 * this->subset_radius_x + 1;
		int subset_height = 2 * this->subset_radius_y + 1;

		//set instance w.r.t. thread id 
		ICGN2D2_* current_instance = this->getInstance(omp_get_thread_num());

		//set reference subset
		current_instance->ref_subset->center = (Point2D)*POI;
		current_instance->ref_subset->fill(this->ref_img);
		float ref_mean_norm = current_instance->ref_subset->zeroMeanNorm();

		//build the hessian matrix
		current_instance->hessian.setZero();
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

				current_instance->steepest_descent_img[r][c][0] = ref_gradient_x;
				current_instance->steepest_descent_img[r][c][1] = ref_gradient_x * x_local;
				current_instance->steepest_descent_img[r][c][2] = ref_gradient_x * y_local;
				current_instance->steepest_descent_img[r][c][3] = ref_gradient_x * xx_local;
				current_instance->steepest_descent_img[r][c][4] = ref_gradient_x * xy_local;
				current_instance->steepest_descent_img[r][c][5] = ref_gradient_x * yy_local;

				current_instance->steepest_descent_img[r][c][6] = ref_gradient_y;
				current_instance->steepest_descent_img[r][c][7] = ref_gradient_y * x_local;
				current_instance->steepest_descent_img[r][c][8] = ref_gradient_y * y_local;
				current_instance->steepest_descent_img[r][c][9] = ref_gradient_y * xx_local;
				current_instance->steepest_descent_img[r][c][10] = ref_gradient_y * xy_local;
				current_instance->steepest_descent_img[r][c][11] = ref_gradient_y * yy_local;

				for (int i = 0; i < 12; i++) {
					float sum = 0;
					for (int j = 0; j < 12; j++) {
						sum = current_instance->hessian(i, j) + current_instance->steepest_descent_img[r][c][i] * current_instance->steepest_descent_img[r][c][j];
						current_instance->hessian(i, j) = sum;
					}
				}
			}
		}

		//compute inversed hessian matrix
		current_instance->inv_hessian = current_instance->hessian.inverse();

		//set target subset
		current_instance->tar_subset->center = (Point2D)*POI;

		//get initial guess
		current_instance->p_initial.setDeformation(POI->deformation.u, POI->deformation.ux, POI->deformation.uy,
			POI->deformation.v, POI->deformation.vx, POI->deformation.vy);

		//IC-GN iteration
		int iteration = 0; //initialize iteration counter
		current_instance->p_current.setDeformation(current_instance->p_initial);
		float max_deformation_norm, ZNSSD;
		Point2D local_coordinate, warped_coordinate, global_coordinate;
		do {
			iteration++;
			//reconstruct target subset
			for (int r = 0; r < subset_height; r++) {
				for (int c = 0; c < subset_width; c++) {
					int x_local = c - this->subset_radius_x;
					int y_local = r - this->subset_radius_y;
					local_coordinate.x = x_local;
					local_coordinate.y = y_local;
					warped_coordinate = current_instance->p_current.warp(local_coordinate);
					global_coordinate = (Point2D)*POI + warped_coordinate;
					current_instance->tar_subset->eg_mat(r, c) = this->tar_interp->compute(global_coordinate);
				}
			}
			float tar_mean_norm = current_instance->tar_subset->zeroMeanNorm();

			//compute error image
			current_instance->error_img = current_instance->tar_subset->eg_mat * (ref_mean_norm / tar_mean_norm)
				- (current_instance->ref_subset->eg_mat);

			//calculate ZNSSD
			ZNSSD = current_instance->error_img.squaredNorm() / (ref_mean_norm * ref_mean_norm);

			//compute numerator
			float numerator[12] = { 0 };
			for (int r = 0; r < subset_height; r++) {
				for (int c = 0; c < subset_width; c++) {
					for (int i = 0; i < 12; i++) {
						numerator[i] += current_instance->steepest_descent_img[r][c][i] * current_instance->error_img(r, c);
					}
				}
			}

			//compute dp
			float dp[12] = { 0 };
			for (int i = 0; i < 12; i++) {
				for (int j = 0; j < 12; j++) {
					dp[i] += current_instance->inv_hessian(i, j) * numerator[j];
				}
			}
			current_instance->p_increment.setDeformation(dp);

			//update warp
			current_instance->p_current.warp_matrix = current_instance->p_current.warp_matrix * current_instance->p_increment.warp_matrix.inverse();

			//update p
			current_instance->p_current.setDeformation();

			//check convergence
			int subset_radius_x2 = this->subset_radius_x * this->subset_radius_x;
			int subset_radius_y2 = this->subset_radius_y * this->subset_radius_y;
			int subset_radius_xy = subset_radius_x2 * subset_radius_y2;

			max_deformation_norm = 0.f;
			max_deformation_norm += current_instance->p_increment.u * current_instance->p_increment.u;
			max_deformation_norm += current_instance->p_increment.ux * current_instance->p_increment.ux * subset_radius_x2;
			max_deformation_norm += current_instance->p_increment.uy * current_instance->p_increment.uy * subset_radius_y2;
			max_deformation_norm += current_instance->p_increment.uxx * current_instance->p_increment.uxx * subset_radius_x2 * subset_radius_x2 / 4.f;
			max_deformation_norm += current_instance->p_increment.uyy * current_instance->p_increment.uyy * subset_radius_y2 * subset_radius_y2 / 4.f;
			max_deformation_norm += current_instance->p_increment.uxy * current_instance->p_increment.uxy * subset_radius_xy;

			max_deformation_norm += current_instance->p_increment.v * current_instance->p_increment.v;
			max_deformation_norm += current_instance->p_increment.vx * current_instance->p_increment.vx * subset_radius_x2;
			max_deformation_norm += current_instance->p_increment.vy * current_instance->p_increment.vy * subset_radius_y2;
			max_deformation_norm += current_instance->p_increment.vxx * current_instance->p_increment.vxx * subset_radius_x2 * subset_radius_x2 / 4.f;
			max_deformation_norm += current_instance->p_increment.vyy * current_instance->p_increment.vyy * subset_radius_y2 * subset_radius_y2 / 4.f;
			max_deformation_norm += current_instance->p_increment.vxy * current_instance->p_increment.vxy * subset_radius_xy;

			max_deformation_norm = sqrtf(max_deformation_norm);
		} while (iteration < this->stop_condition && max_deformation_norm >= this->convergence_criterion);

		//store the final result
		POI->deformation.u = current_instance->p_current.u;
		POI->deformation.ux = current_instance->p_current.ux;
		POI->deformation.uy = current_instance->p_current.uy;
		POI->deformation.uxx = current_instance->p_current.uxx;
		POI->deformation.uyy = current_instance->p_current.uyy;
		POI->deformation.uxy = current_instance->p_current.uxy;

		POI->deformation.v = current_instance->p_current.v;
		POI->deformation.vx = current_instance->p_current.vx;
		POI->deformation.vy = current_instance->p_current.vy;
		POI->deformation.vxx = current_instance->p_current.vxx;
		POI->deformation.vyy = current_instance->p_current.vyy;
		POI->deformation.vxy = current_instance->p_current.vxy;

		//save the results for output
		POI->result.u0 = current_instance->p_initial.u;
		POI->result.v0 = current_instance->p_initial.v;
		POI->result.u = current_instance->p_current.u;
		POI->result.v = current_instance->p_current.v;
		POI->result.ZNCC = 0.5f * (2 - ZNSSD);
		POI->result.iteration = (float)iteration;
		POI->result.convergence = max_deformation_norm;

	}

}//namespace opencorr