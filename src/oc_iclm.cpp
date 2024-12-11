/*
 * This file is part of OpenCorr, an open source C++ library for
 * study and development of 2D, 3D/stereo and volumetric
 * digital image correlation.
 *
 * Copyright (C) 2021-2024, Zhenyu Jiang <zhenyujiang@scut.edu.cn>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one from http://mozilla.org/MPL/2.0/.
 *
 * More information about OpenCorr can be found at https://www.opencorr.org/
 */

#include "oc_iclm.h"

namespace opencorr
{
	ICLM2D1_* ICLM2D1_::allocate(int subset_radius_x, int subset_radius_y)
	{
		int subset_width = 2 * subset_radius_x + 1;
		int subset_height = 2 * subset_radius_y + 1;
		Point2D subset_center(0, 0);

		ICLM2D1_* ICLM_instance = new ICLM2D1_;
		ICLM_instance->ref_subset = new Subset2D(subset_center, subset_radius_x, subset_radius_y);
		ICLM_instance->tar_subset = new Subset2D(subset_center, subset_radius_x, subset_radius_y);
		ICLM_instance->error_img = Eigen::MatrixXf::Zero(subset_height, subset_width);
		ICLM_instance->sd_img = new3D(subset_height, subset_width, 6);

		return ICLM_instance;
	}

	void ICLM2D1_::release(ICLM2D1_* instance)
	{
		delete3D(instance->sd_img);
		delete instance->ref_subset;
		delete instance->tar_subset;
	}

	void ICLM2D1_::update(ICLM2D1_* instance, int subset_radius_x, int subset_radius_y)
	{
		if (instance->sd_img != nullptr)
		{
			delete3D(instance->sd_img);
			instance->sd_img = nullptr;
		}

		if (instance->ref_subset != nullptr)
		{
			delete instance->ref_subset;
			instance->ref_subset = nullptr;
		}

		if (instance->tar_subset != nullptr)
		{
			delete instance->tar_subset;
			instance->tar_subset = nullptr;
		}

		int subset_width = 2 * subset_radius_x + 1;
		int subset_height = 2 * subset_radius_y + 1;
		Point2D subset_center(0, 0);

		instance->ref_subset = new Subset2D(subset_center, subset_radius_x, subset_radius_y);
		instance->tar_subset = new Subset2D(subset_center, subset_radius_x, subset_radius_y);
		instance->error_img.resize(subset_height, subset_width);
		instance->sd_img = new3D(subset_height, subset_width, 6);
	}

	ICLM2D1_* ICLM2D1::getInstance(int tid)
	{
		if (tid >= (int)instance_pool.size())
		{
			throw std::string("CPU thread ID over limit");
		}

		return instance_pool[tid];
	}

	ICLM2D1::ICLM2D1(int subset_radius_x, int subset_radius_y, float conv_criterion, float stop_condition, int thread_number)
		: ref_gradient(nullptr), tar_interp(nullptr)
	{
		this->subset_radius_x = subset_radius_x;
		this->subset_radius_y = subset_radius_y;
		this->conv_criterion = conv_criterion;
		this->stop_condition = stop_condition;
		this->thread_number = thread_number;

		self_adaptive = false;

		for (int i = 0; i < thread_number; i++)
		{
			ICLM2D1_* instance = ICLM2D1_::allocate(subset_radius_x, subset_radius_y);
			instance_pool.push_back(instance);
		}
	}

	ICLM2D1::~ICLM2D1()
	{
		delete ref_gradient;
		delete tar_interp;

		for (auto& instance : instance_pool)
		{
			ICLM2D1_::release(instance);
			delete instance;
		}
		std::vector<ICLM2D1_*>().swap(instance_pool);
	}

	void ICLM2D1::setIteration(float conv_criterion, float stop_condition)
	{
		this->conv_criterion = conv_criterion;
		this->stop_condition = stop_condition;
	}

	void ICLM2D1::setIteration(POI2D* poi)
	{
		conv_criterion = poi->result.convergence;
		stop_condition = (int)poi->result.iteration;
	}

	void ICLM2D1::setDamping(float lambda, float alpha, float beta)
	{
		damping.lambda = lambda;
		damping.alpha = alpha;
		damping.beta = beta;
	}

	void ICLM2D1::prepareRef()
	{
		if (ref_gradient != nullptr)
		{
			delete ref_gradient;
			ref_gradient = nullptr;
		}

		ref_gradient = new Gradient2D4(*ref_img);
		ref_gradient->getGradientX();
		ref_gradient->getGradientY();
	}

	void ICLM2D1::prepareTar()
	{
		if (tar_interp != nullptr)
		{
			delete tar_interp;
			tar_interp = nullptr;
		}

		tar_interp = new BicubicBspline(*tar_img);
		tar_interp->prepare();
	}

	void ICLM2D1::prepare()
	{
		prepareRef();
		prepareTar();
	}

	void ICLM2D1::compute(POI2D* poi)
	{
		//set instance w.r.t. thread id 
		ICLM2D1_* cur_instance = getInstance(omp_get_thread_num());

		int subset_rx = subset_radius_x;
		int subset_ry = subset_radius_y;

		if (self_adaptive)
		{
			//update the instance according to the subset dimension of current POI
			ICLM2D1_::update(cur_instance, poi->subset_radius.x, poi->subset_radius.y);
			subset_rx = poi->subset_radius.x;
			subset_ry = poi->subset_radius.y;
		}

		if (poi->y - subset_ry < 0 || poi->x - subset_rx < 0
			|| poi->y + subset_ry > ref_img->height - 1 || poi->x + subset_rx > ref_img->width - 1
			|| fabs(poi->deformation.u) >= ref_img->width || fabs(poi->deformation.v) >= ref_img->height
			|| poi->result.zncc < 0 || std::isnan(poi->deformation.u) || std::isnan(poi->deformation.v))
		{
			poi->result.zncc = poi->result.zncc >= 0 ? -3.f : poi->result.zncc;
			return;
		}

		//set subset dimension
		int subset_width = 2 * subset_rx + 1;
		int subset_height = 2 * subset_ry + 1;

		//set reference subset
		cur_instance->ref_subset->center = (Point2D)*poi;
		cur_instance->ref_subset->fill(ref_img);
		float ref_mean_norm = cur_instance->ref_subset->zeroMeanNorm();

		//build the Hessian matrix
		cur_instance->hessian.setZero();
		for (int r = 0; r < subset_height; r++)
		{
			for (int c = 0; c < subset_width; c++)
			{
				int x_local = c - subset_rx;
				int y_local = r - subset_ry;
				int x_global = (int)poi->x + x_local;
				int y_global = (int)poi->y + y_local;
				float ref_gradient_x = ref_gradient->gradient_x(y_global, x_global);
				float ref_gradient_y = ref_gradient->gradient_y(y_global, x_global);

				cur_instance->sd_img[r][c][0] = ref_gradient_x;
				cur_instance->sd_img[r][c][1] = ref_gradient_x * x_local;
				cur_instance->sd_img[r][c][2] = ref_gradient_x * y_local;
				cur_instance->sd_img[r][c][3] = ref_gradient_y;
				cur_instance->sd_img[r][c][4] = ref_gradient_y * x_local;
				cur_instance->sd_img[r][c][5] = ref_gradient_y * y_local;

				for (int i = 0; i < 6; i++)
				{
					for (int j = 0; j < i + 1; j++)
					{
						cur_instance->hessian(i, j) += (cur_instance->sd_img[r][c][i] * cur_instance->sd_img[r][c][j]);
						cur_instance->hessian(j, i) = cur_instance->hessian(i, j);
					}
				}
			}
		}

		//set target subset
		cur_instance->tar_subset->center = (Point2D)*poi;

		//get initial guess
		Deformation2D1 p_initial(poi->deformation.u, poi->deformation.ux, poi->deformation.uy, poi->deformation.v, poi->deformation.vx, poi->deformation.vy);

		//IC-LM iteration
		int iteration_counter = 0; //initialize iteration counter
		Deformation2D1 p_current, p_increment;
		p_current.setDeformation(p_initial);
		Point2D local_coor, warped_coor, global_coor;
		float dp_norm_max, znssd, current_lambda;
		float znssd0 = 4.f;
		Matrix6f identity_mat;
		identity_mat.setIdentity();

		do
		{
			iteration_counter++;

			//reconstruct target subset
			for (int r = 0; r < subset_height; r++)
			{
				for (int c = 0; c < subset_width; c++)
				{
					int x_local = c - subset_rx;
					int y_local = r - subset_ry;
					local_coor.x = x_local;
					local_coor.y = y_local;
					warped_coor = p_current.warp(local_coor);
					global_coor = cur_instance->tar_subset->center + warped_coor;
					cur_instance->tar_subset->eg_mat(r, c) = tar_interp->compute(global_coor);
				}
			}

			float tar_mean_norm = cur_instance->tar_subset->zeroMeanNorm();

			//calculate error image
			cur_instance->error_img = cur_instance->tar_subset->eg_mat * (ref_mean_norm / tar_mean_norm) - (cur_instance->ref_subset->eg_mat);

			//calculate ZNSSD
			znssd = cur_instance->error_img.squaredNorm() / (ref_mean_norm * ref_mean_norm);

			//if it is the first iteration step
			if (iteration_counter == 1)
			{
				//calculate lambda
				current_lambda = powf(damping.lambda, znssd / znssd0) - 1.f;
			}

			//calculate the inversed Hessian matrix
			cur_instance->inv_hessian = (cur_instance->hessian + current_lambda * identity_mat).inverse();

			//calculate numerator
			float numerator[6] = { 0.f };
			for (int r = 0; r < subset_height; r++)
			{
				for (int c = 0; c < subset_width; c++)
				{
					for (int i = 0; i < 6; i++)
					{
						numerator[i] += (cur_instance->sd_img[r][c][i] * cur_instance->error_img(r, c));
					}
				}
			}

			//calculate dp
			float dp[6] = { 0.f };
			for (int i = 0; i < 6; i++)
			{
				for (int j = 0; j < 6; j++)
				{
					dp[i] += (cur_instance->inv_hessian(i, j) * numerator[j]);
				}
			}
			p_increment.setDeformation(dp);

			//update lambda and deformation vector according to current znssd
			if (znssd < znssd0)
			{
				//update lambda
				current_lambda = current_lambda * damping.alpha;

				//update warp
				p_current.warp_matrix = p_current.warp_matrix * p_increment.warp_matrix.inverse();

				//update p
				p_current.setDeformation();

				//update znssd0
				znssd0 = znssd;
			}
			else
			{
				current_lambda = current_lambda * damping.beta;
			}

			//check convergence
			int subset_rx2 = subset_rx * subset_rx;
			int subset_ry2 = subset_ry * subset_ry;

			dp_norm_max = p_increment.u * p_increment.u
				+ p_increment.ux * p_increment.ux * subset_rx2
				+ p_increment.uy * p_increment.uy * subset_ry2
				+ p_increment.v * p_increment.v
				+ p_increment.vx * p_increment.vx * subset_rx2
				+ p_increment.vy * p_increment.vy * subset_ry2;

			dp_norm_max = sqrt(dp_norm_max);
		} while (iteration_counter < stop_condition && dp_norm_max >= conv_criterion);

		//store the final result
		poi->deformation.u = p_current.u;
		poi->deformation.ux = p_current.ux;
		poi->deformation.uy = p_current.uy;
		poi->deformation.v = p_current.v;
		poi->deformation.vx = p_current.vx;
		poi->deformation.vy = p_current.vy;

		//save the parameters for output
		poi->result.u0 = p_initial.u;
		poi->result.v0 = p_initial.v;
		poi->result.zncc = 0.5f * (2 - znssd);
		poi->result.iteration = (float)iteration_counter;
		poi->result.convergence = dp_norm_max;

		//store the subset size
		poi->subset_radius.x = subset_rx;
		poi->subset_radius.y = subset_ry;

		//check if the iteration converge at the desired target
		if (poi->result.convergence >= conv_criterion && poi->result.iteration >= stop_condition)
		{
			poi->result.zncc = -4.f;
		}

		//check if the case of NaN occurs for ZNCC or displacments
		if (std::isnan(poi->result.zncc) || std::isnan(poi->deformation.u) || std::isnan(poi->deformation.v))
		{
			poi->deformation.u = poi->result.u0;
			poi->deformation.v = poi->result.v0;
			poi->result.zncc = -5.f;
		}
	}

	void ICLM2D1::compute(std::vector<POI2D>& poi_queue)
	{
		auto queue_length = poi_queue.size();
#pragma omp parallel for
		for (int i = 0; i < queue_length; i++)
		{
			compute(&poi_queue[i]);
		}
	}




	ICLM2D2_* ICLM2D2_::allocate(int subset_radius_x, int subset_radius_y)
	{
		int subset_width = 2 * subset_radius_x + 1;
		int subset_height = 2 * subset_radius_y + 1;
		Point2D subset_center(0, 0);

		ICLM2D2_* ICLM_instance = new ICLM2D2_;
		ICLM_instance->ref_subset = new Subset2D(subset_center, subset_radius_x, subset_radius_y);
		ICLM_instance->tar_subset = new Subset2D(subset_center, subset_radius_x, subset_radius_y);
		ICLM_instance->error_img = Eigen::MatrixXf::Zero(subset_height, subset_width);
		ICLM_instance->sd_img = new3D(subset_height, subset_width, 12);

		return ICLM_instance;
	}

	void ICLM2D2_::release(ICLM2D2_* instance)
	{
		delete3D(instance->sd_img);
		delete instance->ref_subset;
		delete instance->tar_subset;
	}

	void ICLM2D2_::update(ICLM2D2_* instance, int subset_radius_x, int subset_radius_y)
	{
		if (instance->sd_img != nullptr)
		{
			delete3D(instance->sd_img);
			instance->sd_img = nullptr;
		}

		if (instance->ref_subset != nullptr)
		{
			delete instance->ref_subset;
			instance->ref_subset = nullptr;
		}

		if (instance->tar_subset != nullptr)
		{
			delete instance->tar_subset;
			instance->tar_subset = nullptr;
		}

		int subset_width = 2 * subset_radius_x + 1;
		int subset_height = 2 * subset_radius_y + 1;
		Point2D subset_center(0, 0);

		instance->ref_subset = new Subset2D(subset_center, subset_radius_x, subset_radius_y);
		instance->tar_subset = new Subset2D(subset_center, subset_radius_x, subset_radius_y);
		instance->error_img.resize(subset_height, subset_width);
		instance->sd_img = new3D(subset_height, subset_width, 12);
	}

	ICLM2D2_* ICLM2D2::getInstance(int tid)
	{
		if (tid >= (int)instance_pool.size())
		{
			throw std::string("CPU thread ID over limit");
		}

		return instance_pool[tid];
	}

	ICLM2D2::ICLM2D2(int subset_radius_x, int subset_radius_y, float conv_criterion, float stop_condition, int thread_number)
		: ref_gradient(nullptr), tar_interp(nullptr)
	{
		this->subset_radius_x = subset_radius_x;
		this->subset_radius_y = subset_radius_y;
		this->conv_criterion = conv_criterion;
		this->stop_condition = stop_condition;
		this->thread_number = thread_number;

		self_adaptive = false;

		for (int i = 0; i < thread_number; i++)
		{
			ICLM2D2_* instance = ICLM2D2_::allocate(subset_radius_x, subset_radius_y);
			instance_pool.push_back(instance);
		}
	}

	ICLM2D2::~ICLM2D2()
	{
		delete ref_gradient;
		delete tar_interp;

		for (auto& instance : instance_pool)
		{
			ICLM2D2_::release(instance);
			delete instance;
		}
		std::vector<ICLM2D2_*>().swap(instance_pool);
	}

	void ICLM2D2::setIteration(float conv_criterion, float stop_condition)
	{
		this->conv_criterion = conv_criterion;
		this->stop_condition = stop_condition;
	}

	void ICLM2D2::setIteration(POI2D* poi)
	{
		conv_criterion = poi->result.convergence;
		stop_condition = poi->result.iteration;
	}

	void ICLM2D2::setDamping(float lambda, float alpha, float beta)
	{
		damping.lambda = lambda;
		damping.alpha = alpha;
		damping.beta = beta;
	}

	void ICLM2D2::prepareRef()
	{
		if (ref_gradient != nullptr)
		{
			delete ref_gradient;
			ref_gradient = nullptr;
		}

		ref_gradient = new Gradient2D4(*ref_img);
		ref_gradient->getGradientX();
		ref_gradient->getGradientY();
	}

	void ICLM2D2::prepareTar()
	{
		if (tar_interp != nullptr)
		{
			delete tar_interp;
			tar_interp = nullptr;
		}

		tar_interp = new BicubicBspline(*tar_img);
		tar_interp->prepare();
	}

	void ICLM2D2::prepare()
	{
		prepareRef();
		prepareTar();
	}

	void ICLM2D2::compute(POI2D* poi)
	{
		//set instance w.r.t. thread id 
		ICLM2D2_* cur_instance = getInstance(omp_get_thread_num());

		int subset_rx = subset_radius_x;
		int subset_ry = subset_radius_y;

		if (self_adaptive)
		{
			//update the instance according to the subset dimension of current POI
			ICLM2D2_::update(cur_instance, poi->subset_radius.x, poi->subset_radius.y);
			subset_rx = poi->subset_radius.x;
			subset_ry = poi->subset_radius.y;
		}

		if (poi->y - subset_ry < 0 || poi->x - subset_rx < 0
			|| poi->y + subset_ry > ref_img->height - 1 || poi->x + subset_rx > ref_img->width - 1
			|| fabs(poi->deformation.u) >= ref_img->width || fabs(poi->deformation.v) >= ref_img->height
			|| poi->result.zncc < 0 || std::isnan(poi->deformation.u) || std::isnan(poi->deformation.v))
		{
			poi->result.zncc = poi->result.zncc >= 0 ? -3.f : poi->result.zncc;
			return;
		}
		int subset_width = 2 * subset_rx + 1;
		int subset_height = 2 * subset_ry + 1;

		//set reference subset
		cur_instance->ref_subset->center = (Point2D)*poi;
		cur_instance->ref_subset->fill(ref_img);
		float ref_mean_norm = cur_instance->ref_subset->zeroMeanNorm();

		//build the Hessian matrix
		cur_instance->hessian.setZero();
		for (int r = 0; r < subset_height; r++)
		{
			for (int c = 0; c < subset_width; c++)
			{
				int x_local = c - subset_rx;
				int y_local = r - subset_ry;
				float xx_local = (x_local * x_local) * 0.5f;
				float xy_local = (float)(x_local * y_local);
				float yy_local = (y_local * y_local) * 0.5f;
				int x_global = (int)poi->x + x_local;
				int y_global = (int)poi->y + y_local;
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

				for (int i = 0; i < 12; i++)
				{
					for (int j = 0; j < i + 1; j++)
					{
						cur_instance->hessian(i, j) += (cur_instance->sd_img[r][c][i] * cur_instance->sd_img[r][c][j]);
						cur_instance->hessian(j, i) = cur_instance->hessian(i, j);
					}
				}
			}
		}

		//set target subset
		cur_instance->tar_subset->center = (Point2D)*poi;

		//get initial guess
		Deformation2D1 p_initial(poi->deformation.u, poi->deformation.ux, poi->deformation.uy,
			poi->deformation.v, poi->deformation.vx, poi->deformation.vy);

		//IC-LM iteration
		int iteration_counter = 0; //initialize iteration counter
		Deformation2D2 p_current, p_increment;
		p_current.setDeformation(p_initial);
		Point2D local_coor, warped_coor, global_coor;
		float dp_norm_max, znssd, current_lambda;
		float znssd0 = 4.f;
		Matrix12f identity_mat;
		identity_mat.setIdentity();

		do
		{
			iteration_counter++;
			//reconstruct target subset
			for (int r = 0; r < subset_height; r++)
			{
				for (int c = 0; c < subset_width; c++)
				{
					int x_local = c - subset_rx;
					int y_local = r - subset_ry;
					local_coor.x = x_local;
					local_coor.y = y_local;
					warped_coor = p_current.warp(local_coor);
					global_coor = cur_instance->tar_subset->center + warped_coor;
					cur_instance->tar_subset->eg_mat(r, c) = tar_interp->compute(global_coor);
				}
			}
			float tar_mean_norm = cur_instance->tar_subset->zeroMeanNorm();

			//calculate error image
			cur_instance->error_img = cur_instance->tar_subset->eg_mat * (ref_mean_norm / tar_mean_norm) - (cur_instance->ref_subset->eg_mat);

			//calculate ZNSSD
			znssd = cur_instance->error_img.squaredNorm() / (ref_mean_norm * ref_mean_norm);

			//if it is the first iteration step
			if (iteration_counter == 1)
			{
				//calculate lambda
				current_lambda = powf(damping.lambda, znssd / znssd0) - 1;
			}

			//calculate the inversed Hessian matrix
			cur_instance->inv_hessian = (cur_instance->hessian + current_lambda * identity_mat).inverse();

			//calculate numerator
			float numerator[12] = { 0.f };
			for (int r = 0; r < subset_height; r++)
			{
				for (int c = 0; c < subset_width; c++)
				{
					for (int i = 0; i < 12; i++)
					{
						numerator[i] += (cur_instance->sd_img[r][c][i] * cur_instance->error_img(r, c));
					}
				}
			}

			//calculate dp
			float dp[12] = { 0.f };
			for (int i = 0; i < 12; i++)
			{
				for (int j = 0; j < 12; j++)
				{
					dp[i] += (cur_instance->inv_hessian(i, j) * numerator[j]);
				}
			}
			p_increment.setDeformation(dp);

			//update lambda and deformation vector according to current znssd
			if (znssd < znssd0)
			{
				//update lambda
				current_lambda = current_lambda * damping.alpha;

				//update warp
				p_current.warp_matrix = p_current.warp_matrix * p_increment.warp_matrix.inverse();

				//update p
				p_current.setDeformation();

				//update znssd0
				znssd0 = znssd;
			}
			else
			{
				current_lambda = current_lambda * damping.beta;
			}

			//check convergence
			int subset_rx2 = subset_rx * subset_rx;
			int subset_ry2 = subset_ry * subset_ry;
			int subset_radius_xy2 = subset_rx2 * subset_ry2;

			dp_norm_max = p_increment.u * p_increment.u
				+ p_increment.ux * p_increment.ux * subset_rx2
				+ p_increment.uy * p_increment.uy * subset_ry2
				+ p_increment.uxx * p_increment.uxx * subset_rx2 * subset_rx2 * 0.25f
				+ p_increment.uyy * p_increment.uyy * subset_ry2 * subset_ry2 * 0.25f
				+ p_increment.uxy * p_increment.uxy * subset_radius_xy2
				+ p_increment.v * p_increment.v
				+ p_increment.vx * p_increment.vx * subset_rx2
				+ p_increment.vy * p_increment.vy * subset_ry2
				+ p_increment.vxx * p_increment.vxx * subset_rx2 * subset_rx2 * 0.25f
				+ p_increment.vyy * p_increment.vyy * subset_ry2 * subset_ry2 * 0.25f
				+ p_increment.vxy * p_increment.vxy * subset_radius_xy2;

			dp_norm_max = sqrt(dp_norm_max);
		} while (iteration_counter < stop_condition && dp_norm_max >= conv_criterion);

		//store the final result
		poi->deformation.u = p_current.u;
		poi->deformation.ux = p_current.ux;
		poi->deformation.uy = p_current.uy;
		poi->deformation.uxx = p_current.uxx;
		poi->deformation.uxy = p_current.uxy;
		poi->deformation.uyy = p_current.uyy;

		poi->deformation.v = p_current.v;
		poi->deformation.vx = p_current.vx;
		poi->deformation.vy = p_current.vy;
		poi->deformation.vxx = p_current.vxx;
		poi->deformation.vxy = p_current.vxy;
		poi->deformation.vyy = p_current.vyy;

		//save the parameters for output
		poi->result.u0 = p_initial.u;
		poi->result.v0 = p_initial.v;
		poi->result.zncc = 0.5f * (2 - znssd);
		poi->result.iteration = (float)iteration_counter;
		poi->result.convergence = dp_norm_max;

		//store the subset size
		poi->subset_radius.x = subset_rx;
		poi->subset_radius.y = subset_ry;

		//check if the iteration converge at the desired target
		if (poi->result.convergence >= conv_criterion && poi->result.iteration >= stop_condition)
		{
			poi->result.zncc = -4.f;
		}

		//check if the case of NaN occurs for ZNCC or displacments
		if (std::isnan(poi->result.zncc) || std::isnan(poi->deformation.u) || std::isnan(poi->deformation.v))
		{
			poi->deformation.u = poi->result.u0;
			poi->deformation.v = poi->result.v0;
			poi->result.zncc = -5.f;
		}
	}

	void ICLM2D2::compute(std::vector<POI2D>& poi_queue)
	{
		auto queue_length = poi_queue.size();
#pragma omp parallel for
		for (int i = 0; i < queue_length; i++)
		{
			compute(&poi_queue[i]);
		}
	}

}//namespace opencorr