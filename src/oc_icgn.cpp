/*
 * This file is part of OpenCorr, an open source C++ library for
 * study and development of 2D, 3D/stereo and volumetric
 * digital image correlation.
 *
 * Copyright (C) 2021-2025, Zhenyu Jiang <zhenyujiang@scut.edu.cn>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one from http://mozilla.org/MPL/2.0/.
 *
 * More information about OpenCorr can be found at https://www.opencorr.org/
 */

#include <omp.h>
 
#include "oc_icgn.h"

namespace opencorr
{
	std::unique_ptr<ICGN2D1_> ICGN2D1_::allocate(int subset_radius_x, int subset_radius_y)
	{
		int subset_width = 2 * subset_radius_x + 1;
		int subset_height = 2 * subset_radius_y + 1;
		Point2D subset_center(0, 0);

		std::unique_ptr<ICGN2D1_> ICGN_instance = std::make_unique<ICGN2D1_>();
		ICGN_instance->ref_subset = std::make_unique<Subset2D>(subset_center, subset_radius_x, subset_radius_y);
		ICGN_instance->tar_subset = std::make_unique<Subset2D>(subset_center, subset_radius_x, subset_radius_y);
		ICGN_instance->error_img = Eigen::MatrixXf::Zero(subset_height, subset_width);
		ICGN_instance->sd_img = new3D(subset_height, subset_width, 6);

		return ICGN_instance;
	}

	void ICGN2D1_::release(std::unique_ptr<ICGN2D1_>& instance)
	{
		if (instance->sd_img != nullptr)
		{
			delete3D(instance->sd_img);
		}

		instance->ref_subset.reset();
		instance->tar_subset.reset();
	}

	void ICGN2D1_::update(std::unique_ptr<ICGN2D1_>& instance, int subset_radius_x, int subset_radius_y)
	{
		release(instance);

		int subset_width = 2 * subset_radius_x + 1;
		int subset_height = 2 * subset_radius_y + 1;
		Point2D subset_center(0, 0);

		instance->ref_subset = std::make_unique<Subset2D>(subset_center, subset_radius_x, subset_radius_y);
		instance->tar_subset = std::make_unique<Subset2D>(subset_center, subset_radius_x, subset_radius_y);
		instance->error_img.resize(subset_height, subset_width);
		instance->sd_img = new3D(subset_height, subset_width, 6);
	}

	std::unique_ptr<ICGN2D1_>& ICGN2D1::getInstance(int tid)
	{
		if (tid >= (int)instance_pool.size())
		{
			throw std::string("CPU thread ID over limit");
		}

		return instance_pool[tid];
	}

	ICGN2D1::ICGN2D1(int subset_radius_x, int subset_radius_y, float conv_criterion, float stop_condition, int thread_number)
		: ref_gradient(nullptr), tar_interp(nullptr)
	{
		this->subset_radius_x = subset_radius_x;
		this->subset_radius_y = subset_radius_y;
		this->conv_criterion = conv_criterion;
		this->stop_condition = stop_condition;
		this->thread_number = thread_number;

		self_adaptive = false;

		instance_pool.resize(thread_number);
#pragma omp parallel for
		for (int i = 0; i < thread_number; i++)
		{
			instance_pool[i] = ICGN2D1_::allocate(subset_radius_x, subset_radius_y);
		}
	}

	ICGN2D1::~ICGN2D1()
	{
		ref_gradient.reset();
		tar_interp.reset();

		for (auto& instance : instance_pool)
		{
			ICGN2D1_::release(instance);
			instance.reset();
		}
		std::vector<std::unique_ptr<ICGN2D1_>>().swap(instance_pool);
	}

	void ICGN2D1::setIteration(float conv_criterion, float stop_condition)
	{
		this->conv_criterion = conv_criterion;
		this->stop_condition = stop_condition;
	}

	void ICGN2D1::setIteration(POI2D* poi)
	{
		conv_criterion = poi->result.convergence;
		stop_condition = (int)poi->result.iteration;
	}

	void ICGN2D1::prepareRef()
	{
		if (ref_gradient != nullptr)
		{
			ref_gradient.reset();
		}

		ref_gradient = std::make_unique<Gradient2D4>(*ref_img);
		ref_gradient->getGradientX();
		ref_gradient->getGradientY();
	}

	void ICGN2D1::prepareTar()
	{
		if (tar_interp != nullptr)
		{
			tar_interp.reset();
		}

		tar_interp = std::make_unique<BicubicBspline>(*tar_img);
		tar_interp->prepare();
	}

	void ICGN2D1::prepare()
	{
		prepareRef();
		prepareTar();
	}

	void ICGN2D1::compute(POI2D* poi)
	{
		//set instance w.r.t. thread id 
		std::unique_ptr<ICGN2D1_>& icgn = getInstance(omp_get_thread_num());

		int subset_rx = subset_radius_x;
		int subset_ry = subset_radius_y;

		if (self_adaptive)
		{
			//update the instance according to the subset dimension of current POI
			ICGN2D1_::update(icgn, poi->subset_radius.x, poi->subset_radius.y);
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
		icgn->ref_subset->center = (Point2D)*poi;
		icgn->ref_subset->fill(ref_img);
		float ref_mean_norm = icgn->ref_subset->zeroMeanNorm();

		//build the Hessian matrix
		icgn->hessian.setZero();
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

				icgn->sd_img[r][c][0] = ref_gradient_x;
				icgn->sd_img[r][c][1] = ref_gradient_x * x_local;
				icgn->sd_img[r][c][2] = ref_gradient_x * y_local;
				icgn->sd_img[r][c][3] = ref_gradient_y;
				icgn->sd_img[r][c][4] = ref_gradient_y * x_local;
				icgn->sd_img[r][c][5] = ref_gradient_y * y_local;

				for (int i = 0; i < 6; i++)
				{
					for (int j = 0; j < i + 1; j++)
					{
						icgn->hessian(i, j) += (icgn->sd_img[r][c][i] * icgn->sd_img[r][c][j]);
						icgn->hessian(j, i) = icgn->hessian(i, j);
					}
				}
			}
		}

		//calculate the inversed Hessian matrix
		icgn->inv_hessian = icgn->hessian.inverse();

		//set target subset
		icgn->tar_subset->center = (Point2D)*poi;

		//get initial guess
		Deformation2D1 p_initial(poi->deformation.u, poi->deformation.ux, poi->deformation.uy, poi->deformation.v, poi->deformation.vx, poi->deformation.vy);

		//IC-GN iteration
		int iteration_counter = 0; //initialize iteration counter
		Deformation2D1 p_current, p_increment;
		p_current.setDeformation(p_initial);
		float dp_norm_max, znssd;
		Point2D local_coor, warped_coor, global_coor;

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
					global_coor = icgn->tar_subset->center + warped_coor;
					icgn->tar_subset->eg_mat(r, c) = tar_interp->compute(global_coor);
				}
			}

			float tar_mean_norm = icgn->tar_subset->zeroMeanNorm();

			//calculate error image
			icgn->error_img = icgn->tar_subset->eg_mat * (ref_mean_norm / tar_mean_norm) - (icgn->ref_subset->eg_mat);

			//calculate ZNSSD
			znssd = icgn->error_img.squaredNorm() / (ref_mean_norm * ref_mean_norm);

			//calculate numerator
			float numerator[6] = { 0.f };
			for (int r = 0; r < subset_height; r++)
			{
				for (int c = 0; c < subset_width; c++)
				{
					for (int i = 0; i < 6; i++)
					{
						numerator[i] += (icgn->sd_img[r][c][i] * icgn->error_img(r, c));
					}
				}
			}

			//calculate dp
			float dp[6] = { 0.f };
			for (int i = 0; i < 6; i++)
			{
				for (int j = 0; j < 6; j++)
				{
					dp[i] += (icgn->inv_hessian(i, j) * numerator[j]);
				}
			}
			p_increment.setDeformation(dp);

			//update warp
			p_current.warp_matrix = p_current.warp_matrix * p_increment.warp_matrix.inverse();

			//update p
			p_current.setDeformation();

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

	void ICGN2D1::compute(std::vector<POI2D>& poi_queue)
	{
		auto queue_length = poi_queue.size();
#pragma omp parallel for
		for (int i = 0; i < queue_length; i++)
		{
			compute(&poi_queue[i]);
		}
	}

	void ICGN2D1::compute(POI2D* poi, Point2D& center_offset)
	{
		//set instance w.r.t. thread id 
		std::unique_ptr<ICGN2D1_>& icgn = getInstance(omp_get_thread_num());

		int subset_rx = subset_radius_x;
		int subset_ry = subset_radius_y;

		if (self_adaptive)
		{
			//update the instance according to the subset dimension of current POI
			ICGN2D1_::update(icgn, poi->subset_radius.x, poi->subset_radius.y);
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
		icgn->ref_subset->center = (Point2D)*poi;
		icgn->ref_subset->fill(ref_img);
		float ref_mean_norm = icgn->ref_subset->zeroMeanNorm();

		//build the Hessian matrix
		icgn->hessian.setZero();
		for (int r = 0; r < subset_height; r++)
		{
			for (int c = 0; c < subset_width; c++)
			{
				int x_local_int = c - subset_rx;
				int y_local_int = r - subset_ry;
				int x_global = (int)poi->x + x_local_int;
				int y_global = (int)poi->y + y_local_int;
				float ref_gradient_x = ref_gradient->gradient_x(y_global, x_global);
				float ref_gradient_y = ref_gradient->gradient_y(y_global, x_global);

				float x_local = x_local_int - center_offset.x;
				float y_local = y_local_int - center_offset.y;

				icgn->sd_img[r][c][0] = ref_gradient_x;
				icgn->sd_img[r][c][1] = ref_gradient_x * x_local;
				icgn->sd_img[r][c][2] = ref_gradient_x * y_local;
				icgn->sd_img[r][c][3] = ref_gradient_y;
				icgn->sd_img[r][c][4] = ref_gradient_y * x_local;
				icgn->sd_img[r][c][5] = ref_gradient_y * y_local;

				for (int i = 0; i < 6; i++)
				{
					for (int j = 0; j < i + 1; j++)
					{
						icgn->hessian(i, j) += (icgn->sd_img[r][c][i] * icgn->sd_img[r][c][j]);
						icgn->hessian(j, i) = icgn->hessian(i, j);
					}
				}
			}
		}

		//calculate the inversed Hessian matrix
		icgn->inv_hessian = icgn->hessian.inverse();

		//set target subset
		icgn->tar_subset->center.x = poi->x + center_offset.x;
		icgn->tar_subset->center.y = poi->y + center_offset.y;

		//get initial guess
		Deformation2D1 p_initial(poi->deformation.u, poi->deformation.ux, poi->deformation.uy, poi->deformation.v, poi->deformation.vx, poi->deformation.vy);

		//IC-GN iteration
		int iteration_counter = 0; //initialize iteration counter
		Deformation2D1 p_current, p_increment;
		p_current.setDeformation(p_initial);
		float dp_norm_max, znssd;
		Point2D local_coor, warped_coor, global_coor;
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
					local_coor.x = (x_local - center_offset.x);
					local_coor.y = (y_local - center_offset.y);
					warped_coor = p_current.warp(local_coor);
					global_coor = icgn->tar_subset->center + warped_coor;
					icgn->tar_subset->eg_mat(r, c) = tar_interp->compute(global_coor);
				}
			}

			float tar_mean_norm = icgn->tar_subset->zeroMeanNorm();

			//calculate error image
			icgn->error_img = icgn->tar_subset->eg_mat * (ref_mean_norm / tar_mean_norm) - (icgn->ref_subset->eg_mat);

			//calculate ZNSSD
			znssd = icgn->error_img.squaredNorm() / (ref_mean_norm * ref_mean_norm);

			//calculate numerator
			float numerator[6] = { 0.f };
			for (int r = 0; r < subset_height; r++)
			{
				for (int c = 0; c < subset_width; c++)
				{
					for (int i = 0; i < 6; i++)
					{
						numerator[i] += (icgn->sd_img[r][c][i] * icgn->error_img(r, c));
					}
				}
			}

			//calculate dp
			float dp[6] = { 0.f };
			for (int i = 0; i < 6; i++)
			{
				for (int j = 0; j < 6; j++)
				{
					dp[i] += (icgn->inv_hessian(i, j) * numerator[j]);
				}
			}
			p_increment.setDeformation(dp);

			//update warp
			p_current.warp_matrix = p_current.warp_matrix * p_increment.warp_matrix.inverse();

			//update p
			p_current.setDeformation();

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

	void ICGN2D1::compute(std::vector<POI2D>& poi_queue, std::vector<Point2D>& center_offset_queue)
	{
		auto queue_length = poi_queue.size();
#pragma omp parallel for
		for (int i = 0; i < queue_length; i++)
		{
			compute(&poi_queue[i], center_offset_queue[i]);
		}
	}




	std::unique_ptr<ICGN2D2_> ICGN2D2_::allocate(int subset_radius_x, int subset_radius_y)
	{
		int subset_width = 2 * subset_radius_x + 1;
		int subset_height = 2 * subset_radius_y + 1;
		Point2D subset_center(0, 0);

		std::unique_ptr<ICGN2D2_> ICGN_instance = std::make_unique<ICGN2D2_>();
		ICGN_instance->ref_subset = std::make_unique<Subset2D>(subset_center, subset_radius_x, subset_radius_y);
		ICGN_instance->tar_subset = std::make_unique<Subset2D>(subset_center, subset_radius_x, subset_radius_y);
		ICGN_instance->error_img = Eigen::MatrixXf::Zero(subset_height, subset_width);
		ICGN_instance->sd_img = new3D(subset_height, subset_width, 12);

		return ICGN_instance;
	}

	void ICGN2D2_::release(std::unique_ptr<ICGN2D2_>& instance)
	{
		if (instance->sd_img != nullptr)
		{
			delete3D(instance->sd_img);
		}

		instance->ref_subset.reset();
		instance->tar_subset.reset();
	}

	void ICGN2D2_::update(std::unique_ptr<ICGN2D2_>& instance, int subset_radius_x, int subset_radius_y)
	{
		release(instance);

		int subset_width = 2 * subset_radius_x + 1;
		int subset_height = 2 * subset_radius_y + 1;
		Point2D subset_center(0, 0);

		instance->ref_subset = std::make_unique<Subset2D>(subset_center, subset_radius_x, subset_radius_y);
		instance->tar_subset = std::make_unique<Subset2D>(subset_center, subset_radius_x, subset_radius_y);
		instance->error_img.resize(subset_height, subset_width);
		instance->sd_img = new3D(subset_height, subset_width, 12);
	}

	std::unique_ptr<ICGN2D2_>& ICGN2D2::getInstance(int tid)
	{
		if (tid >= (int)instance_pool.size())
		{
			throw std::string("CPU thread ID over limit");
		}

		return instance_pool[tid];
	}

	ICGN2D2::ICGN2D2(int subset_radius_x, int subset_radius_y, float conv_criterion, float stop_condition, int thread_number)
		: ref_gradient(nullptr), tar_interp(nullptr)
	{
		this->subset_radius_x = subset_radius_x;
		this->subset_radius_y = subset_radius_y;
		this->conv_criterion = conv_criterion;
		this->stop_condition = stop_condition;
		this->thread_number = thread_number;

		self_adaptive = false;

		instance_pool.resize(thread_number);
#pragma omp parallel for
		for (int i = 0; i < thread_number; i++)
		{
			instance_pool[i] = ICGN2D2_::allocate(subset_radius_x, subset_radius_y);
		}
	}

	ICGN2D2::~ICGN2D2()
	{
		ref_gradient.reset();
		tar_interp.reset();

		for (auto& instance : instance_pool)
		{
			ICGN2D2_::release(instance);
			instance.reset();
		}
		std::vector<std::unique_ptr<ICGN2D2_>>().swap(instance_pool);
	}

	void ICGN2D2::setIteration(float conv_criterion, float stop_condition)
	{
		this->conv_criterion = conv_criterion;
		this->stop_condition = stop_condition;
	}

	void ICGN2D2::setIteration(POI2D* poi)
	{
		conv_criterion = poi->result.convergence;
		stop_condition = poi->result.iteration;
	}

	void ICGN2D2::prepareRef()
	{
		if (ref_gradient != nullptr)
		{
			ref_gradient.reset();
		}

		ref_gradient = std::make_unique<Gradient2D4>(*ref_img);
		ref_gradient->getGradientX();
		ref_gradient->getGradientY();
	}

	void ICGN2D2::prepareTar()
	{
		if (tar_interp != nullptr)
		{
			tar_interp.reset();
		}

		tar_interp = std::make_unique<BicubicBspline>(*tar_img);
		tar_interp->prepare();
	}

	void ICGN2D2::prepare()
	{
		prepareRef();
		prepareTar();
	}

	void ICGN2D2::compute(POI2D* poi)
	{
		//set instance w.r.t. thread id 
		std::unique_ptr<ICGN2D2_>& icgn = getInstance(omp_get_thread_num());

		int subset_rx = subset_radius_x;
		int subset_ry = subset_radius_y;

		if (self_adaptive)
		{
			//update the instance according to the subset dimension of current POI
			ICGN2D2_::update(icgn, poi->subset_radius.x, poi->subset_radius.y);
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
		icgn->ref_subset->center = (Point2D)*poi;
		icgn->ref_subset->fill(ref_img);
		float ref_mean_norm = icgn->ref_subset->zeroMeanNorm();

		//build the Hessian matrix
		icgn->hessian.setZero();
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

				icgn->sd_img[r][c][0] = ref_gradient_x;
				icgn->sd_img[r][c][1] = ref_gradient_x * x_local;
				icgn->sd_img[r][c][2] = ref_gradient_x * y_local;
				icgn->sd_img[r][c][3] = ref_gradient_x * xx_local;
				icgn->sd_img[r][c][4] = ref_gradient_x * xy_local;
				icgn->sd_img[r][c][5] = ref_gradient_x * yy_local;

				icgn->sd_img[r][c][6] = ref_gradient_y;
				icgn->sd_img[r][c][7] = ref_gradient_y * x_local;
				icgn->sd_img[r][c][8] = ref_gradient_y * y_local;
				icgn->sd_img[r][c][9] = ref_gradient_y * xx_local;
				icgn->sd_img[r][c][10] = ref_gradient_y * xy_local;
				icgn->sd_img[r][c][11] = ref_gradient_y * yy_local;

				for (int i = 0; i < 12; i++)
				{
					for (int j = 0; j < i + 1; j++)
					{
						icgn->hessian(i, j) += (icgn->sd_img[r][c][i] * icgn->sd_img[r][c][j]);
						icgn->hessian(j, i) = icgn->hessian(i, j);
					}
				}
			}
		}

		//calculate the inversed Hessian matrix
		icgn->inv_hessian = icgn->hessian.inverse();

		//set target subset
		icgn->tar_subset->center = (Point2D)*poi;

		//get initial guess
		Deformation2D1 p_initial(poi->deformation.u, poi->deformation.ux, poi->deformation.uy, poi->deformation.v, poi->deformation.vx, poi->deformation.vy);

		//IC-GN iteration
		int iteration_counter = 0; //initialize iteration counter
		Deformation2D2 p_current, p_increment;
		p_current.setDeformation(p_initial);
		float dp_norm_max, znssd;
		Point2D local_coor, warped_coor, global_coor;
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
					global_coor = icgn->tar_subset->center + warped_coor;
					icgn->tar_subset->eg_mat(r, c) = tar_interp->compute(global_coor);
				}
			}
			float tar_mean_norm = icgn->tar_subset->zeroMeanNorm();

			//calculate error image
			icgn->error_img = icgn->tar_subset->eg_mat * (ref_mean_norm / tar_mean_norm) - (icgn->ref_subset->eg_mat);

			//calculate ZNSSD
			znssd = icgn->error_img.squaredNorm() / (ref_mean_norm * ref_mean_norm);

			//calculate numerator
			float numerator[12] = { 0.f };
			for (int r = 0; r < subset_height; r++)
			{
				for (int c = 0; c < subset_width; c++)
				{
					for (int i = 0; i < 12; i++)
					{
						numerator[i] += (icgn->sd_img[r][c][i] * icgn->error_img(r, c));
					}
				}
			}

			//calculate dp
			float dp[12] = { 0.f };
			for (int i = 0; i < 12; i++)
			{
				for (int j = 0; j < 12; j++)
				{
					dp[i] += (icgn->inv_hessian(i, j) * numerator[j]);
				}
			}
			p_increment.setDeformation(dp);

			//update warp
			p_current.warp_matrix = p_current.warp_matrix * p_increment.warp_matrix.inverse();

			//update p
			p_current.setDeformation();

			//check convergence
			int subset_rx2 = subset_rx * subset_rx;
			int subset_ry2 = subset_ry * subset_ry;
			int subset_radius_xy2 = subset_rx2 * subset_ry2;
			int subset_rx4 = subset_rx2 * subset_rx2 * 0.25f;
			int subset_ry4 = subset_ry2 * subset_ry2 * 0.25f;

			dp_norm_max = p_increment.u * p_increment.u
				+ p_increment.ux * p_increment.ux * subset_rx2
				+ p_increment.uy * p_increment.uy * subset_ry2
				+ p_increment.uxx * p_increment.uxx * subset_rx4
				+ p_increment.uyy * p_increment.uyy * subset_ry4
				+ p_increment.uxy * p_increment.uxy * subset_radius_xy2
				+ p_increment.v * p_increment.v
				+ p_increment.vx * p_increment.vx * subset_rx2
				+ p_increment.vy * p_increment.vy * subset_ry2
				+ p_increment.vxx * p_increment.vxx * subset_rx4
				+ p_increment.vyy * p_increment.vyy * subset_ry4
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

	void ICGN2D2::compute(std::vector<POI2D>& poi_queue)
	{
		auto queue_length = poi_queue.size();
#pragma omp parallel for
		for (int i = 0; i < queue_length; i++)
		{
			compute(&poi_queue[i]);
		}
	}

	void ICGN2D2::compute(POI2D* poi, Point2D& center_offset)
	{
		//set instance w.r.t. thread id 
		std::unique_ptr<ICGN2D2_>& icgn = getInstance(omp_get_thread_num());

		int subset_rx = subset_radius_x;
		int subset_ry = subset_radius_y;

		if (self_adaptive)
		{
			//update the instance according to the subset dimension of current POI
			ICGN2D2_::update(icgn, poi->subset_radius.x, poi->subset_radius.y);
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
		icgn->ref_subset->center = (Point2D)*poi;
		icgn->ref_subset->fill(ref_img);
		float ref_mean_norm = icgn->ref_subset->zeroMeanNorm();

		//build the Hessian matrix
		icgn->hessian.setZero();
		for (int r = 0; r < subset_height; r++)
		{
			for (int c = 0; c < subset_width; c++)
			{
				int x_local_int = c - subset_rx;
				int y_local_int = r - subset_ry;
				int x_global = (int)poi->x + x_local_int;
				int y_global = (int)poi->y + y_local_int;
				float ref_gradient_x = ref_gradient->gradient_x(y_global, x_global);
				float ref_gradient_y = ref_gradient->gradient_y(y_global, x_global);

				float x_local = x_local_int - center_offset.x;
				float y_local = y_local_int - center_offset.y;
				float xx_local = (x_local * x_local) * 0.5f;
				float xy_local = x_local * y_local;
				float yy_local = (y_local * y_local) * 0.5f;

				icgn->sd_img[r][c][0] = ref_gradient_x;
				icgn->sd_img[r][c][1] = ref_gradient_x * x_local;
				icgn->sd_img[r][c][2] = ref_gradient_x * y_local;
				icgn->sd_img[r][c][3] = ref_gradient_x * xx_local;
				icgn->sd_img[r][c][4] = ref_gradient_x * xy_local;
				icgn->sd_img[r][c][5] = ref_gradient_x * yy_local;

				icgn->sd_img[r][c][6] = ref_gradient_y;
				icgn->sd_img[r][c][7] = ref_gradient_y * x_local;
				icgn->sd_img[r][c][8] = ref_gradient_y * y_local;
				icgn->sd_img[r][c][9] = ref_gradient_y * xx_local;
				icgn->sd_img[r][c][10] = ref_gradient_y * xy_local;
				icgn->sd_img[r][c][11] = ref_gradient_y * yy_local;

				for (int i = 0; i < 12; i++)
				{
					for (int j = 0; j < i + 1; j++)
					{
						icgn->hessian(i, j) += (icgn->sd_img[r][c][i] * icgn->sd_img[r][c][j]);
						icgn->hessian(j, i) = icgn->hessian(i, j);
					}
				}
			}
		}

		//calculate the inversed Hessian matrix
		icgn->inv_hessian = icgn->hessian.inverse();

		//set target subset
		icgn->tar_subset->center.x = poi->x + center_offset.x;
		icgn->tar_subset->center.y = poi->y + center_offset.y;

		//get initial guess
		Deformation2D1 p_initial(poi->deformation.u, poi->deformation.ux, poi->deformation.uy, poi->deformation.v, poi->deformation.vx, poi->deformation.vy);

		//IC-GN iteration
		int iteration_counter = 0; //initialize iteration counter
		Deformation2D2 p_current, p_increment;
		p_current.setDeformation(p_initial);
		float dp_norm_max, znssd;
		Point2D local_coor, warped_coor, global_coor;
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
					local_coor.x = x_local - center_offset.x;
					local_coor.y = y_local - center_offset.y;
					warped_coor = p_current.warp(local_coor);
					global_coor = icgn->tar_subset->center + warped_coor;
					icgn->tar_subset->eg_mat(r, c) = tar_interp->compute(global_coor);
				}
			}
			float tar_mean_norm = icgn->tar_subset->zeroMeanNorm();

			//calculate error image
			icgn->error_img = icgn->tar_subset->eg_mat * (ref_mean_norm / tar_mean_norm) - (icgn->ref_subset->eg_mat);

			//calculate ZNSSD
			znssd = icgn->error_img.squaredNorm() / (ref_mean_norm * ref_mean_norm);

			//calculate numerator
			float numerator[12] = { 0.f };
			for (int r = 0; r < subset_height; r++)
			{
				for (int c = 0; c < subset_width; c++)
				{
					for (int i = 0; i < 12; i++)
					{
						numerator[i] += (icgn->sd_img[r][c][i] * icgn->error_img(r, c));
					}
				}
			}

			//calculate dp
			float dp[12] = { 0.f };
			for (int i = 0; i < 12; i++)
			{
				for (int j = 0; j < 12; j++)
				{
					dp[i] += (icgn->inv_hessian(i, j) * numerator[j]);
				}
			}
			p_increment.setDeformation(dp);

			//update warp
			p_current.warp_matrix = p_current.warp_matrix * p_increment.warp_matrix.inverse();

			//update p
			p_current.setDeformation();

			//check convergence
			int subset_rx2 = subset_rx * subset_rx;
			int subset_ry2 = subset_ry * subset_ry;
			int subset_radius_xy2 = subset_rx2 * subset_ry2;
			int subset_rx4 = subset_rx2 * subset_rx2 * 0.25f;
			int subset_ry4 = subset_ry2 * subset_ry2 * 0.25f;

			dp_norm_max = p_increment.u * p_increment.u
				+ p_increment.ux * p_increment.ux * subset_rx2
				+ p_increment.uy * p_increment.uy * subset_ry2
				+ p_increment.uxx * p_increment.uxx * subset_rx4
				+ p_increment.uyy * p_increment.uyy * subset_ry4
				+ p_increment.uxy * p_increment.uxy * subset_radius_xy2
				+ p_increment.v * p_increment.v
				+ p_increment.vx * p_increment.vx * subset_rx2
				+ p_increment.vy * p_increment.vy * subset_ry2
				+ p_increment.vxx * p_increment.vxx * subset_rx4
				+ p_increment.vyy * p_increment.vyy * subset_ry4
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

	void ICGN2D2::compute(std::vector<POI2D>& poi_queue, std::vector<Point2D>& center_offset_queue)
	{
		auto queue_length = poi_queue.size();
#pragma omp parallel for
		for (int i = 0; i < queue_length; i++)
		{
			compute(&poi_queue[i], center_offset_queue[i]);
		}
	}



	//DVC
	std::unique_ptr<ICGN3D1_> ICGN3D1_::allocate(int subset_radius_x, int subset_radius_y, int subset_radius_z)
	{
		int dim_x = 2 * subset_radius_x + 1;
		int dim_y = 2 * subset_radius_y + 1;
		int dim_z = 2 * subset_radius_z + 1;
		Point3D subset_center(0, 0, 0);

		std::unique_ptr<ICGN3D1_> ICGN_instance = std::make_unique<ICGN3D1_>();
		ICGN_instance->ref_subset = std::make_unique<Subset3D>(subset_center, subset_radius_x, subset_radius_y, subset_radius_z);
		ICGN_instance->tar_subset = std::make_unique<Subset3D>(subset_center, subset_radius_x, subset_radius_y, subset_radius_z);
		ICGN_instance->error_img = new3D(dim_z, dim_y, dim_x);
		ICGN_instance->sd_img = new4D(dim_z, dim_y, dim_x, 12);

		return ICGN_instance;
	}

	void ICGN3D1_::release(std::unique_ptr<ICGN3D1_>& instance)
	{
		if (instance->error_img != nullptr)
		{
			delete3D(instance->error_img);
		}

		if (instance->sd_img != nullptr)
		{
			delete4D(instance->sd_img);
		}

		instance->ref_subset.reset();
		instance->tar_subset.reset();
	}

	void ICGN3D1_::update(std::unique_ptr<ICGN3D1_>& instance, int subset_radius_x, int subset_radius_y, int subset_radius_z)
	{
		release(instance);

		int dim_x = 2 * subset_radius_x + 1;
		int dim_y = 2 * subset_radius_y + 1;
		int dim_z = 2 * subset_radius_z + 1;
		Point3D subset_center(0, 0, 0);

		instance->ref_subset = std::make_unique<Subset3D>(subset_center, subset_radius_x, subset_radius_y, subset_radius_z);
		instance->tar_subset = std::make_unique<Subset3D>(subset_center, subset_radius_x, subset_radius_y, subset_radius_z);
		instance->error_img = new3D(dim_z, dim_y, dim_x);
		instance->sd_img = new4D(dim_z, dim_y, dim_x, 12);
	}

	std::unique_ptr<ICGN3D1_>& ICGN3D1::getInstance(int tid)
	{
		if (tid >= (int)instance_pool.size())
		{
			throw std::string("CPU thread ID over limit");
		}
		return instance_pool[tid];
	}

	ICGN3D1::ICGN3D1(int subset_radius_x, int subset_radius_y, int subset_radius_z, float conv_criterion, float stop_condition, int thread_number)
		: ref_gradient(nullptr), tar_interp(nullptr)
	{
		this->subset_radius_x = subset_radius_x;
		this->subset_radius_y = subset_radius_y;
		this->subset_radius_z = subset_radius_z;
		this->conv_criterion = conv_criterion;
		this->stop_condition = stop_condition;
		this->thread_number = thread_number;

		instance_pool.resize(thread_number);
#pragma omp parallel for
		for (int i = 0; i < thread_number; i++)
		{
			instance_pool[i] = ICGN3D1_::allocate(subset_radius_x, subset_radius_y, subset_radius_z);
		}
	}

	ICGN3D1::~ICGN3D1()
	{
		ref_gradient.reset();
		tar_interp.reset();

		for (auto& instance : instance_pool)
		{
			ICGN3D1_::release(instance);
			instance.reset();
		}
		std::vector<std::unique_ptr<ICGN3D1_>>().swap(instance_pool);
	}

	void ICGN3D1::setIteration(float conv_criterion, float stop_condition)
	{
		this->conv_criterion = conv_criterion;
		this->stop_condition = stop_condition;
	}

	void ICGN3D1::setIteration(POI3D* poi)
	{
		conv_criterion = poi->result.convergence;
		stop_condition = (int)poi->result.iteration;
	}

	void ICGN3D1::prepareRef()
	{
		if (ref_gradient != nullptr)
		{
			ref_gradient.reset();
		}

		ref_gradient = std::make_unique<Gradient3D4>(*ref_img);
		ref_gradient->getGradientX();
		ref_gradient->getGradientY();
		ref_gradient->getGradientZ();
	}

	void ICGN3D1::prepareTar()
	{
		if (tar_interp != nullptr)
		{
			tar_interp.reset();
		}

		tar_interp = std::make_unique<TricubicBspline>(*tar_img);
		tar_interp->prepare();
	}

	void ICGN3D1::prepare()
	{
		prepareRef();
		prepareTar();
	}

	void ICGN3D1::compute(POI3D* poi)
	{
		//set instance w.r.t. thread id 
		std::unique_ptr<ICGN3D1_>& icgn = getInstance(omp_get_thread_num());

		int subset_rx = subset_radius_x;
		int subset_ry = subset_radius_y;
		int subset_rz = subset_radius_z;

		if ((poi->x - subset_rx) < 0 || (poi->y - subset_ry) < 0 || (poi->z - subset_rz) < 0
			|| (poi->x + subset_rx) > (ref_img->dim_x - 1) || (poi->y + subset_ry) > (ref_img->dim_y - 1) || (poi->z + subset_rz) > (ref_img->dim_z - 1)
			|| fabs(poi->deformation.u) >= ref_img->dim_x || fabs(poi->deformation.v) >= ref_img->dim_y || fabs(poi->deformation.w) >= ref_img->dim_z
			|| poi->result.zncc < 0 || std::isnan(poi->deformation.u) || std::isnan(poi->deformation.v) || std::isnan(poi->deformation.w))
		{
			poi->result.zncc = poi->result.zncc >= 0 ? -3.f : poi->result.zncc;
			return;
		}
		int subset_dim_x = 2 * subset_rx + 1;
		int subset_dim_y = 2 * subset_ry + 1;
		int subset_dim_z = 2 * subset_rz + 1;

		//set reference subset
		icgn->ref_subset->center = (Point3D)*poi;
		icgn->ref_subset->fill(ref_img);
		float ref_mean_norm = icgn->ref_subset->zeroMeanNorm();

		//build the hessian matrix
		icgn->hessian.setZero();
		for (int i = 0; i < subset_dim_z; i++)
		{
			for (int j = 0; j < subset_dim_y; j++)
			{
				for (int k = 0; k < subset_dim_x; k++)
				{
					int x_local = k - subset_rx;
					int y_local = j - subset_ry;
					int z_local = i - subset_rz;
					int x_global = (int)poi->x + x_local;
					int y_global = (int)poi->y + y_local;
					int z_global = (int)poi->z + z_local;
					float ref_gradient_x = ref_gradient->gradient_x[z_global][y_global][x_global];
					float ref_gradient_y = ref_gradient->gradient_y[z_global][y_global][x_global];
					float ref_gradient_z = ref_gradient->gradient_z[z_global][y_global][x_global];

					icgn->sd_img[i][j][k][0] = ref_gradient_x;
					icgn->sd_img[i][j][k][1] = ref_gradient_x * x_local;
					icgn->sd_img[i][j][k][2] = ref_gradient_x * y_local;
					icgn->sd_img[i][j][k][3] = ref_gradient_x * z_local;
					icgn->sd_img[i][j][k][4] = ref_gradient_y;
					icgn->sd_img[i][j][k][5] = ref_gradient_y * x_local;
					icgn->sd_img[i][j][k][6] = ref_gradient_y * y_local;
					icgn->sd_img[i][j][k][7] = ref_gradient_y * z_local;
					icgn->sd_img[i][j][k][8] = ref_gradient_z;
					icgn->sd_img[i][j][k][9] = ref_gradient_z * x_local;
					icgn->sd_img[i][j][k][10] = ref_gradient_z * y_local;
					icgn->sd_img[i][j][k][11] = ref_gradient_z * z_local;

					for (int r = 0; r < 12; r++)
					{
						for (int c = 0; c < r + 1; c++)
						{
							icgn->hessian(r, c) += (icgn->sd_img[i][j][k][r] * icgn->sd_img[i][j][k][c]);
							icgn->hessian(c, r) = icgn->hessian(r, c);
						}
					}
				}
			}
		}
		//calculate the inversed Hessian matrix
		icgn->inv_hessian = icgn->hessian.inverse();

		//set target subset
		icgn->tar_subset->center = (Point3D)*poi;

		//get initial guess
		Deformation3D1 p_initial(poi->deformation.u, poi->deformation.ux, poi->deformation.uy, poi->deformation.uz,
			poi->deformation.v, poi->deformation.vx, poi->deformation.vy, poi->deformation.vz,
			poi->deformation.w, poi->deformation.wx, poi->deformation.wy, poi->deformation.wz);

		//IC-GN iteration
		int iteration_counter = 0; //initialize iteration counter
		Deformation3D1 p_current, p_increment;
		p_current.setDeformation(p_initial);
		float dp_norm_max, znssd;
		Point3D local_coor, warped_coor, global_coor;
		do
		{
			iteration_counter++;
			//reconstruct target subset
			for (int i = 0; i < subset_dim_z; i++)
			{
				for (int j = 0; j < subset_dim_y; j++)
				{
					for (int k = 0; k < subset_dim_x; k++)
					{
						int x_local = k - subset_rx;
						int y_local = j - subset_ry;
						int z_local = i - subset_rz;
						local_coor.x = x_local;
						local_coor.y = y_local;
						local_coor.z = z_local;
						warped_coor = p_current.warp(local_coor);
						global_coor = icgn->tar_subset->center + warped_coor;
						icgn->tar_subset->vol_mat[i][j][k] = tar_interp->compute(global_coor);
					}
				}
			}
			float tar_mean_norm = icgn->tar_subset->zeroMeanNorm();

			//calculate error image
			float error_factor = ref_mean_norm / tar_mean_norm;
			float squared_sum = 0;
			for (int i = 0; i < subset_dim_z; i++)
			{
				for (int j = 0; j < subset_dim_y; j++)
				{
					for (int k = 0; k < subset_dim_x; k++)
					{
						icgn->error_img[i][j][k] = error_factor * icgn->tar_subset->vol_mat[i][j][k] - icgn->ref_subset->vol_mat[i][j][k];
						squared_sum += (icgn->error_img[i][j][k] * icgn->error_img[i][j][k]);
					}
				}
			}

			//calculate ZNSSD
			znssd = squared_sum / (ref_mean_norm * ref_mean_norm);

			//calculate numerator
			float numerator[12] = { 0.f };
			for (int i = 0; i < subset_dim_z; i++)
			{
				for (int j = 0; j < subset_dim_y; j++)
				{
					for (int k = 0; k < subset_dim_x; k++)
					{
						for (int l = 0; l < 12; l++)
						{
							numerator[l] += (icgn->sd_img[i][j][k][l] * icgn->error_img[i][j][k]);
						}
					}
				}
			}

			//calculate dp
			float dp[12] = { 0.f };
			for (int i = 0; i < 12; i++)
			{
				for (int j = 0; j < 12; j++)
				{
					dp[i] += (icgn->inv_hessian(i, j) * numerator[j]);
				}
			}
			p_increment.setDeformation(dp);

			//update warp
			p_current.warp_matrix = p_current.warp_matrix * p_increment.warp_matrix.inverse();

			//update p
			p_current.setDeformation();

			//check convergence
			dp_norm_max = sqrt(p_increment.u * p_increment.u + p_increment.v * p_increment.v + p_increment.w * p_increment.w);

		} while (iteration_counter < stop_condition && dp_norm_max >= conv_criterion);

		//store the final results
		poi->deformation.u = p_current.u;
		poi->deformation.ux = p_current.ux;
		poi->deformation.uy = p_current.uy;
		poi->deformation.uz = p_current.uz;
		poi->deformation.v = p_current.v;
		poi->deformation.vx = p_current.vx;
		poi->deformation.vy = p_current.vy;
		poi->deformation.vz = p_current.vz;
		poi->deformation.w = p_current.w;
		poi->deformation.wx = p_current.wx;
		poi->deformation.wy = p_current.wy;
		poi->deformation.wz = p_current.wz;

		//save the parameters for output
		poi->result.u0 = p_initial.u;
		poi->result.v0 = p_initial.v;
		poi->result.w0 = p_initial.w;
		poi->result.zncc = 0.5f * (2 - znssd);
		poi->result.iteration = (float)iteration_counter;
		poi->result.convergence = dp_norm_max;

		//store the subset size
		poi->subset_radius.x = subset_rx;
		poi->subset_radius.y = subset_ry;
		poi->subset_radius.z = subset_rz;

		//check if the iteration converge at the desired target
		if (poi->result.convergence >= conv_criterion && poi->result.iteration >= stop_condition)
		{
			poi->result.zncc = -4.f;
		}

		//check if the case of NaN occurs for ZNCC or displacments
		if (std::isnan(poi->result.zncc) || std::isnan(poi->deformation.u) || std::isnan(poi->deformation.v) || std::isnan(poi->deformation.w))
		{
			poi->deformation.u = poi->result.u0;
			poi->deformation.v = poi->result.v0;
			poi->deformation.w = poi->result.w0;
			poi->result.zncc = -5.f;
		}
	}

	void ICGN3D1::compute(std::vector<POI3D>& poi_queue)
	{
		auto queue_length = poi_queue.size();
#pragma omp parallel for
		for (int i = 0; i < queue_length; i++)
		{
			compute(&poi_queue[i]);
		}
	}

}//namespace opencorr