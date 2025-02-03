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

#include "oc_nr.h"

namespace opencorr
{
	std::unique_ptr<NR2D1_> NR2D1_::allocate(int subset_radius_x, int subset_radius_y)
	{
		int subset_width = 2 * subset_radius_x + 1;
		int subset_height = 2 * subset_radius_y + 1;
		Point2D subset_center(0, 0);

		std::unique_ptr<NR2D1_> NR_instance = std::make_unique<NR2D1_>();
		NR_instance->ref_subset = std::make_unique<Subset2D>(subset_center, subset_radius_x, subset_radius_y);
		NR_instance->tar_subset = std::make_unique<Subset2D>(subset_center, subset_radius_x, subset_radius_y);
		NR_instance->tar_gradient_x = Eigen::MatrixXf::Zero(subset_height, subset_width);
		NR_instance->tar_gradient_y = Eigen::MatrixXf::Zero(subset_height, subset_width);
		NR_instance->error_img = Eigen::MatrixXf::Zero(subset_height, subset_width);
		NR_instance->sd_img = new3D(subset_height, subset_width, 6);

		return NR_instance;
	}

	void NR2D1_::release(std::unique_ptr<NR2D1_>& instance)
	{
		if (instance->sd_img != nullptr)
		{
			delete3D(instance->sd_img);
		}

		instance->ref_subset.reset();
		instance->tar_subset.reset();
	}

	void NR2D1_::update(std::unique_ptr<NR2D1_>& instance, int subset_radius_x, int subset_radius_y)
	{
		release(instance);

		int subset_width = 2 * subset_radius_x + 1;
		int subset_height = 2 * subset_radius_y + 1;
		Point2D subset_center(0, 0);

		instance->ref_subset = std::make_unique<Subset2D>(subset_center, subset_radius_x, subset_radius_y);
		instance->tar_subset = std::make_unique<Subset2D>(subset_center, subset_radius_x, subset_radius_y);
		instance->tar_gradient_x.resize(subset_height, subset_width);
		instance->tar_gradient_y.resize(subset_height, subset_width);
		instance->error_img.resize(subset_height, subset_width);
		instance->sd_img = new3D(subset_height, subset_width, 6);
	}

	std::unique_ptr<NR2D1_>& NR2D1::getInstance(int tid)
	{
		if (tid >= (int)instance_pool.size())
		{
			throw std::string("CPU thread ID over limit");
		}

		return instance_pool[tid];
	}

	NR2D1::NR2D1(int subset_radius_x, int subset_radius_y, float conv_criterion, float stop_condition, int thread_number)
		: tar_gradient(nullptr), tar_interp(nullptr), tar_interp_x(nullptr), tar_interp_y(nullptr)
	{
		this->subset_radius_x = subset_radius_x;
		this->subset_radius_y = subset_radius_y;
		this->conv_criterion = conv_criterion;
		this->stop_condition = stop_condition;
		this->thread_number = thread_number;

		instance_pool.resize(thread_number);
#pragma omp parallel for
		for (int i = 0; i < thread_number; i++)
		{
			instance_pool[i] = NR2D1_::allocate(subset_radius_x, subset_radius_y);
		}
	}

	NR2D1::~NR2D1()
	{
		tar_gradient.reset();
		tar_interp.reset();
		tar_interp_x.reset();
		tar_interp_y.reset();

		for (auto& instance : instance_pool)
		{
			NR2D1_::release(instance);
			instance.reset();
		}
		std::vector<std::unique_ptr<NR2D1_>>().swap(instance_pool);
	}

	void NR2D1::setIteration(float conv_criterion, float stop_condition)
	{
		this->conv_criterion = conv_criterion;
		this->stop_condition = stop_condition;
	}

	void NR2D1::setIteration(POI2D* poi)
	{
		conv_criterion = poi->result.convergence;
		stop_condition = (int)poi->result.iteration;
	}

	void NR2D1::prepare()
	{
		//create gradient maps of tar image
		if (tar_gradient != nullptr)
		{
			tar_gradient.reset();
		}
		tar_gradient = std::make_unique<Gradient2D4>(*tar_img);
		tar_gradient->getGradientX();
		tar_gradient->getGradientY();

		//create interpolation coefficient table of tar image
		if (tar_interp != nullptr)
		{
			tar_interp.reset();
		}
		tar_interp = std::make_unique<BicubicBspline>(*tar_img);
		tar_interp->prepare();

		//create interpolation coefficient table of gradient along x
		Image2D gradient_img(tar_img->width, tar_img->height);
		gradient_img.eg_mat = tar_gradient->gradient_x;

		if (tar_interp_x != nullptr)
		{
			tar_interp_x.reset();
		}
		tar_interp_x = std::make_unique<BicubicBspline>(gradient_img);
		tar_interp_x->prepare();

		//create interpolation coefficient table of gradient along y
		gradient_img.eg_mat = tar_gradient->gradient_y;

		if (tar_interp_y != nullptr)
		{
			tar_interp_y.reset();
		}
		tar_interp_y = std::make_unique<BicubicBspline>(gradient_img);
		tar_interp_y->prepare();
	}

	void NR2D1::compute(POI2D* poi)
	{
		//set instance w.r.t. thread id 
		std::unique_ptr<NR2D1_>& fa_nr = getInstance(omp_get_thread_num());

		if (poi->y - subset_radius_y < 0 || poi->x - subset_radius_x < 0
			|| poi->y + subset_radius_y > ref_img->height - 1 || poi->x + subset_radius_x > ref_img->width - 1
			|| fabs(poi->deformation.u) >= ref_img->width || fabs(poi->deformation.v) >= ref_img->height
			|| poi->result.zncc < 0 || std::isnan(poi->deformation.u) || std::isnan(poi->deformation.v))
		{
			poi->result.zncc = poi->result.zncc < -1 ? poi->result.zncc : -1;
		}
		else
		{
			int subset_width = 2 * subset_radius_x + 1;
			int subset_height = 2 * subset_radius_y + 1;

			//set reference subset
			fa_nr->ref_subset->center = (Point2D)*poi;
			fa_nr->ref_subset->fill(ref_img);
			float ref_mean_norm = fa_nr->ref_subset->zeroMeanNorm();

			//set target subset
			fa_nr->tar_subset->center = (Point2D)*poi;

			//get initial guess
			Deformation2D1 p_initial(poi->deformation.u, poi->deformation.ux, poi->deformation.uy, poi->deformation.v, poi->deformation.vx, poi->deformation.vy);

			//Newton-Raphson iteration
			int iteration_counter = 0; //initialize iteration counter
			Deformation2D1 p_current, p_increment;
			p_current.setDeformation(p_initial);
			float dp_norm_max, znssd;
			do
			{
				iteration_counter++;
				//reconstruct the subsets of warped target as well as the corresponding matrices of its gradients
				for (int r = 0; r < subset_height; r++)
				{
					for (int c = 0; c < subset_width; c++)
					{
						int x_local = c - subset_radius_x;
						int y_local = r - subset_radius_y;
						Point2D local_coor(x_local, y_local);
						Point2D warped_coor = p_current.warp(local_coor);
						Point2D global_coor = fa_nr->tar_subset->center + warped_coor;

						fa_nr->tar_subset->eg_mat(r, c) = tar_interp->compute(global_coor);
						fa_nr->tar_gradient_x(r, c) = tar_interp_x->compute(global_coor);
						fa_nr->tar_gradient_y(r, c) = tar_interp_y->compute(global_coor);
					}
				}
				float tar_mean_norm = fa_nr->tar_subset->zeroMeanNorm();

				//build the Hessian matrix
				fa_nr->hessian.setZero();
				for (int r = 0; r < subset_height; r++)
				{
					for (int c = 0; c < subset_width; c++)
					{
						int x_local = c - subset_radius_x;
						int y_local = r - subset_radius_y;
						float tar_grad_x = fa_nr->tar_gradient_x(r, c);
						float tar_grad_y = fa_nr->tar_gradient_y(r, c);

						fa_nr->sd_img[r][c][0] = tar_grad_x;
						fa_nr->sd_img[r][c][1] = tar_grad_x * x_local;
						fa_nr->sd_img[r][c][2] = tar_grad_x * y_local;
						fa_nr->sd_img[r][c][3] = tar_grad_y;
						fa_nr->sd_img[r][c][4] = tar_grad_y * x_local;
						fa_nr->sd_img[r][c][5] = tar_grad_y * y_local;

						for (int i = 0; i < 6; i++)
						{
							for (int j = 0; j < 6; j++)
							{
								fa_nr->hessian(i, j) += (fa_nr->sd_img[r][c][i] * fa_nr->sd_img[r][c][j]);
							}
						}
					}
				}

				//calculate the inversed Hessian matrix
				fa_nr->inv_hessian = fa_nr->hessian.inverse();

				//calculate error image
				fa_nr->error_img = fa_nr->ref_subset->eg_mat * (tar_mean_norm / ref_mean_norm) - (fa_nr->tar_subset->eg_mat);

				//calculate ZNSSD
				znssd = fa_nr->error_img.squaredNorm() / (tar_mean_norm * tar_mean_norm);

				//calculate numerator
				float numerator[6] = { 0.f };
				for (int r = 0; r < subset_height; r++)
				{
					for (int c = 0; c < subset_width; c++)
					{
						for (int i = 0; i < 6; i++)
						{
							numerator[i] += (fa_nr->sd_img[r][c][i] * fa_nr->error_img(r, c));
						}
					}
				}

				//calculate dp
				float dp[6] = { 0.f };
				for (int i = 0; i < 6; i++)
				{
					for (int j = 0; j < 6; j++)
					{
						dp[i] += (fa_nr->inv_hessian(i, j) * numerator[j]);
					}
				}
				p_increment.setDeformation(dp);

				//update p
				p_current.setDeformation(p_current.u + p_increment.u, p_current.ux + p_increment.ux, p_current.uy + p_increment.uy,
					p_current.v + p_increment.v, p_current.vx + p_increment.vx, p_current.vy + p_increment.vy);

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
		}

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
			poi->result.zncc = -5;
		}
	}

	void NR2D1::compute(std::vector<POI2D>& poi_queue)
	{
		auto queue_length = poi_queue.size();
#pragma omp parallel for
		for (int i = 0; i < queue_length; i++)
		{
			compute(&poi_queue[i]);
		}
	}

}//namespace opencorr