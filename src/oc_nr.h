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

#pragma once

#ifndef _NR_H_
#define _NR_H_

#include "oc_cubic_bspline.h"
#include "oc_dic.h"
#include "oc_gradient.h"

namespace opencorr
{
	//this part of module is the implementation of
	//W. Chen et al, Experimental Mechanics (2017) 57(6): 979-996.
	//https://doi.org/10.1007/s11340-017-0294-y

	class NR2D1_
	{
	public:
		std::unique_ptr<Subset2D> ref_subset;
		std::unique_ptr<Subset2D> tar_subset;
		Eigen::MatrixXf tar_gradient_x;
		Eigen::MatrixXf tar_gradient_y;
		Eigen::MatrixXf error_img;
		Matrix6f hessian;
		Matrix6f inv_hessian;
		float*** sd_img; //steepest descent image

		static std::unique_ptr<NR2D1_> allocate(int subset_radius_x, int subset_radius_y);
		static void release(std::unique_ptr<NR2D1_>& instance);
		static void update(std::unique_ptr<NR2D1_>& instance, int subset_radius_x, int subset_radius_y);
	};

	class NR2D1 : public DIC
	{
	private:
		std::unique_ptr<Gradient2D4> tar_gradient; //gradient for calculating Hessian matrix of reference subset
		std::unique_ptr<Interpolation2D> tar_interp; //interpolation for generating target subset during iteration
		std::unique_ptr<Interpolation2D> tar_interp_x; //interpolation for generating target gradient along axis-x during iteration
		std::unique_ptr<Interpolation2D> tar_interp_y; //interpolation for generating target gradient along axis-y during iteration

		float conv_criterion; //convergence criterion: norm of maximum deformation increment in subset
		float stop_condition; //stop condition: max iteration

		std::vector<std::unique_ptr<NR2D1_>> instance_pool; //pool of instances for multi-thread processing
		std::unique_ptr<NR2D1_>& getInstance(int tid); //get an instance according to the number of current thread id

	public:
		NR2D1(int subset_radius_x, int subset_radius_y, float conv_criterion, float stop_condition, int thread_number);
		~NR2D1();

		void prepare(); //calculate gradient maps and interpolation coefficient tables of tar image and gradients

		void compute(POI2D* poi);
		void compute(std::vector<POI2D>& poi_queue);

		void setIteration(float conv_criterion, float stop_condition);
		void setIteration(POI2D* poi);
	};

}//namespace opencorr

#endif //_NR_H_
