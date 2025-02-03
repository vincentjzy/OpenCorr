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

#ifndef _ICLM_H_
#define _ICLM_H_

#include <memory>

#include "oc_cubic_bspline.h"
#include "oc_dic.h"
#include "oc_gradient.h"

namespace opencorr
{
	//this part of module is the implementation of
	//B. Chen & E. Jungstedt, Optics and Lasers in Engineering (2022) 151: 106930.
	//https://doi.org/10.1016/j.optlaseng.2021.106930

	struct DampingParameter
	{
		float lambda = 100.f;
		float alpha = 0.1f;
		float beta = 10.f;
	};

	class ICLM2D1_
	{
	public:
		std::unique_ptr<Subset2D> ref_subset;
		std::unique_ptr<Subset2D> tar_subset;
		Eigen::MatrixXf error_img;
		Matrix6f hessian;
		Matrix6f inv_hessian;
		float*** sd_img; //steepest descent image

		static std::unique_ptr<ICLM2D1_> allocate(int subset_radius_x, int subset_radius_y);
		static void release(std::unique_ptr<ICLM2D1_>& instance);
		static void update(std::unique_ptr<ICLM2D1_>& instance, int subset_radius_x, int subset_radius_y);
	};

	class ICLM2D1 : public DIC
	{
	private:
		std::unique_ptr<Interpolation2D> tar_interp; //interpolation for generating target subset during iteration
		std::unique_ptr<Gradient2D4> ref_gradient; //gradient for calculating Hessian matrix of reference subset

		float conv_criterion; //convergence criterion: norm of maximum deformation increment in subset
		float stop_condition; //stop condition: max iteration

		DampingParameter damping; //damping parameters

		std::vector<std::unique_ptr<ICLM2D1_>> instance_pool; //pool of instances for multi-thread processing
		std::unique_ptr<ICLM2D1_>& getInstance(int tid); //get an instance according to the number of current thread id

	public:
		ICLM2D1(int subset_radius_x, int subset_radius_y, float conv_criterion, float stop_condition, int thread_number);
		~ICLM2D1();

		void setIteration(float conv_criterion, float stop_condition);
		void setIteration(POI2D* poi);

		void setDamping(float lambda, float alpha, float beta);

		void prepareRef(); //calculate gradient maps of ref image
		void prepareTar(); //calculate interpolation coefficient look_up table of tar image
		void prepare(); //calculate gradient maps of ref image and interpolation coefficient look_up table of tar image

		void compute(POI2D* poi);
		void compute(std::vector<POI2D>& poi_queue);
	};



	class ICLM2D2_
	{
	public:
		std::unique_ptr<Subset2D> ref_subset;
		std::unique_ptr<Subset2D> tar_subset;
		Eigen::MatrixXf error_img;
		Matrix12f hessian;
		Matrix12f inv_hessian;
		float*** sd_img;

		static std::unique_ptr<ICLM2D2_> allocate(int subset_radius_x, int subset_radius_y);
		static void release(std::unique_ptr<ICLM2D2_>& instance);
		static void update(std::unique_ptr<ICLM2D2_>& instance, int subset_radius_x, int subset_radius_y);
	};

	class ICLM2D2 : public DIC
	{
	private:
		std::unique_ptr<Interpolation2D> tar_interp;
		std::unique_ptr<Gradient2D4> ref_gradient;

		float conv_criterion;
		float stop_condition;

		DampingParameter damping;

		std::vector<std::unique_ptr<ICLM2D2_>> instance_pool;
		std::unique_ptr<ICLM2D2_>& getInstance(int tid);

	public:
		ICLM2D2(int subset_radius_x, int subset_radius_y, float conv_criterion, float stop_condition, int thread_number);
		~ICLM2D2();

		void setIteration(float conv_criterion, float stop_condition);
		void setIteration(POI2D* poi);

		void setDamping(float lambda, float alpha, float beta);

		void prepareRef();
		void prepareTar();
		void prepare();

		void compute(POI2D* poi);
		void compute(std::vector<POI2D>& poi_queue);
	};

}//namespace opencorr

#endif //_ICLM_H_
