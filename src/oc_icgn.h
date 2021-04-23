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

#pragma once

#ifndef _ICGN_H_
#define _ICGN_H_

#include <omp.h>
#include <cmath>
#include "oc_bicubic_bspline.h"
#include "oc_dic.h"
#include "oc_gradient.h"
#include "oc_image.h"
#include "oc_interpolation.h"
#include "oc_poi.h"
#include "oc_point.h"
#include "oc_subset.h"

namespace opencorr
{
	//1st shape function
	class ICGN2D1_
	{
	public:
		Subset2D* ref_subset;
		Subset2D* tar_subset;
		Eigen::MatrixXf error_img;
		Matrix6f hessian, inv_hessian;
		float*** steepest_descent_img;
		Deformation2D1 p_initial, p_current, p_increment;

		static ICGN2D1_* allocate(int subset_radius_x, int subset_radius_y);
		static void release(ICGN2D1_* instance);
	};

	class ICGN2D1 : public DIC
	{
	private:
		Interpolation2D* tar_interp;
		Gradient2D4* ref_gradient;

		float convergence_criterion; // Convergence criterion: Norm of maximum deformation increment in subset
		float stop_condition; // Stop condition: Max iteration

		std::vector<ICGN2D1_*> instance_pool;
		ICGN2D1_* getInstance(int tid);

	public:
		ICGN2D1();
		ICGN2D1(int subset_radius_x, int subset_radius_y, float convergence_criterion, float stop_condition);
		~ICGN2D1();

		void prepare();
		void compute(POI2D* POI);

		void setSubsetRadii(int subset_radius_x, int subset_radius_y);
		void setIteration(float convergence_criterion, float stop_condition);
		void setIteration(POI2D* POI);
	};

	//2nd shape function/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	class ICGN2D2_
	{
	public:
		Subset2D* ref_subset;
		Subset2D* tar_subset;
		Eigen::MatrixXf error_img;
		Matrix12f hessian, inv_hessian;
		float*** steepest_descent_img;
		Deformation2D1 p_initial;
		Deformation2D2 p_current, p_increment;

		static ICGN2D2_* allocate(int subset_radius_x, int subset_radius_y);
		static void release(ICGN2D2_* instance);
	};

	class ICGN2D2 : public DIC
	{
	private:
		Interpolation2D* tar_interp;
		Gradient2D4* ref_gradient;

		float convergence_criterion; // Convergence criterion: Norm of maximum deformation increment in subset
		float stop_condition; // Stop condition: Max iteration

		std::vector<ICGN2D2_*> instance_pool;
		ICGN2D2_* getInstance(int tid);

	public:
		ICGN2D2();
		ICGN2D2(int subset_radius_x, int subset_radius_y, float convergence_criterion, float stop_condition);
		~ICGN2D2();

		void prepare();
		void compute(POI2D* POI);

		void setSubsetRadii(int subset_radius_x, int subset_radius_y);
		void setIteration(float convergence_criterion, float stop_condition);
		void setIteration(POI2D* POI);

	};

}//namespace opencorr

#endif //_ICGN_H_
