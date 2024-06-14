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

#pragma once

#ifndef _ICGN_H_
#define _ICGN_H_

#include "oc_cubic_bspline.h"
#include "oc_dic.h"
#include "oc_gradient.h"
#include "oc_image.h"
#include "oc_interpolation.h"
#include "oc_poi.h"
#include "oc_point.h"
#include "oc_subset.h"

namespace opencorr
{
	//this part of module is the implementation of
	//Z. Jiang et al, Optics and Lasers in Engineering (2015) 65: 93-102.
	//https://doi.org/10.1016/j.optlaseng.2014.06.011

	class ICGN2D1_
	{
	public:
		Subset2D* ref_subset;
		Subset2D* tar_subset;
		Eigen::MatrixXf error_img;
		Matrix6f hessian, inv_hessian;
		float*** sd_img; //steepest descent image

		static ICGN2D1_* allocate(int subset_radius_x, int subset_radius_y);
		static void release(ICGN2D1_* instance);
		static void update(ICGN2D1_* instance, int subset_radius_x, int subset_radius_y);
	};

	class ICGN2D1 : public DIC
	{
	private:
		Interpolation2D* tar_interp; //interpolation for generating target subset during iteration
		Gradient2D4* ref_gradient; //gradient for calculating Hessian matrix of reference subset

		float conv_criterion; //convergence criterion: norm of maximum deformation increment in subset
		float stop_condition; //stop condition: max iteration

		std::vector<ICGN2D1_*> instance_pool; //pool of instances for multi-thread processing
		ICGN2D1_* getInstance(int tid); //get an instance according to the number of current thread id

	public:
		ICGN2D1(int subset_radius_x, int subset_radius_y, float conv_criterion, float stop_condition, int thread_number);
		~ICGN2D1();

		void prepareRef(); //calculate gradient maps of ref image
		void prepareTar(); //calculate interpolation coefficient look_up table of tar image
		void prepare(); //calculate gradient maps of ref image and interpolation coefficient look_up table of tar image

		void compute(POI2D* poi);
		void compute(std::vector<POI2D>& poi_queue);

		void setIteration(float conv_criterion, float stop_condition);
		void setIteration(POI2D* poi);
	};



	//this part of module is the implementation of
	//Y. Gao et al, Optics and Lasers in Engineering (2015) 65: 73-80.
	//https://doi.org/10.1016/j.optlaseng.2014.05.013

	class ICGN2D2_
	{
	public:
		Subset2D* ref_subset;
		Subset2D* tar_subset;
		Eigen::MatrixXf error_img;
		Matrix12f hessian, inv_hessian;
		float*** sd_img;

		static ICGN2D2_* allocate(int subset_radius_x, int subset_radius_y);
		static void release(ICGN2D2_* instance);
		static void update(ICGN2D2_* instance, int subset_radius_x, int subset_radius_y);
	};

	class ICGN2D2 : public DIC
	{
	private:
		Interpolation2D* tar_interp;
		Gradient2D4* ref_gradient;

		float conv_criterion;
		float stop_condition;

		std::vector<ICGN2D2_*> instance_pool;
		ICGN2D2_* getInstance(int tid);

	public:
		ICGN2D2(int subset_radius_x, int subset_radius_y, float conv_criterion, float stop_condition, int thread_number);
		~ICGN2D2();

		void prepareRef();
		void prepareTar();
		void prepare();

		void compute(POI2D* poi);
		void compute(std::vector<POI2D>& poi_queue);

		void setIteration(float conv_criterion, float stop_condition);
		void setIteration(POI2D* poi);
	};



	//the 3D part of module is the implementation of
	//J. Yang et al, Optics and Lasers in Engineering (2021) 136: 106323.
	//https://doi.org/10.1016/j.optlaseng.2020.106323

	class ICGN3D1_
	{
	public:
		Subset3D* ref_subset;
		Subset3D* tar_subset;
		float*** error_img;
		Matrix12f hessian, inv_hessian;
		float**** sd_img; //steepest descent image

		static ICGN3D1_* allocate(int subset_radius_x, int subset_radius_y, int subset_radius_z);
		static void release(ICGN3D1_* instance);
		static void update(ICGN3D1_* instance, int subset_radius_x, int subset_radius_y, int subset_radius_z);
	};

	class ICGN3D1 : public DVC
	{
	private:
		Interpolation3D* tar_interp; //interpolation for generating target subset during iteration
		Gradient3D4* ref_gradient; //gradient for calculating Hessian matrix of reference subset

		float conv_criterion; //convergence criterion: norm of maximum displacement increment in subset
		float stop_condition; //stop condition: max iteration

		std::vector<ICGN3D1_*> instance_pool; //pool of instances for multi-thread processing
		ICGN3D1_* getInstance(int tid); //get an instance according to the number of current thread id

	public:
		ICGN3D1(int subset_radius_x, int subset_radius_y, int subset_radius_z,
			float conv_criterion, float stop_condition, int thread_number);
		~ICGN3D1();

		void prepareRef(); //calculate gradient matrices of ref image
		void prepareTar(); //calculate interpolation coefficient matrix of tar image
		void prepare(); //calculate gradient matrices of ref image and interpolation coefficient matrix of tar image

		void compute(POI3D* poi);
		void compute(std::vector<POI3D>& poi_queue);

		void setIteration(float conv_criterion, float stop_condition);
		void setIteration(POI3D* poi);
	};

}//namespace opencorr

#endif //_ICGN_H_
