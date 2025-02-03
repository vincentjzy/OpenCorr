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

#ifndef _ICGN_GPU_H_
#define _ICGN_GPU_H_

#ifdef OPENCORRGPU_EXPORTS
#define OC_API __declspec(dllexport)
#else
#define OC_API __declspec(dllimport)
#endif

#include <vector>
#include "oc_poi.h"

namespace opencorr
{
	struct Img2D
	{
		int width, height;
		float* data;
	};

	struct Img3D
	{
		int dim_x, dim_y, dim_z;
		float* data;
	};

	//this module is the implementation of
	//L. Zhang et al, Optics and Lasers in Engineering (2015) 69: 7-12.
	//https://doi.org/10.1016/j.optlaseng.2015.01.012

	class OC_API ICGN2D1GPU
	{
	private:
		void* _self;

	public:
		ICGN2D1GPU(int subset_radius_x, int subset_radius_y, float conv_criterion, int stop_condition);

		void setImages(Img2D ref_img, Img2D tar_img);
		void setSubset(int radius_x, int radius_y);
		void setIteration(float convergence_criterion, int stop_condition);

		void prepare();
		void compute(std::vector<POI2D>& poi_queue);
	};

	//this module is the implementation of
	//A. Lin et al, Optics and Lasers in Engineering (2022) 149: 106812.
	//https://doi.org/10.1016/j.optlaseng.2021.106812

	class OC_API ICGN2D2GPU
	{
	private:
		void* _self;

	public:
		ICGN2D2GPU(int subset_radius_x, int subset_radius_y, float conv_criterion, int stop_condition);

		void setImages(Img2D ref_img, Img2D tar_img);
		void setSubset(int radius_x, int radius_y);
		void setIteration(float convergence_criterion, int stop_condition);

		void prepare();
		void compute(std::vector<POI2D>& poi_queue);
	};

	//this module is the implementation of
	//J. Yang et al, Optics and Lasers in Engineering (2021) 136: 106323.
	//https://doi.org/10.1016/j.optlaseng.2020.106323
	
	class OC_API ICGN3D1GPU
	{
	private:
		void* _self;

	public:
		ICGN3D1GPU(int subset_radius_x, int subset_radius_y, int subset_radius_z, float conv_criterion, int stop_condition);

		void setImages(Img3D ref_img, Img3D tar_img);
		void setSubset(int radius_x, int radius_y, int radius_z);
		void setIteration(float convergence_criterion, int stop_condition);

		void prepare();
		void compute(std::vector<POI3D>& poi_queue);
	};

}//namespace opencorr

#endif //_ICGN_GPU_H_