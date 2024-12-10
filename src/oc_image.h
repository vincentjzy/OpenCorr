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

#ifndef _IMAGE_H_
#define _IMAGE_H_

#include <Eigen>
#include <opencv2/opencv.hpp>
//#include <opencv2/world.hpp>
#include <opencv2/core/eigen.hpp>

#include "oc_array.h"

namespace opencorr
{
	class Image2D
	{
	public:
		int height, width;
		unsigned int size;

		std::string file_path;

		cv::Mat cv_mat;
		Eigen::MatrixXf eg_mat;

		Image2D(int width, int height);
		Image2D(std::string file_path);
		~Image2D() = default;

		void load(std::string file_path);
	};

	class Image3D
	{
	public:
		int dim_x, dim_y, dim_z;
		unsigned long size;

		std::string file_path;

		float*** vol_mat = nullptr;

		Image3D(int dim_x, int dim_y, int dim_z);
		Image3D(std::string file_path);
		~Image3D() = default;

		void loadBin(std::string file_path);
		void loadTiff(std::string file_path);
		void load(std::string file_path);

		void release();
	};

}//namespace opencorr

#endif //_IMAGE_H_