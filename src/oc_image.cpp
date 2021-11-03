/*
 * This file is part of OpenCorr, an open source C++ library for
 * study and development of 2D, 3D/stereo and volumetric
 * digital image correlation.
 *
 * Copyright (C) 2021, Zhenyu Jiang <zhenyujiang@scut.edu.cn>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * More information about OpenCorr can be found at https://www.opencorr.org/
 */

#include "oc_image.h"

namespace opencorr
{

	Image2D::Image2D(int width, int height) {
		eg_mat = Eigen::MatrixXf::Zero(height, width);
		this->width = width;
		this->height = height;
	}

	Image2D::Image2D(string file_path) {
		cv_mat = cv::imread(file_path, CV_LOAD_IMAGE_GRAYSCALE);

		if (!cv_mat.data) {
			throw std::string("Fail to load file: " + file_path);
		}

		this->file_path = file_path;
		width = cv_mat.cols;
		height = cv_mat.rows;
		eg_mat.resize(height, width);

		cv::cv2eigen(cv_mat, eg_mat);
	}

	void Image2D::load(string file_path) {
		cv_mat = cv::imread(file_path, CV_LOAD_IMAGE_GRAYSCALE);

		if (!cv_mat.data) {
			throw std::string("Fail to load file: " + file_path);
		}

		this->file_path = file_path;
		if (width != cv_mat.cols || height != cv_mat.rows) {
			width = cv_mat.cols;
			height = cv_mat.rows;
			eg_mat.resize(height, width);
		}

		cv::cv2eigen(cv_mat, eg_mat);
	}

}//namespace opencorr

