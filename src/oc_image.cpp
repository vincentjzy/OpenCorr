/*
 * This file is part of OpenCorr, an open source C++ library for
 * study and development of 2D, 3D/stereo and volumetric
 * digital image correlation.
 *
 * Copyright (C) 2021, Zhenyu Jiang <zhenyujiang@scut.edu.cn>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one from http://mozilla.org/MPL/2.0/.
 *
 * More information about OpenCorr can be found at https://www.opencorr.org/
 */

#include <fstream>

#include "oc_image.h"

namespace opencorr
{

	Image2D::Image2D(int width, int height) {
		eg_mat = Eigen::MatrixXf::Zero(height, width);
		this->width = width;
		this->height = height;
	}

	Image2D::Image2D(string file_path) {
		cv_mat = cv::imread(file_path, cv::IMREAD_GRAYSCALE);

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
		cv_mat = cv::imread(file_path, cv::IMREAD_GRAYSCALE);

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

	Image3D::Image3D(int dim_x, int dim_y, int dim_z) {
		vol_mat = new3D(dim_z, dim_y, dim_x);
		this->dim_x = dim_x;
		this->dim_y = dim_y;
		this->dim_z = dim_z;
	}

	Image3D::Image3D(string file_path) {
		std::ifstream file_in;
		file_in.open(file_path, std::ios::in | std::ios::binary);

		if (!file_in.is_open()) {
			throw std::string("Fail to open file: " + file_path);
		}

		//get the length of data
		file_in.seekg(0, file_in.end);
		int file_length = file_in.tellg();
		file_in.seekg(0, file_in.beg);
		int data_length = file_length - sizeof(int) * 3;

		//head information is an array of int[3]: dimension of x, y, and z
		int img_dimension[3];
		file_in.read((char*)img_dimension, sizeof(int) * 3);
		dim_x = img_dimension[0];
		dim_y = img_dimension[1];
		dim_z = img_dimension[2];

		//create a 3D matrix and fill it with the data (float) in binary file
		vol_mat = new3D(dim_z, dim_y, dim_x);
		int matrix_size = dim_z * dim_y * dim_x;
		file_in.read((char*)**vol_mat, sizeof(float) * matrix_size);

		file_in.close();
	}

	Image3D::~Image3D() {
		if (vol_mat != nullptr) {
			delete3D(vol_mat);
		}
	}

	void Image3D::load(string file_path) {
		if (vol_mat != nullptr) {
			delete3D(vol_mat);
		}

		std::ifstream file_in;
		file_in.open(file_path, std::ios::in | std::ios::binary);

		if (!file_in.is_open()) {
			std::cerr << "failed to open file " << file_path << std::endl;
		}

		//get the length of data
		file_in.seekg(0, file_in.end);
		int file_length = file_in.tellg();
		file_in.seekg(0, file_in.beg);
		int data_length = file_length - sizeof(int) * 3;

		//head information is an array of int[3]: dimension of x, y, and z
		int img_dimension[3];
		file_in.read((char*)img_dimension, sizeof(int) * 3);
		dim_x = img_dimension[0];
		dim_y = img_dimension[1];
		dim_z = img_dimension[2];

		//create a 3D matrix and fill it with the data (float) in binary file
		vol_mat = new3D(dim_z, dim_y, dim_x);
		int matrix_size = dim_z * dim_y * dim_x;
		file_in.read((char*)**vol_mat, sizeof(float) * matrix_size);

		file_in.close();
	}

	void Image3D::loadTIFF(string file_path) {
		if (vol_mat != nullptr) {
			delete3D(vol_mat);
		}

		//read a tiff image consisting of multiple pages and store it in a vector of cv::Mat
		std::vector<cv::Mat> tiff_mat;
		if (!cv::imreadmulti(file_path, tiff_mat, cv::IMREAD_GRAYSCALE)) {
			throw std::string("Fail to load multi-page tiff: " + file_path);
		}

		//get the three dimensions of 3D image
		dim_x = tiff_mat[0].cols;
		dim_y = tiff_mat[0].rows;
		dim_z = (int)tiff_mat.size();

		//create a 3D matrix and fill it with the data ifnTIFF
		vol_mat = new3D(dim_z, dim_y, dim_x);
		int matrix_size = dim_x * dim_y * dim_z;
#pragma omp parallel for
		for (int i = 0; i < dim_z; i++) {
			for (int j = 0; j < dim_y; j++) {
				for (int k = 0; k < dim_x; k++) {
					vol_mat[i][j][k] = (float)tiff_mat[i].at<uchar>(j, k);
				}
			}
		}
	}

}//namespace opencorr

