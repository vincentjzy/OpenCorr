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

#include <fstream>

#include "oc_image.h"

namespace opencorr
{
	//2D image
	Image2D::Image2D(int width, int height)
	{
		eg_mat = Eigen::MatrixXf::Zero(height, width);
		this->width = width;
		this->height = height;
		size = height * width;
	}

	Image2D::Image2D(std::string file_path)
	{
		cv_mat = cv::imread(file_path, cv::IMREAD_GRAYSCALE);

		if (!cv_mat.data)
		{
			throw std::string("Fail to load file: " + file_path);
		}

		this->file_path = file_path;
		width = cv_mat.cols;
		height = cv_mat.rows;
		size = height * width;
		eg_mat.resize(height, width);

		cv::cv2eigen(cv_mat, eg_mat);
	}

	void Image2D::load(std::string file_path)
	{
		cv_mat = cv::imread(file_path, cv::IMREAD_GRAYSCALE);

		if (!cv_mat.data)
		{
			throw std::string("Fail to load file: " + file_path);
		}

		this->file_path = file_path;

		if (width != cv_mat.cols || height != cv_mat.rows)
		{
			width = cv_mat.cols;
			height = cv_mat.rows;
			size = height * width;
			eg_mat.resize(height, width);
		}

		cv::cv2eigen(cv_mat, eg_mat);
	}


	//3D image
	Image3D::Image3D(int dim_x, int dim_y, int dim_z)
	{
		vol_mat = new3D(dim_z, dim_y, dim_x);
		this->dim_x = dim_x;
		this->dim_y = dim_y;
		this->dim_z = dim_z;
		size = dim_z * dim_y * dim_x;
	}

	Image3D::Image3D(std::string file_path)
	{
		//check if the file is a bin or tiff
		size_t dot_pos = file_path.find_last_of(".");
		std::string file_ext = file_path.substr(dot_pos + 1);
		if (file_ext == "bin" || file_ext == "BIN")
		{
			loadBin(file_path);
		}
		else if (file_ext == "tif" || file_ext == "TIF" || file_ext == "tiff" || file_ext == "TIFF")
		{
			loadTiff(file_path);
		}
		else
		{
			std::cerr << "Not binary file or multi-page tiff" << std::endl;
		}
	}

	Image3D::~Image3D()
	{
		if (vol_mat != nullptr)
		{
			delete3D(vol_mat);
		}
	}

	void Image3D::loadBin(std::string file_path)
	{
		if (vol_mat != nullptr)
		{
			delete3D(vol_mat);
		}

		std::ifstream file_in;
		file_in.open(file_path, std::ios::in | std::ios::binary);

		if (!file_in.is_open())
		{
			std::cerr << "Failed to open bin file: " << file_path << std::endl;
		}

		this->file_path = file_path;

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
		size = dim_z * dim_y * dim_x;

		//create a 3D matrix and fill it with the data (float) in binary file
		vol_mat = new3D(dim_z, dim_y, dim_x);
		file_in.read((char*)**vol_mat, sizeof(float) * size);

		file_in.close();
	}

	void Image3D::loadTiff(std::string file_path)
	{
		if (vol_mat != nullptr)
		{
			delete3D(vol_mat);
		}

		//read a tiff image consisting of multiple pages and store it in a vector of cv::Mat
		std::vector<cv::Mat> tiff_mat;
		if (!cv::imreadmulti(file_path, tiff_mat, cv::IMREAD_GRAYSCALE))
		{
			std::cerr << "Fail to load multi-page tiff: " + file_path << std::endl;
		}

		//get the three dimensions of 3D image
		dim_x = tiff_mat[0].cols;
		dim_y = tiff_mat[0].rows;
		dim_z = (int)tiff_mat.size();
		size = dim_z * dim_y * dim_x;

		//create a 3D matrix and fill it with the data ifnTIFF
		vol_mat = new3D(dim_z, dim_y, dim_x);

#pragma omp parallel for
		for (int i = 0; i < dim_z; i++)
		{
			for (int j = 0; j < dim_y; j++)
			{
				for (int k = 0; k < dim_x; k++) {
					vol_mat[i][j][k] = (float)tiff_mat[i].at<uchar>(j, k);
				}
			}
		}
	}

	void Image3D::load(std::string file_path)
	{
		//check if the file is a bin or tiff
		size_t dot_pos = file_path.find_last_of(".");
		std::string file_ext = file_path.substr(dot_pos + 1);
		if (file_ext == "bin" || file_ext == "BIN")
		{
			loadBin(file_path);
		}
		else if (file_ext == "tif" || file_ext == "TIF" || file_ext == "tiff" || file_ext == "TIFF")
		{
			loadTiff(file_path);
		}
		else
		{
			std::cerr << "Not binary file or multi-page tiff" << std::endl;
		}
	}

}//namespace opencorr

