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

#include "oc_subset.h"

namespace opencorr
{
	//2D susbet
	Subset2D::Subset2D(Point2D center, int radius_x, int radius_y)
	{
		if (radius_x < 1 || radius_y < 1)
		{
			std::cerr << "Too small radius:" << radius_x << ", " << radius_y << std::endl;
		}

		this->center = center;
		this->radius_x = radius_x;
		this->radius_y = radius_y;
		width = radius_x * 2 + 1;
		height = radius_y * 2 + 1;

		eg_mat = Eigen::MatrixXf::Zero(height, width);
	}

	void Subset2D::fill(Image2D* image)
	{
		Point2D topleft_point(center.x - radius_x, center.y - radius_y);
		eg_mat << image->eg_mat.block(topleft_point.y, topleft_point.x, height, width);
	}

	float Subset2D::zeroMeanNorm()
	{
		float subset_mean = eg_mat.mean();
		eg_mat.array() -= subset_mean;
		float subset_sum = eg_mat.squaredNorm();

		return sqrt(subset_sum);
	}


	//3D subvolume
	Subset3D::Subset3D(Point3D center, int radius_x, int radius_y, int radius_z)
	{
		if (radius_x < 1 || radius_y < 1 || radius_z < 1)
		{
			std::cerr << "Too small radius:" << radius_x << ", " << radius_y << ", " << radius_z << std::endl;
		}

		if (vol_mat != nullptr)
		{
			delete3D(vol_mat);
		}

		this->center = center;
		this->radius_x = radius_x;
		this->radius_y = radius_y;
		this->radius_z = radius_z;
		dim_x = radius_x * 2 + 1;
		dim_y = radius_y * 2 + 1;
		dim_z = radius_z * 2 + 1;

		vol_mat = new3D(dim_z, dim_y, dim_x);
	}

	Subset3D::~Subset3D()
	{
		if (vol_mat != nullptr)
		{
			delete3D(vol_mat);
		}
	}

	void Subset3D::fill(Image3D* image)
	{
		Point3D start_point(center.x - radius_x, center.y - radius_y, center.z - radius_z);
		for (int i = 0; i < dim_z; i++)
		{
			for (int j = 0; j < dim_y; j++)
			{
				for (int k = 0; k < dim_x; k++)
				{
					vol_mat[i][j][k] = image->vol_mat[int(start_point.z + i)][int(start_point.y + j)][int(start_point.x + k)];
				}
			}
		}
	}

	float Subset3D::zeroMeanNorm()
	{
		//calculate the mean of gray-scale values
		float mean_value = 0;
		for (int i = 0; i < dim_z; i++)
		{
			for (int j = 0; j < dim_y; j++)
			{
				for (int k = 0; k < dim_x; k++)
				{
					mean_value += vol_mat[i][j][k];
				}
			}
		}
		mean_value /= (dim_x * dim_y * dim_z);

		//make the distribution of gray-scale values zero-mean
		float subset_sum = 0;
		for (int i = 0; i < dim_z; i++)
		{
			for (int j = 0; j < dim_y; j++)
			{
				for (int k = 0; k < dim_x; k++)
				{
					vol_mat[i][j][k] -= mean_value;
					subset_sum += (vol_mat[i][j][k] * vol_mat[i][j][k]);
				}
			}
		}

		return sqrt(subset_sum);
	}

}//namespace opencorr