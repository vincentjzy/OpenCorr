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

#include "oc_gradient.h"

namespace opencorr
{
	//order of derivative: 1, order of accuracy: 4
	Gradient2D4::Gradient2D4(Image2D& image)
	{
		grad_img = &image;
	}

	Gradient2D4::~Gradient2D4() {}

	void Gradient2D4::getGradientX()
	{
		int height = grad_img->height;
		int width = grad_img->width;

		gradient_x = Eigen::MatrixXf::Zero(height, width);

#pragma omp parallel for
		for (int r = 0; r < height; r++)
		{
			for (int c = 2; c < width - 2; c++)
			{
				float result = 0.0f;
				result -= grad_img->eg_mat(r, c + 2) / 12.f;
				result += grad_img->eg_mat(r, c + 1) * (2.f / 3.f);
				result -= grad_img->eg_mat(r, c - 1) * (2.f / 3.f);
				result += grad_img->eg_mat(r, c - 2) / 12.f;
				gradient_x(r, c) = result;
			}
		}
	}

	void Gradient2D4::getGradientY()
	{
		int height = grad_img->height;
		int width = grad_img->width;

		gradient_y = Eigen::MatrixXf::Zero(height, width);

#pragma omp parallel for
		for (int r = 2; r < height - 2; r++)
		{
			for (int c = 0; c < width; c++)
			{
				float result = 0.0f;
				result -= grad_img->eg_mat(r + 2, c) / 12.f;
				result += grad_img->eg_mat(r + 1, c) * (2.f / 3.f);
				result -= grad_img->eg_mat(r - 1, c) * (2.f / 3.f);
				result += grad_img->eg_mat(r - 2, c) / 12.f;
				gradient_y(r, c) = result;
			}
		}
	}

	void Gradient2D4::getGradientXY()
	{
		int height = grad_img->height;
		int width = grad_img->width;

		gradient_xy = Eigen::MatrixXf::Zero(height, width);

		if (gradient_x.rows() != height || gradient_x.cols() != width)
		{
			getGradientX();
		}

#pragma omp parallel for
		for (int r = 2; r < height - 2; r++)
		{
			for (int c = 0; c < width; c++)
			{
				float result = 0.0f;
				result -= gradient_x(r + 2, c) / 12.f;
				result += gradient_x(r + 1, c) * (2.f / 3.f);
				result -= gradient_x(r - 1, c) * (2.f / 3.f);
				result += gradient_x(r - 2, c) / 12.f;
				gradient_xy(r, c) = result;
			}
		}
	}

	Gradient3D4::Gradient3D4(Image3D& image)
	{
		grad_img = &image;
	}

	Gradient3D4::~Gradient3D4()
	{
		if (gradient_x != nullptr)
		{
			delete3D(gradient_x);
		}
		if (gradient_y != nullptr)
		{
			delete3D(gradient_y);
		}
		if (gradient_z != nullptr)
		{
			delete3D(gradient_z);
		}
	}

	void Gradient3D4::getGradientX()
	{
		int dim_x = grad_img->dim_x;
		int dim_y = grad_img->dim_y;
		int dim_z = grad_img->dim_z;

		if (gradient_x != nullptr)
		{
			delete3D(gradient_x);
		}
		gradient_x = new3D(dim_z, dim_y, dim_x);

#pragma omp parallel for
		for (int i = 0; i < dim_z; i++)
		{
			for (int j = 0; j < dim_y; j++)
			{
				for (int k = 2; k < dim_x - 2; k++)
				{
					float result = 0.0f;
					result -= grad_img->vol_mat[i][j][k + 2] / 12.f;
					result += grad_img->vol_mat[i][j][k + 1] * (2.f / 3.f);
					result -= grad_img->vol_mat[i][j][k - 1] * (2.f / 3.f);
					result += grad_img->vol_mat[i][j][k - 2] / 12.f;
					gradient_x[i][j][k] = result;
				}
			}
		}
	}

	void Gradient3D4::getGradientY()
	{
		int dim_x = grad_img->dim_x;
		int dim_y = grad_img->dim_y;
		int dim_z = grad_img->dim_z;

		if (gradient_y != nullptr)
		{
			delete3D(gradient_y);
		}
		gradient_y = new3D(dim_z, dim_y, dim_x);

#pragma omp parallel for
		for (int k = 0; k < dim_x; k++)
		{
			for (int i = 0; i < dim_z; i++)
			{
				for (int j = 2; j < dim_y - 2; j++)
				{
					float result = 0.0f;
					result -= grad_img->vol_mat[i][j + 2][k] / 12.f;
					result += grad_img->vol_mat[i][j + 1][k] * (2.f / 3.f);
					result -= grad_img->vol_mat[i][j - 1][k] * (2.f / 3.f);
					result += grad_img->vol_mat[i][j - 2][k] / 12.f;
					gradient_y[i][j][k] = result;
				}
			}
		}
	}

	void Gradient3D4::getGradientZ()
	{
		int dim_x = grad_img->dim_x;
		int dim_y = grad_img->dim_y;
		int dim_z = grad_img->dim_z;

		if (gradient_z != nullptr)
		{
			delete3D(gradient_z);
		}
		gradient_z = new3D(dim_z, dim_y, dim_x);

#pragma omp parallel for
		for (int j = 0; j < dim_y; j++)
		{
			for (int k = 0; k < dim_x; k++)
			{
				for (int i = 2; i < dim_z - 2; i++)
				{
					float result = 0.0f;
					result -= grad_img->vol_mat[i + 2][j][k] / 12.f;
					result += grad_img->vol_mat[i + 1][j][k] * (2.f / 3.f);
					result -= grad_img->vol_mat[i - 1][j][k] * (2.f / 3.f);
					result += grad_img->vol_mat[i - 2][j][k] / 12.f;
					gradient_z[i][j][k] = result;
				}
			}
		}
	}

} //namespcae opencorr