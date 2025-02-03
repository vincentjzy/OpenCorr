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

#include "oc_cubic_bspline.h"

namespace opencorr
{
	//auxiliary functions
	//return the lower value of the two inputs
	static int getLow(int x, int y)
	{
		return x < y ? x : y;
	}

	//retrun the higher value of the two inputs
	static int getHigh(int x, int y)
	{
		return x > y ? x : y;
	}

	//Four cubic B-spline basis functions when input falls in different range
	static float basis0(float coor_decimal)
	{
		return (1.f / 6.f) * (coor_decimal * (coor_decimal * (-coor_decimal + 3.f) - 3.f) + 1.f); //(1/6)*(2-(x+1))^3 for x-(-1)
	}

	static float basis1(float coor_decimal)
	{
		return (1.f / 6.f) * (coor_decimal * coor_decimal * (3.f * coor_decimal - 6.f) + 4.f); //(2/3)-(1/2)*(2-x)*x^2 for x-0
	}

	static float basis2(float coor_decimal)
	{
		return (1.f / 6.f) * (coor_decimal * (coor_decimal * (-3.f * coor_decimal + 3.f) + 3.f) + 1.f); //(2/3)-(1/2)*(2-(1-x))*(1-x)^2 for x-1
	}

	static float basis3(float coor_decimal)
	{
		return (1.f / 6.f) * (coor_decimal * coor_decimal * coor_decimal); //(1/6)*(2-(2-x))^3 for x-2
	}

	//bicubic B-spline interpolation
	BicubicBspline::BicubicBspline(Image2D& image)
		: coefficient(nullptr)
	{
		setImage(image);
	}

	BicubicBspline::~BicubicBspline()
	{
		if (coefficient != nullptr)
		{
			delete4D(coefficient);
		}
	}

	void BicubicBspline::setImage(Image2D& image)
	{
		if (image.height < 5 || image.width < 5)
		{
			std::cerr << "Too small image:" << image.width << ", " << image.height << std::endl;
		}
		else
		{
			interp_img = &image;
			height = image.height;
			width = image.width;
		}
	}

	void BicubicBspline::prepare()
	{
		if (coefficient != nullptr)
		{
			delete4D(coefficient);
		}
		coefficient = new4D(height, width, 4, 4);

#pragma omp parallel for
		for (int r = 1; r < height - 2; r++)
		{
			for (int c = 1; c < width - 2; c++)
			{
				//fill grayscale values into 4x4 grid
				float mat_q[4][4] = { 0.f };
				for (int i = 0; i < 4; i++)
				{
					for (int j = 0; j < 4; j++) {
						mat_q[i][j] = interp_img->eg_mat(r - 1 + i, c - 1 + j);
					}
				}

				//calculate interpolation coefficient matrix
				float mat_p[4][4] = { 0.f };
				for (int k = 0; k < 4; k++)
				{
					for (int l = 0; l < 4; l++)
					{
						for (int m = 0; m < 4; m++)
						{
							for (int n = 0; n < 4; n++)
							{
								mat_p[k][l] += BC_MATRIX[l][m] * BC_MATRIX[k][n] * mat_q[n][m];
							}
						}
					}
				}

				//rearrange the order of coefficient matrix
				for (int k = 0; k < 4; k++)
				{
					for (int l = 0; l < 4; l++)
					{
						coefficient[r][c][k][l] = mat_p[3 - k][3 - l];
					}
				}
			}
		}
	}

	float BicubicBspline::compute(Point2D& location)
	{
		float value = 0.f;
		if (location.x < 1 || location.y < 1
			|| location.x >= width - 2 || location.y >= height - 2
			|| std::isnan(location.x) || std::isnan(location.y))
		{
			value = -1.f;
		}
		else
		{
			int x_integral = floor(location.x);
			int y_integral = floor(location.y);

			float x_decimal = location.x - x_integral;
			float y_decimal = location.y - y_integral;

			float x2_decimal = x_decimal * x_decimal;
			float y2_decimal = y_decimal * y_decimal;

			float x3_decimal = x2_decimal * x_decimal;
			float y3_decimal = y2_decimal * y_decimal;

			float**& local_coefficient = coefficient[y_integral][x_integral];

			value = local_coefficient[0][0]
				+ local_coefficient[0][1] * x_decimal
				+ local_coefficient[0][2] * x2_decimal
				+ local_coefficient[0][3] * x3_decimal

				+ local_coefficient[1][0] * y_decimal
				+ local_coefficient[1][1] * y_decimal * x_decimal
				+ local_coefficient[1][2] * y_decimal * x2_decimal
				+ local_coefficient[1][3] * y_decimal * x3_decimal

				+ local_coefficient[2][0] * y2_decimal
				+ local_coefficient[2][1] * y2_decimal * x_decimal
				+ local_coefficient[2][2] * y2_decimal * x2_decimal
				+ local_coefficient[2][3] * y2_decimal * x3_decimal

				+ local_coefficient[3][0] * y3_decimal
				+ local_coefficient[3][1] * y3_decimal * x_decimal
				+ local_coefficient[3][2] * y3_decimal * x2_decimal
				+ local_coefficient[3][3] * y3_decimal * x3_decimal;
		}

		return value;
	}


	//tricubic B-spline interpolation
	TricubicBspline::TricubicBspline(Image3D& image)
		: coefficient(nullptr)
	{
		setImage(image);
	}

	TricubicBspline::~TricubicBspline()
	{
		if (coefficient != nullptr)
		{
			delete3D(coefficient);
		}
	}

	void TricubicBspline::setImage(Image3D& image)
	{
		if (image.dim_x < 15 || image.dim_y < 15 || image.dim_z < 15)
		{
			std::cerr << "Too small volume image:" << image.dim_x << ", " << image.dim_y << ", " << image.dim_z << std::endl;
		}
		else
		{
			interp_img = &image;
			dim_x = image.dim_x;
			dim_y = image.dim_y;
			dim_z = image.dim_z;
		}
	}

	void TricubicBspline::prepare()
	{
		if (coefficient != nullptr)
		{
			delete3D(coefficient);
		}
		coefficient = new3D(dim_z, dim_y, dim_x);
		float*** conv_buffer = new3D(dim_z, dim_y, dim_x);

		//convolution along x-axis
#pragma omp parallel for
		for (int i = 0; i < dim_z; i++)
		{
			for (int j = 0; j < dim_y; j++)
			{
				for (int k = 7; k < dim_x - 7; k++)
				{
					coefficient[i][j][k] = BSPLINE_PREFILTER[0] * interp_img->vol_mat[i][j][k]
						+ BSPLINE_PREFILTER[1] * (interp_img->vol_mat[i][j][k - 1] + interp_img->vol_mat[i][j][k + 1])
						+ BSPLINE_PREFILTER[2] * (interp_img->vol_mat[i][j][k - 2] + interp_img->vol_mat[i][j][k + 2])
						+ BSPLINE_PREFILTER[3] * (interp_img->vol_mat[i][j][k - 3] + interp_img->vol_mat[i][j][k + 3])
						+ BSPLINE_PREFILTER[4] * (interp_img->vol_mat[i][j][k - 4] + interp_img->vol_mat[i][j][k + 4])
						+ BSPLINE_PREFILTER[5] * (interp_img->vol_mat[i][j][k - 5] + interp_img->vol_mat[i][j][k + 5])
						+ BSPLINE_PREFILTER[6] * (interp_img->vol_mat[i][j][k - 6] + interp_img->vol_mat[i][j][k + 6])
						+ BSPLINE_PREFILTER[7] * (interp_img->vol_mat[i][j][k - 7] + interp_img->vol_mat[i][j][k + 7]);
				}
				for (int k = 0; k < 7; k++)
				{
					coefficient[i][j][k] = BSPLINE_PREFILTER[0] * interp_img->vol_mat[i][j][k]
						+ BSPLINE_PREFILTER[1] * (interp_img->vol_mat[i][j][getHigh(k - 1, 0)] + interp_img->vol_mat[i][j][k + 1])
						+ BSPLINE_PREFILTER[2] * (interp_img->vol_mat[i][j][getHigh(k - 2, 0)] + interp_img->vol_mat[i][j][k + 2])
						+ BSPLINE_PREFILTER[3] * (interp_img->vol_mat[i][j][getHigh(k - 3, 0)] + interp_img->vol_mat[i][j][k + 3])
						+ BSPLINE_PREFILTER[4] * (interp_img->vol_mat[i][j][getHigh(k - 4, 0)] + interp_img->vol_mat[i][j][k + 4])
						+ BSPLINE_PREFILTER[5] * (interp_img->vol_mat[i][j][getHigh(k - 5, 0)] + interp_img->vol_mat[i][j][k + 5])
						+ BSPLINE_PREFILTER[6] * (interp_img->vol_mat[i][j][getHigh(k - 6, 0)] + interp_img->vol_mat[i][j][k + 6])
						+ BSPLINE_PREFILTER[7] * (interp_img->vol_mat[i][j][getHigh(k - 7, 0)] + interp_img->vol_mat[i][j][k + 7]);
				}
				for (int k = dim_x - 7; k < dim_x; k++)
				{
					coefficient[i][j][k] = BSPLINE_PREFILTER[0] * interp_img->vol_mat[i][j][k]
						+ BSPLINE_PREFILTER[1] * (interp_img->vol_mat[i][j][k - 1] + interp_img->vol_mat[i][j][getLow(k + 1, dim_x - 1)])
						+ BSPLINE_PREFILTER[2] * (interp_img->vol_mat[i][j][k - 2] + interp_img->vol_mat[i][j][getLow(k + 2, dim_x - 1)])
						+ BSPLINE_PREFILTER[3] * (interp_img->vol_mat[i][j][k - 3] + interp_img->vol_mat[i][j][getLow(k + 3, dim_x - 1)])
						+ BSPLINE_PREFILTER[4] * (interp_img->vol_mat[i][j][k - 4] + interp_img->vol_mat[i][j][getLow(k + 4, dim_x - 1)])
						+ BSPLINE_PREFILTER[5] * (interp_img->vol_mat[i][j][k - 5] + interp_img->vol_mat[i][j][getLow(k + 5, dim_x - 1)])
						+ BSPLINE_PREFILTER[6] * (interp_img->vol_mat[i][j][k - 6] + interp_img->vol_mat[i][j][getLow(k + 6, dim_x - 1)])
						+ BSPLINE_PREFILTER[7] * (interp_img->vol_mat[i][j][k - 7] + interp_img->vol_mat[i][j][getLow(k + 7, dim_x - 1)]);
				}
			}
		}

		//convolution along y-axis
#pragma omp parallel for
		for (int k = 0; k < dim_x; k++)
		{
			for (int i = 0; i < dim_z; i++)
			{
				for (int j = 7; j < dim_y - 7; j++)
				{
					conv_buffer[i][j][k] = BSPLINE_PREFILTER[0] * coefficient[i][j][k]
						+ BSPLINE_PREFILTER[1] * (coefficient[i][j - 1][k] + coefficient[i][j + 1][k])
						+ BSPLINE_PREFILTER[2] * (coefficient[i][j - 2][k] + coefficient[i][j + 2][k])
						+ BSPLINE_PREFILTER[3] * (coefficient[i][j - 3][k] + coefficient[i][j + 3][k])
						+ BSPLINE_PREFILTER[4] * (coefficient[i][j - 4][k] + coefficient[i][j + 4][k])
						+ BSPLINE_PREFILTER[5] * (coefficient[i][j - 5][k] + coefficient[i][j + 5][k])
						+ BSPLINE_PREFILTER[6] * (coefficient[i][j - 6][k] + coefficient[i][j + 6][k])
						+ BSPLINE_PREFILTER[7] * (coefficient[i][j - 7][k] + coefficient[i][j + 7][k]);
				}
				for (int j = 0; j < 7; j++)
				{
					conv_buffer[i][j][k] = BSPLINE_PREFILTER[0] * coefficient[i][j][k]
						+ BSPLINE_PREFILTER[1] * (coefficient[i][getHigh(j - 1, 0)][k] + coefficient[i][j + 1][k])
						+ BSPLINE_PREFILTER[2] * (coefficient[i][getHigh(j - 2, 0)][k] + coefficient[i][j + 2][k])
						+ BSPLINE_PREFILTER[3] * (coefficient[i][getHigh(j - 3, 0)][k] + coefficient[i][j + 3][k])
						+ BSPLINE_PREFILTER[4] * (coefficient[i][getHigh(j - 4, 0)][k] + coefficient[i][j + 4][k])
						+ BSPLINE_PREFILTER[5] * (coefficient[i][getHigh(j - 5, 0)][k] + coefficient[i][j + 5][k])
						+ BSPLINE_PREFILTER[6] * (coefficient[i][getHigh(j - 6, 0)][k] + coefficient[i][j + 6][k])
						+ BSPLINE_PREFILTER[7] * (coefficient[i][getHigh(j - 7, 0)][k] + coefficient[i][j + 7][k]);
				}
				for (int j = dim_y - 7; j < dim_y; j++)
				{
					conv_buffer[i][j][k] = BSPLINE_PREFILTER[0] * coefficient[i][j][k]
						+ BSPLINE_PREFILTER[1] * (coefficient[i][j - 1][k] + coefficient[i][getLow(j + 1, dim_y - 1)][k])
						+ BSPLINE_PREFILTER[2] * (coefficient[i][j - 2][k] + coefficient[i][getLow(j + 2, dim_y - 1)][k])
						+ BSPLINE_PREFILTER[3] * (coefficient[i][j - 3][k] + coefficient[i][getLow(j + 3, dim_y - 1)][k])
						+ BSPLINE_PREFILTER[4] * (coefficient[i][j - 4][k] + coefficient[i][getLow(j + 4, dim_y - 1)][k])
						+ BSPLINE_PREFILTER[5] * (coefficient[i][j - 5][k] + coefficient[i][getLow(j + 5, dim_y - 1)][k])
						+ BSPLINE_PREFILTER[6] * (coefficient[i][j - 6][k] + coefficient[i][getLow(j + 6, dim_y - 1)][k])
						+ BSPLINE_PREFILTER[7] * (coefficient[i][j - 7][k] + coefficient[i][getLow(j + 7, dim_y - 1)][k]);
				}
			}
		}


		//convolution along z-axis
#pragma omp parallel for
		for (int j = 0; j < dim_y; j++)
		{
			for (int k = 0; k < dim_x; k++)
			{
				for (int i = 7; i < dim_z - 7; i++)
				{
					coefficient[i][j][k] = BSPLINE_PREFILTER[0] * conv_buffer[i][j][k]
						+ BSPLINE_PREFILTER[1] * (conv_buffer[i - 1][j][k] + conv_buffer[i + 1][j][k])
						+ BSPLINE_PREFILTER[2] * (conv_buffer[i - 2][j][k] + conv_buffer[i + 2][j][k])
						+ BSPLINE_PREFILTER[3] * (conv_buffer[i - 3][j][k] + conv_buffer[i + 3][j][k])
						+ BSPLINE_PREFILTER[4] * (conv_buffer[i - 4][j][k] + conv_buffer[i + 4][j][k])
						+ BSPLINE_PREFILTER[5] * (conv_buffer[i - 5][j][k] + conv_buffer[i + 5][j][k])
						+ BSPLINE_PREFILTER[6] * (conv_buffer[i - 6][j][k] + conv_buffer[i + 6][j][k])
						+ BSPLINE_PREFILTER[7] * (conv_buffer[i - 7][j][k] + conv_buffer[i + 7][j][k]);
				}
				for (int i = 0; i < 7; i++)
				{
					coefficient[i][j][k] = BSPLINE_PREFILTER[0] * conv_buffer[i][j][k] +
						+BSPLINE_PREFILTER[1] * (conv_buffer[getHigh(i - 1, 0)][j][k] + conv_buffer[i + 1][j][k])
						+ BSPLINE_PREFILTER[2] * (conv_buffer[getHigh(i - 2, 0)][j][k] + conv_buffer[i + 2][j][k])
						+ BSPLINE_PREFILTER[3] * (conv_buffer[getHigh(i - 3, 0)][j][k] + conv_buffer[i + 3][j][k])
						+ BSPLINE_PREFILTER[4] * (conv_buffer[getHigh(i - 4, 0)][j][k] + conv_buffer[i + 4][j][k])
						+ BSPLINE_PREFILTER[5] * (conv_buffer[getHigh(i - 5, 0)][j][k] + conv_buffer[i + 5][j][k])
						+ BSPLINE_PREFILTER[6] * (conv_buffer[getHigh(i - 6, 0)][j][k] + conv_buffer[i + 6][j][k])
						+ BSPLINE_PREFILTER[7] * (conv_buffer[getHigh(i - 7, 0)][j][k] + conv_buffer[i + 7][j][k]);
				}
				for (int i = dim_z - 7; i < dim_z; i++)
				{
					coefficient[i][j][k] = BSPLINE_PREFILTER[0] * conv_buffer[i][j][k]
						+ BSPLINE_PREFILTER[1] * (conv_buffer[i - 1][j][k] + conv_buffer[getLow(i + 1, dim_z - 1)][j][k])
						+ BSPLINE_PREFILTER[2] * (conv_buffer[i - 2][j][k] + conv_buffer[getLow(i + 2, dim_z - 1)][j][k])
						+ BSPLINE_PREFILTER[3] * (conv_buffer[i - 3][j][k] + conv_buffer[getLow(i + 3, dim_z - 1)][j][k])
						+ BSPLINE_PREFILTER[4] * (conv_buffer[i - 4][j][k] + conv_buffer[getLow(i + 4, dim_z - 1)][j][k])
						+ BSPLINE_PREFILTER[5] * (conv_buffer[i - 5][j][k] + conv_buffer[getLow(i + 5, dim_z - 1)][j][k])
						+ BSPLINE_PREFILTER[6] * (conv_buffer[i - 6][j][k] + conv_buffer[getLow(i + 6, dim_z - 1)][j][k])
						+ BSPLINE_PREFILTER[7] * (conv_buffer[i - 7][j][k] + conv_buffer[getLow(i + 7, dim_z - 1)][j][k]);
				}
			}
		}

		delete3D(conv_buffer);
	}

	float TricubicBspline::compute(Point3D& location)
	{
		float value = 0.f;
		if (location.x < 1 || location.y < 1 || location.z < 1
			|| location.x >= dim_x - 2 || location.y >= dim_y - 2 || location.z >= dim_z - 2
			|| std::isnan(location.x) || std::isnan(location.y) || std::isnan(location.z))
		{
			value = -1.f;
		}
		else
		{
			int x_integral = floor(location.x);
			int y_integral = floor(location.y);
			int z_integral = floor(location.z);

			float x_decimal = location.x - x_integral;
			float y_decimal = location.y - y_integral;
			float z_decimal = location.z - z_integral;

			float basis_x[4], basis_y[4], basis_z[4];
			float sum_x[4], sum_y[4];

			basis_x[0] = basis0(x_decimal);
			basis_x[1] = basis1(x_decimal);
			basis_x[2] = basis2(x_decimal);
			basis_x[3] = basis3(x_decimal);

			basis_y[0] = basis0(y_decimal);
			basis_y[1] = basis1(y_decimal);
			basis_y[2] = basis2(y_decimal);
			basis_y[3] = basis3(y_decimal);

			basis_z[0] = basis0(z_decimal);
			basis_z[1] = basis1(z_decimal);
			basis_z[2] = basis2(z_decimal);
			basis_z[3] = basis3(z_decimal);

			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					sum_x[j] = basis_x[0] * coefficient[z_integral + i - 1][y_integral + j - 1][x_integral - 1]
						+ basis_x[1] * coefficient[z_integral + i - 1][y_integral + j - 1][x_integral]
						+ basis_x[2] * coefficient[z_integral + i - 1][y_integral + j - 1][x_integral + 1]
						+ basis_x[3] * coefficient[z_integral + i - 1][y_integral + j - 1][x_integral + 2];
				}
				sum_y[i] = basis_y[0] * sum_x[0] + basis_y[1] * sum_x[1] + basis_y[2] * sum_x[2] + basis_y[3] * sum_x[3];
			}
			value = basis_z[0] * sum_y[0] + basis_z[1] * sum_y[1] + basis_z[2] * sum_y[2] + basis_z[3] * sum_y[3];
		}

		return value;
	}

}//namespace opencorr
