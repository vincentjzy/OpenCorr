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

#include "oc_sift.h"

namespace opencorr
{
	//SIFT 2D
	SIFT2D::SIFT2D()
	{
		sift_config.n_features = 0;
		sift_config.n_octave_layers = 3;
		sift_config.contrast_threshold = 0.04f; //default configuration in OpenCV
		sift_config.edge_threshold = 10.f;
		sift_config.sigma = 1.6f;
		matching_ratio = 0.8f;
	}

	SIFT2D::~SIFT2D() {}

	Sift2dConfig SIFT2D::getSiftConfig() const
	{
		return sift_config;
	}

	float SIFT2D::getMatchingRatio() const
	{
		return matching_ratio;
	}

	void SIFT2D::setSiftConfig(Sift2dConfig sift_config)
	{
		this->sift_config = sift_config;
	}

	void SIFT2D::setMatching(float matching_ratio)
	{
		this->matching_ratio = matching_ratio;
	}

	void SIFT2D::prepare()
	{
		ref_mat = &ref_img->cv_mat;
		tar_mat = &tar_img->cv_mat;
	}

	void SIFT2D::compute()
	{
		//initialization, refer to opencv document for details
		std::vector<cv::KeyPoint> ref_kp;
		cv::Mat ref_descriptor;
		cv::Ptr<cv::Feature2D> ref_sift = cv::SIFT::create(
			sift_config.n_features,
			sift_config.n_octave_layers,
			sift_config.contrast_threshold,
			sift_config.edge_threshold,
			sift_config.sigma);

		std::vector<cv::KeyPoint> tar_kp;
		cv::Mat tar_descriptor;
		cv::Ptr<cv::Feature2D> tar_sift = cv::SIFT::create(
			sift_config.n_features,
			sift_config.n_octave_layers,
			sift_config.contrast_threshold,
			sift_config.edge_threshold,
			sift_config.sigma);

		//extract features and construct their descriptor
#pragma omp parallel sections
		{
#pragma omp section
			{
				ref_sift->detect(*ref_mat, ref_kp);
				ref_sift->compute(*ref_mat, ref_kp, ref_descriptor);
			}
#pragma omp section
			{
				tar_sift->detect(*tar_mat, tar_kp);
				tar_sift->compute(*tar_mat, tar_kp, tar_descriptor);
			}
		}

		//match the keypoints in the reference image with those in the target image
		cv::FlannBasedMatcher matcher;
		std::vector<std::vector<cv::DMatch>> matches;
		matcher.knnMatch(ref_descriptor, tar_descriptor, matches, 2);

		//check if the matching ratio is satisfied, then assign the matched keypoints to the queues
		int matches_size = (int)matches.size();
#pragma omp parallel sections
		{
#pragma omp section
			{
				for (int i = 0; i < matches_size; ++i)
				{
					if (matches[i][0].distance < matching_ratio * matches[i][1].distance)
					{
						Point2D ref_candidate(ref_kp[i].pt.x, ref_kp[i].pt.y);
						ref_matched_kp.push_back(ref_candidate);
					}
				}
			}
#pragma omp section
			{
				for (int j = 0; j < matches_size; ++j)
				{
					if (matches[j][0].distance < matching_ratio * matches[j][1].distance)
					{
						Point2D tar_candidate(tar_kp[matches[j][0].trainIdx].pt.x, tar_kp[matches[j][0].trainIdx].pt.y);
						tar_matched_kp.push_back(tar_candidate);
					}
				}
			}
		}
	}

	void SIFT2D::clear()
	{
		std::vector<Point2D>().swap(ref_matched_kp);
		std::vector<Point2D>().swap(tar_matched_kp);
	}



	SIFT3D::SIFT3D()
	{
		sift_config.n_octave_layers = 3;
		sift_config.min_dimension = 8;
		sift_config.alpha = 0.1f;
		sift_config.beta = 0.9f;
		sift_config.gamma = 0.4f;
		sift_config.sigma_source = 1.15f;
		sift_config.sigma_base = 1.6f;
		sift_config.gradient_threshold = 0.0000000001f;
		sift_config.truncate_threshold = 0.2f * 128 / 768;

		matching_ratio = 0.85f;

		physical_unit[0] = 1.f;
		physical_unit[1] = 1.f;
		physical_unit[2] = 1.f;
	}

	SIFT3D::~SIFT3D() {}

	Sift3dConfig SIFT3D::getSiftConfig() const
	{
		return sift_config;
	}

	float SIFT3D::getPhysicalUnit(int dim) const
	{
		float dim_unit = 0.f;
		switch (dim)
		{
		case 0:
			dim_unit = physical_unit[0];
			break;
		case 1:
			dim_unit = physical_unit[1];
			break;
		case 2:
			dim_unit = physical_unit[2];
			break;
		}

		return dim_unit;
	}

	float SIFT3D::getMatchingRatio() const
	{
		return matching_ratio;
	}

	void SIFT3D::setSiftConfig(Sift3dConfig sift_config)
	{
		this->sift_config = sift_config;
	}

	void SIFT3D::setPhysicalUnit(float unit_x, float unit_y, float unit_z)
	{
		physical_unit[0] = unit_x;
		physical_unit[1] = unit_y;
		physical_unit[2] = unit_z;
	}

	void SIFT3D::setMatchingRatio(float matching_ratio)
	{
		this->matching_ratio = matching_ratio;
	}

	void SIFT3D::prepare()
	{
		//initialize icosahedron
		icosahedron[0] = TriangleTile(Point3D(0.000000f, -0.525731f, 0.850651f), 1, Point3D(0.000000f, 0.525731f, 0.850651f), 0, Point3D(0.850651f, 0.000000f, 0.525731f), 8);
		icosahedron[1] = TriangleTile(Point3D(0.850651f, 0.000000f, 0.525731f), 8, Point3D(0.000000f, 0.525731f, 0.850651f), 0, Point3D(0.525731f, 0.850651f, 0.000000f), 4);
		icosahedron[2] = TriangleTile(Point3D(0.525731f, 0.850651f, 0.000000f), 4, Point3D(0.000000f, 0.525731f, 0.850651f), 0, Point3D(-0.525731f, 0.850651f, 0.000000f), 5);
		icosahedron[3] = TriangleTile(Point3D(-0.525731f, 0.850651f, 0.000000f), 5, Point3D(0.000000f, 0.525731f, 0.850651f), 0, Point3D(-0.850651f, 0.000000f, 0.525731f), 9);
		icosahedron[4] = TriangleTile(Point3D(-0.850651f, 0.000000f, 0.525731f), 9, Point3D(0.000000f, 0.525731f, 0.850651f), 0, Point3D(0.000000f, -0.525731f, 0.850651f), 1);
		icosahedron[5] = TriangleTile(Point3D(0.525731f, -0.850651f, 0.000000f), 6, Point3D(0.000000f, -0.525731f, 0.850651f), 1, Point3D(0.850651f, 0.000000f, 0.525731f), 8);
		icosahedron[6] = TriangleTile(Point3D(0.525731f, -0.850651f, 0.000000f), 6, Point3D(0.850651f, 0.000000f, 0.525731f), 8, Point3D(0.850651f, 0.000000f, -0.525731f), 10);
		icosahedron[7] = TriangleTile(Point3D(0.850651f, 0.000000f, -0.525731f), 10, Point3D(0.850651f, 0.000000f, 0.525731f), 8, Point3D(0.525731f, 0.850651f, 0.000000f), 4);
		icosahedron[8] = TriangleTile(Point3D(0.850651f, 0.000000f, -0.525731f), 10, Point3D(0.525731f, 0.850651f, 0.000000f), 4, Point3D(0.000000f, 0.525731f, -0.850651f), 2);
		icosahedron[9] = TriangleTile(Point3D(0.000000f, 0.525731f, -0.850651f), 2, Point3D(0.525731f, 0.850651f, 0.000000f), 4, Point3D(-0.525731f, 0.850651f, 0.000000f), 5);
		icosahedron[10] = TriangleTile(Point3D(0.000000f, 0.525731f, -0.850651f), 2, Point3D(-0.525731f, 0.850651f, 0.000000f), 5, Point3D(-0.850651f, 0.000000f, -0.525731f), 11);
		icosahedron[11] = TriangleTile(Point3D(-0.850651f, 0.000000f, -0.525731f), 11, Point3D(-0.525731f, 0.850651f, 0.000000f), 5, Point3D(-0.850651f, 0.000000f, 0.525731f), 9);
		icosahedron[12] = TriangleTile(Point3D(-0.850651f, 0.000000f, -0.525731f), 11, Point3D(-0.850651f, 0.000000f, 0.525731f), 9, Point3D(-0.525731f, -0.850651f, 0.000000f), 7);
		icosahedron[13] = TriangleTile(Point3D(-0.525731f, -0.850651f, 0.000000f), 7, Point3D(-0.850651f, 0.000000f, 0.525731f), 9, Point3D(0.000000f, -0.525731f, 0.850651f), 1);
		icosahedron[14] = TriangleTile(Point3D(-0.525731f, -0.850651f, 0.000000f), 7, Point3D(0.000000f, -0.525731f, 0.850651f), 1, Point3D(0.525731f, -0.850651f, 0.000000f), 6);
		icosahedron[15] = TriangleTile(Point3D(0.525731f, -0.850651f, 0.000000f), 6, Point3D(0.000000f, -0.525731f, -0.850651f), 3, Point3D(-0.525731f, -0.850651f, 0.000000f), 7);
		icosahedron[16] = TriangleTile(Point3D(-0.525731f, -0.850651f, 0.000000f), 7, Point3D(0.000000f, -0.525731f, -0.850651f), 3, Point3D(-0.850651f, 0.000000f, -0.525731f), 11);
		icosahedron[17] = TriangleTile(Point3D(-0.850651f, 0.000000f, -0.525731f), 11, Point3D(0.000000f, -0.525731f, -0.850651f), 3, Point3D(0.000000f, 0.525731f, -0.850651f), 2);
		icosahedron[18] = TriangleTile(Point3D(0.000000f, 0.525731f, -0.850651f), 2, Point3D(0.000000f, -0.525731f, -0.850651f), 3, Point3D(0.850651f, 0.000000f, -0.525731f), 10);
		icosahedron[19] = TriangleTile(Point3D(0.850651f, 0.000000f, -0.525731f), 10, Point3D(0.000000f, -0.525731f, -0.850651f), 3, Point3D(0.525731f, -0.850651f, 0.000000f), 6);
	}

	void SIFT3D::compute()
	{
		//initialization
		std::vector<Layer3D> gaussian_pyramid, dog_pyramid;
		std::vector<Keypoint3D> ref_kp, tar_kp;
		int ref_amount, tar_amount; //amount of extracted kp

		//CAUTION: omp parallel for has been employed in the following functions,
		//no need to run these functions in parallel using omp parallel sections


		//extract features from the reference image
		createGaussianPyramid(ref_img, gaussian_pyramid);
		createDogPyramid(gaussian_pyramid, dog_pyramid);
		detectExtrema(dog_pyramid, ref_kp);

		assignOrientation(ref_kp, gaussian_pyramid);

		//construct descriptors
		ref_amount = (int)ref_kp.size();
		float** ref_descriptor = new2D(ref_amount, 768);
		constructDescriptor(ref_kp, gaussian_pyramid, ref_descriptor);

		std::cout << ref_kp.size() << " features are extracted from the reference image." << std::endl;

		//clear the data in pyramids
		clearPyramid(gaussian_pyramid);
		clearPyramid(dog_pyramid);

		//extract features from the target image
		createGaussianPyramid(tar_img, gaussian_pyramid);
		createDogPyramid(gaussian_pyramid, dog_pyramid);
		detectExtrema(dog_pyramid, tar_kp);

		assignOrientation(tar_kp, gaussian_pyramid);

		//construct descriptors
		tar_amount = (int)tar_kp.size();
		float** tar_descriptor = new2D(tar_amount, 768);
		constructDescriptor(tar_kp, gaussian_pyramid, tar_descriptor);

		std::cout << tar_kp.size() << " features are extracted from the target image." << std::endl;

		//clear pyramids and release memory
		clearPyramid(gaussian_pyramid);
		clearPyramid(dog_pyramid);

		//monodirectional matching, but many-to-one correspondences are eliminated through reverse matching
		monodirectionalMatch(ref_kp, ref_descriptor, tar_kp, tar_descriptor, ref_matched_kp, tar_matched_kp);

		//rigorous birectional matching, less matched keypoints, more computation time
		//bidirectionalMatch(ref_kp, ref_descriptor, tar_kp, tar_descriptor, ref_matched_kp, tar_matched_kp);

		//release memory
		delete2D(ref_descriptor);
		delete2D(tar_descriptor);
	}

	void SIFT3D::clear()
	{
		std::vector<Point3D>().swap(ref_matched_kp);
		std::vector<Point3D>().swap(tar_matched_kp);
	}

	void SIFT3D::initializeIcosahedron(TriangleTile* icosahedron)
	{
		//vertex triplets forming the triangles of an icosahedron
		int triangle_triplets[] =
		{
			1, 0, 8,
			8, 0, 4,
			4, 0, 5,
			5, 0, 9,
			9, 0, 1,
			6, 1, 8,
			6, 8, 10,
			10, 8, 4,
			10, 4, 2,
			2, 4, 5,
			2, 5, 11,
			11, 5, 9,
			11, 9, 7,
			7, 9, 1,
			7, 1, 6,
			6, 3, 7,
			7, 3, 11,
			11, 3, 2,
			2, 3, 10,
			10, 3, 6
		};

		//vertices of a icosahedron
		float triangle_vertices[] =
		{
			0.0000000f, -0.5257311f, 0.8506508f, 0.0000000f, 0.5257311f, 0.8506508f, 0.8506508f, 0.0000000f, 0.5257311f,
			0.8506508f, 0.0000000f, 0.5257311f, 0.0000000f, 0.5257311f, 0.8506508f, 0.5257311f, 0.8506508f, 0.0000000f,
			0.5257311f, 0.8506508f, 0.0000000f, 0.0000000f, 0.5257311f, 0.8506508f, -0.5257311f, 0.8506508f, 0.0000000f,
			-0.5257311f, 0.8506508f, 0.0000000f, 0.0000000f, 0.5257311f, 0.8506508f, -0.8506508f, 0.0000000f, 0.5257311f,
			-0.8506508f, 0.0000000f, 0.5257311f, 0.0000000f, 0.5257311f, 0.8506508f, 0.0000000f, -0.5257311f, 0.8506508f,
			0.5257311f, -0.8506508f, 0.0000000f, 0.0000000f, -0.5257311f, 0.8506508f, 0.8506508f, 0.0000000f, 0.5257311f,
			0.5257311f, -0.8506508f, 0.0000000f, 0.8506508f, 0.0000000f, 0.5257311f, 0.8506508f, 0.0000000f, -0.5257311f,
			0.8506508f, 0.0000000f, -0.5257311f, 0.8506508f, 0.0000000f, 0.5257311f, 0.5257311f, 0.8506508f, 0.0000000f,
			0.8506508f, 0.0000000f, -0.5257311f, 0.5257311f, 0.8506508f, 0.0000000f, 0.0000000f, 0.5257311f, -0.8506508f,
			0.0000000f, 0.5257311f, -0.8506508f, 0.5257311f, 0.8506508f, 0.0000000f, -0.5257311f, 0.8506508f, 0.0000000f,
			0.0000000f, 0.5257311f, -0.8506508f, -0.5257311f, 0.8506508f, 0.0000000f, -0.8506508f, 0.0000000f, -0.5257311f,
			-0.8506508f, 0.0000000f, -0.5257311f, -0.5257311f, 0.8506508f, 0.0000000f, -0.8506508f, 0.0000000f, 0.5257311f,
			-0.8506508f, 0.0000000f, -0.5257311f, -0.8506508f, 0.0000000f, 0.5257311f, -0.5257311f, -0.8506508f, 0.0000000f,
			-0.5257311f, -0.8506508f, 0.0000000f, -0.8506508f, 0.0000000f, 0.5257311f, 0.0000000f, -0.5257311f, 0.8506508f,
			-0.5257311f, -0.8506508f, 0.0000000f, 0.0000000f, -0.5257311f, 0.8506508f, 0.5257311f, -0.8506508f, 0.0000000f,
			0.5257311f, -0.8506508f, 0.0000000f, 0.0000000f, -0.5257311f, -0.8506508f, -0.5257311f, -0.8506508f, 0.0000000f,
			-0.5257311f, -0.8506508f, 0.0000000f, 0.0000000f, -0.5257311f, -0.8506508f, -0.8506508f, 0.0000000f, -0.5257311f,
			-0.8506508f, 0.0000000f, -0.5257311f, 0.0000000f, -0.5257311f, -0.8506508f, 0.0000000f, 0.5257311f, -0.8506508f,
			0.0000000f, 0.5257311f, -0.8506508f, 0.0000000f, -0.5257311f, -0.8506508f, 0.8506508f, 0.0000000f, -0.5257311f,
			0.8506508f, 0.0000000f, -0.5257311f, 0.0000000f, -0.5257311f, -0.8506508f, 0.5257311f, -0.8506508f, 0.0000000f
		};

		for (int i = 0; i < 20; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				icosahedron[i].vertex_idx[j] = triangle_triplets[i * 3 + j];
				icosahedron[i].vertices[j].x = triangle_vertices[(i * 3 + j) * 3 + 0];
				icosahedron[i].vertices[j].y = triangle_vertices[(i * 3 + j) * 3 + 1];
				icosahedron[i].vertices[j].z = triangle_vertices[(i * 3 + j) * 3 + 2];
			}
		}
	}

	void SIFT3D::gaussianBlur(float*** src_img, float*** dst_img, int* dim_xyz, float* unit_xyz, float sigma)
	{
		//determine the dimension with maximum physical unit
		float unit_max = unit_xyz[0] > unit_xyz[1] ? unit_xyz[0] : unit_xyz[1];
		unit_max = unit_max > unit_xyz[2] ? unit_max : unit_xyz[2];

		//size of kernel matrix = (6 * sigma + 1) * (6 * sigma + 1) * (6 * sigma + 1)
		int kernel_radius;
		if (sigma > 0)
		{
			kernel_radius = ceil(3.f * sigma) > 1 ? ceil(3.f * sigma) : 1;
		}
		else
		{
			sigma = 0.f;
			kernel_radius = 1;
		}

		//create the convolution kernel for x-axis
		int radius_x = kernel_radius * floor(unit_max / unit_xyz[0] + 0.5f);
		float* kernel_x = new float[radius_x + 1];
		kernel_x[0] = 1.f; //G(0,sigma) = exp(-0.5f * 0 * 0) = 1
		for (int i = 1; i <= radius_x; i++)
		{
			float x = i / (sigma + FLT_EPSILON);
			kernel_x[i] = exp(-0.5f * x * x);
			kernel_x[0] += (kernel_x[i] * 2.f);
		}

		//normalize the kernel
		kernel_x[0] = 1.f / kernel_x[0];
		for (int i = 1; i <= radius_x; i++)
		{
			kernel_x[i] *= kernel_x[0];
		}

		//create an intermediate matrix for 3D convolution
		float*** conv_buffer = new3D(dim_xyz[2], dim_xyz[1], dim_xyz[0]);

		//convolution along x-axis
#pragma omp parallel for
		for (int i = 0; i < dim_xyz[2]; i++)
		{
			for (int j = 0; j < dim_xyz[1]; j++)
			{
				for (int k = radius_x; k < dim_xyz[0] - radius_x; k++)
				{
					dst_img[i][j][k] = kernel_x[0] * src_img[i][j][k];
					for (int r = 1; r <= radius_x; r++)
					{
						dst_img[i][j][k] += kernel_x[r] * (src_img[i][j][k - r] + src_img[i][j][k + r]);
					}
				}
				//proess the points close to the boundary of image using mirror extension
				for (int k = 0; k < radius_x; k++)
				{
					dst_img[i][j][k] = kernel_x[0] * src_img[i][j][k];
					for (int r = 1; r <= radius_x; r++)
					{
						dst_img[i][j][k] += kernel_x[r] * (src_img[i][j][mirrorLow(k - r, 0)] + src_img[i][j][mirrorHigh(k + r, dim_xyz[0] - 1)]);
					}
				}
				for (int k = dim_xyz[0] - radius_x; k < dim_xyz[0]; k++)
				{
					dst_img[i][j][k] = kernel_x[0] * src_img[i][j][k];
					for (int r = 1; r <= radius_x; r++)
					{
						dst_img[i][j][k] += kernel_x[r] * (src_img[i][j][mirrorLow(k - r, 0)] + src_img[i][j][mirrorHigh(k + r, dim_xyz[0] - 1)]);
					}
				}
			}
		}

		//create the convolution kernel for y-axis
		int radius_y = kernel_radius * floor(unit_max / unit_xyz[1] + 0.5);
		float* kernel_y = new float[radius_y + 1];
		kernel_y[0] = 1.f; //G(0,sigma) = exp(-0.5f * 0 * 0) = 1
		for (int i = 1; i <= radius_y; i++)
		{
			float y = i / (sigma + FLT_EPSILON);
			kernel_y[i] = exp(-0.5f * y * y);
			kernel_y[0] += (kernel_y[i] * 2.f);
		}

		//normalize the kernel
		kernel_y[0] = 1.f / kernel_y[0];
		for (int i = 1; i <= radius_y; i++)
		{
			kernel_y[i] *= kernel_y[0];
		}

		//convolution along y-axis
#pragma omp parallel for
		for (int k = 0; k < dim_xyz[0]; k++)
		{
			for (int i = 0; i < dim_xyz[2]; i++)
			{
				for (int j = radius_y; j < dim_xyz[1] - radius_y; j++)
				{
					conv_buffer[i][j][k] = kernel_y[0] * dst_img[i][j][k];
					for (int r = 1; r <= radius_y; r++)
					{
						conv_buffer[i][j][k] += kernel_y[r] * (dst_img[i][j - r][k] + dst_img[i][j + r][k]);
					}
				}
				//proess the points close to the boundary of image using mirror extension
				for (int j = 0; j < radius_y; j++)
				{
					conv_buffer[i][j][k] = kernel_y[0] * dst_img[i][j][k];
					for (int r = 1; r <= radius_y; r++)
					{
						conv_buffer[i][j][k] += kernel_y[r] * (dst_img[i][mirrorLow(j - r, 0)][k] + dst_img[i][mirrorHigh(j + r, dim_xyz[1] - 1)][k]);
					}
				}
				for (int j = dim_xyz[1] - radius_y; j < dim_xyz[1]; j++)
				{
					conv_buffer[i][j][k] = kernel_y[0] * dst_img[i][j][k];
					for (int r = 1; r <= radius_y; r++)
					{
						conv_buffer[i][j][k] += kernel_y[r] * (dst_img[i][mirrorLow(j - r, 0)][k] + dst_img[i][mirrorHigh(j + r, dim_xyz[1] - 1)][k]);
					}
				}
			}
		}

		//create the convolution kernel for z-axis
		int radius_z = kernel_radius * floor(unit_max / unit_xyz[2] + 0.5f);
		float* kernel_z = new float[radius_z + 1];
		kernel_z[0] = 1.f; //G(0,sigma) = exp(-0.5f * 0 * 0) = 1
		for (int i = 1; i <= radius_z; i++)
		{
			float z = i / (sigma + FLT_EPSILON);
			kernel_z[i] = exp(-0.5f * z * z);
			kernel_z[0] += (kernel_z[i] * 2.f);
		}

		//normalize the kernel
		kernel_z[0] = 1.f / kernel_z[0];
		for (int i = 1; i <= radius_z; i++)
		{
			kernel_z[i] *= kernel_z[0];
		}

		//convolution along z-axis
#pragma omp parallel for
		for (int j = 0; j < dim_xyz[1]; j++)
		{
			for (int k = 0; k < dim_xyz[0]; k++)
			{
				for (int i = radius_z; i < dim_xyz[2] - radius_z; i++)
				{
					dst_img[i][j][k] = kernel_z[0] * conv_buffer[i][j][k];
					for (int r = 1; r <= radius_z; r++)
					{
						dst_img[i][j][k] += kernel_z[r] * (conv_buffer[i - r][j][k] + conv_buffer[i + r][j][k]);
					}
				}
				//proess the points close to the boundary of image using mirror extension
				for (int i = 0; i < radius_z; i++)
				{
					dst_img[i][j][k] = kernel_z[0] * conv_buffer[i][j][k];
					for (int r = 1; r <= radius_z; r++)
					{
						dst_img[i][j][k] += kernel_z[r] * (conv_buffer[mirrorLow(i - r, 0)][j][k] + conv_buffer[mirrorHigh(i + r, dim_xyz[2] - 1)][j][k]);
					}
				}
				for (int i = dim_xyz[2] - radius_z; i < dim_xyz[2]; i++)
				{
					dst_img[i][j][k] = kernel_z[0] * conv_buffer[i][j][k];
					for (int r = 1; r <= radius_z; r++)
					{
						dst_img[i][j][k] += kernel_z[r] * (conv_buffer[mirrorLow(i - r, 0)][j][k] + conv_buffer[mirrorHigh(i + r, dim_xyz[2] - 1)][j][k]);
					}
				}
			}
		}

		//release memory
		delete3D(conv_buffer);
		delete[] kernel_x;
		delete[] kernel_y;
		delete[] kernel_z;
	}

	void SIFT3D::downSampling(float*** src_img, float*** dst_img, int* dst_dim_xyz)
	{
#pragma omp parallel for
		for (int i = 0; i < dst_dim_xyz[2]; i++)
		{
			for (int j = 0; j < dst_dim_xyz[1]; j++)
			{
				for (int k = 0; k < dst_dim_xyz[0]; k++)
				{
					dst_img[i][j][k] = src_img[i * 2][j * 2][k * 2];
				}
			}
		}
	}

	void SIFT3D::clearPyramid(std::vector<Layer3D>& pyramid)
	{
		int layer_number = (int)pyramid.size();
		for (int i = 0; i < layer_number; i++)
		{
			if (pyramid[i].vol_mat != nullptr)
			{
				delete3D(pyramid[i].vol_mat);
			}

		}

		std::vector<Layer3D>().swap(pyramid);
	}

	int SIFT3D::cartisan2Barycentric(Point3D& cart_coor, Point3D& bary_coor, TriangleTile& triangle)
	{
		Point3D e1, e2, t, p, q;
		float k;
		e1 = triangle.vertices[1] - triangle.vertices[0]; //V2 - V1
		e2 = triangle.vertices[2] - triangle.vertices[0]; //V3 - V1
		t = -1 * triangle.vertices[0];					  //O(0,0,0) - V1

		//p = cart_coor x e2
		p = cart_coor / e2; //cross porduct
		//q = t x e1;
		q = t / e1; //cross porduct

		float det = e1 * p; //dot product
		if (fabs(det) < FLT_EPSILON * 10.f)
		{
			return -1;
		}
		float det_inv = 1.f / det;

		bary_coor.z = det_inv * (cart_coor * q);
		bary_coor.y = det_inv * (p * t);
		bary_coor.x = 1.f - bary_coor.y - bary_coor.z;

		k = det_inv * (q * e2);

		if (k < 0)
		{
			return -1;
		}

		if (bary_coor.x < -FLT_EPSILON * 10.f || bary_coor.y < -FLT_EPSILON * 10.f || bary_coor.z < -FLT_EPSILON * 10.f)
		{
			return -1;
		}

		//verify k * cart_coor = bary_coor.x * V1 + bary_coor.y * V2 + bary_coor.z * V3, this check can be omitted
		Point3D residual = k * cart_coor - bary_coor.x * triangle.vertices[0] - bary_coor.y * triangle.vertices[1] - bary_coor.z * triangle.vertices[2];
		if (residual.vectorNorm() > FLT_EPSILON * 10.f)
		{
			return -1;
		}

		return 1;
	}

	int SIFT3D::bruteforceMatch(std::vector<Keypoint3D>& kp1, float** descriptor1, std::vector<Keypoint3D>& kp2, float** descriptor2, int* matched_idx)
	{
		int kp1_amount = (int)kp1.size();
		int kp2_amount = (int)kp2.size();
		float matching_ratio_square = matching_ratio * matching_ratio;
		int match_counter = 0;

		//match each kp1 with all kp2
#pragma omp parallel for reduction (+:match_counter)
		for (int i = 0; i < kp1_amount; i++)
		{
			//create two arrays for ratio check
			int candidate_idx[2] = { -1,-1 };//[0]: the shortest, [1]: the second shortest,
			float candidate_distance[2] = { FLT_MAX,FLT_MAX }; //[0]: the shortest, [1]: the second shortest

			for (int j = 0; j < kp2_amount; j++)
			{
				//calculate squared Euclidean distance between kp1 and kp2
				float squared_distance = 0;
				for (int k = 0; k < 768; k++)
				{
					float component_difference = descriptor1[i][k] - descriptor2[j][k];
					squared_distance += component_difference * component_difference;
				}

				//store the information of kp with the shortest distance or the second shortest distance
				if (squared_distance < candidate_distance[0])
				{
					candidate_idx[1] = candidate_idx[0];
					candidate_distance[1] = candidate_distance[0];
					candidate_idx[0] = j;
					candidate_distance[0] = squared_distance;
				}
				else if (squared_distance < candidate_distance[1])
				{
					candidate_idx[1] = j;
					candidate_distance[1] = squared_distance;
				}
			}

			//check if the matching ratio is satisfied
			if (candidate_distance[0] < matching_ratio_square * candidate_distance[1])
			{
				matched_idx[i] = candidate_idx[0];
				match_counter++;
			}
		}

		return match_counter;
	}

	void SIFT3D::createGaussianPyramid(Image3D* vol_img, std::vector<Layer3D>& gaussian_pyramid)
	{
		//determine the minimum dimension of the input image
		int dim_min = vol_img->dim_x < vol_img->dim_y ? vol_img->dim_x : vol_img->dim_y;
		dim_min = dim_min < vol_img->dim_z ? dim_min : vol_img->dim_z;

		//set the octave and height of pyramid
		sift_config.n_octave = floor(log2((float)dim_min) - log2((float)sift_config.min_dimension)) + 1;
		sift_config.n_octave = sift_config.n_octave > 0 ? sift_config.n_octave : 1;
		int layer_per_octave = sift_config.n_octave_layers + 3; //no local extrema search in the bottom layer and the top layer in each octave of DoG pyramid
		int layer_number = sift_config.n_octave * layer_per_octave; //overall number of layers in a pyramid
		gaussian_pyramid.resize(layer_number);

		//set the bottom layer
		int x_length = vol_img->dim_x;
		int y_length = vol_img->dim_y;
		int z_length = vol_img->dim_z;
		float x_unit = getPhysicalUnit(0);
		float y_unit = getPhysicalUnit(1);
		float z_unit = getPhysicalUnit(2);
		float kappa = pow(2.f, 1.f / sift_config.n_octave_layers);

		gaussian_pyramid[0].dim_xyz[0] = x_length;
		gaussian_pyramid[0].dim_xyz[1] = y_length;
		gaussian_pyramid[0].dim_xyz[2] = z_length;
		gaussian_pyramid[0].unit_xyz[0] = x_unit;
		gaussian_pyramid[0].unit_xyz[1] = y_unit;
		gaussian_pyramid[0].unit_xyz[2] = z_unit;
		gaussian_pyramid[0].octave = 0;
		gaussian_pyramid[0].scale = 1.f / kappa * sift_config.sigma_base;
		gaussian_pyramid[0].sigma = sqrt(gaussian_pyramid[0].scale * gaussian_pyramid[0].scale - sift_config.sigma_source * sift_config.sigma_source);
		gaussian_pyramid[0].vol_mat = new3D(z_length, y_length, x_length);

		//set the other layers
		for (int i = 1; i < layer_number; i++)
		{
			gaussian_pyramid[i].octave = floor(1.f * i / layer_per_octave); //get current octave
			int layer_in_octave = i % layer_per_octave; //get layer position in current octave
			if (layer_in_octave == 0) //bottom layer of current octave
			{
				x_length /= 2;
				y_length /= 2;
				z_length /= 2;
				x_unit *= 2;
				y_unit *= 2;
				z_unit *= 2;

				gaussian_pyramid[i].scale = gaussian_pyramid[(gaussian_pyramid[i].octave - 1) * layer_per_octave + sift_config.n_octave_layers].scale;
			}
			else
			{
				gaussian_pyramid[i].scale = kappa * gaussian_pyramid[i - 1].scale;
				gaussian_pyramid[i].sigma = sqrt(kappa * kappa - 1.f) * gaussian_pyramid[layer_in_octave - 1].scale;
			}

			gaussian_pyramid[i].dim_xyz[0] = x_length;
			gaussian_pyramid[i].dim_xyz[1] = y_length;
			gaussian_pyramid[i].dim_xyz[2] = z_length;
			gaussian_pyramid[i].unit_xyz[0] = x_unit;
			gaussian_pyramid[i].unit_xyz[1] = y_unit;
			gaussian_pyramid[i].unit_xyz[2] = z_unit;

			gaussian_pyramid[i].vol_mat = new3D(z_length, y_length, x_length);
		}

		//fill each layer with blurred image
		gaussianBlur(vol_img->vol_mat, gaussian_pyramid[0].vol_mat, gaussian_pyramid[0].dim_xyz, gaussian_pyramid[0].unit_xyz, gaussian_pyramid[0].sigma); //bottom layer
		for (int i = 1; i < layer_number; i++)
		{
			if (i % layer_per_octave == 0)
			{
				downSampling(gaussian_pyramid[i - 3].vol_mat, gaussian_pyramid[i].vol_mat, gaussian_pyramid[i].dim_xyz); //bottom layer in current octave
			}
			else
			{
				gaussianBlur(gaussian_pyramid[i - 1].vol_mat, gaussian_pyramid[i].vol_mat, gaussian_pyramid[i].dim_xyz, gaussian_pyramid[i].unit_xyz, gaussian_pyramid[i].sigma);
			}
		}
	}

	void SIFT3D::createDogPyramid(std::vector<Layer3D>& gaussian_pyramid, std::vector<Layer3D>& dog_pyramid)
	{
		int layer_per_octave = sift_config.n_octave_layers + 2; //number of layers in each octave
		dog_pyramid.resize(sift_config.n_octave * layer_per_octave);

#pragma omp parallel for
		for (int m = 0; m < sift_config.n_octave; m++)
		{
			for (int n = 0; n < layer_per_octave; n++)
			{
				int g_idx = m * (sift_config.n_octave_layers + 3) + n;
				int d_idx = m * layer_per_octave + n;
				dog_pyramid[d_idx].dim_xyz[0] = gaussian_pyramid[g_idx].dim_xyz[0];
				dog_pyramid[d_idx].dim_xyz[1] = gaussian_pyramid[g_idx].dim_xyz[1];
				dog_pyramid[d_idx].dim_xyz[2] = gaussian_pyramid[g_idx].dim_xyz[2];
				dog_pyramid[d_idx].unit_xyz[0] = gaussian_pyramid[g_idx].unit_xyz[0];
				dog_pyramid[d_idx].unit_xyz[1] = gaussian_pyramid[g_idx].unit_xyz[1];
				dog_pyramid[d_idx].unit_xyz[2] = gaussian_pyramid[g_idx].unit_xyz[2];
				dog_pyramid[d_idx].octave = m;
				dog_pyramid[d_idx].scale = gaussian_pyramid[g_idx].scale;
				dog_pyramid[d_idx].vol_mat = new3D(dog_pyramid[d_idx].dim_xyz[2], dog_pyramid[d_idx].dim_xyz[1], dog_pyramid[d_idx].dim_xyz[0]);
				dog_pyramid[d_idx].max_abs = -1.f;

				for (int i = 0; i < dog_pyramid[d_idx].dim_xyz[2]; i++)
				{
					for (int j = 0; j < dog_pyramid[d_idx].dim_xyz[1]; j++)
					{
						for (int k = 0; k < dog_pyramid[d_idx].dim_xyz[0]; k++)
						{
							dog_pyramid[d_idx].vol_mat[i][j][k] = gaussian_pyramid[g_idx + 1].vol_mat[i][j][k] - gaussian_pyramid[g_idx].vol_mat[i][j][k];
							float dog_abs = fabs(dog_pyramid[d_idx].vol_mat[i][j][k]);
							dog_pyramid[d_idx].max_abs = dog_pyramid[d_idx].max_abs < dog_abs ? dog_abs : dog_pyramid[d_idx].max_abs;
						}
					}
				}
			}
		}
	}

	void SIFT3D::detectExtrema(std::vector<Layer3D>& dog_pyramid, std::vector<Keypoint3D>& kp_queue)
	{
		for (int m = 0; m < sift_config.n_octave; m++)
		{
			//skip the bottom layer and the top layer
			for (int n = 1; n < (sift_config.n_octave_layers + 1); n++)
			{
				int layer_idx = m * (sift_config.n_octave_layers + 2) + n;
				for (int i = IMG_BORDER; i < dog_pyramid[layer_idx].dim_xyz[2] - IMG_BORDER; i++)
				{
					for (int j = IMG_BORDER; j < dog_pyramid[layer_idx].dim_xyz[1] - IMG_BORDER; j++)
					{
						for (int k = IMG_BORDER; k < dog_pyramid[layer_idx].dim_xyz[0] - IMG_BORDER; k++)
						{
							float dog_value = dog_pyramid[layer_idx].vol_mat[i][j][k];

							//check if the DoG value is large enough
							if (fabs(dog_value) >= sift_config.alpha * dog_pyramid[layer_idx].max_abs)
							{
								//check if the DoG value at current position is greater or less than all the eight neighbors
								if ((dog_value > dog_pyramid[layer_idx].vol_mat[i - 1][j][k]
									&& dog_value > dog_pyramid[layer_idx].vol_mat[i + 1][j][k]
									&& dog_value > dog_pyramid[layer_idx].vol_mat[i][j - 1][k]
									&& dog_value > dog_pyramid[layer_idx].vol_mat[i][j + 1][k]
									&& dog_value > dog_pyramid[layer_idx].vol_mat[i][j][k - 1]
									&& dog_value > dog_pyramid[layer_idx].vol_mat[i][j][k + 1]
									&& dog_value > dog_pyramid[layer_idx - 1].vol_mat[i][j][k]
									&& dog_value > dog_pyramid[layer_idx + 1].vol_mat[i][j][k])
									|| (dog_value < dog_pyramid[layer_idx].vol_mat[i - 1][j][k]
										&& dog_value < dog_pyramid[layer_idx].vol_mat[i + 1][j][k]
										&& dog_value < dog_pyramid[layer_idx].vol_mat[i][j - 1][k]
										&& dog_value < dog_pyramid[layer_idx].vol_mat[i][j + 1][k]
										&& dog_value < dog_pyramid[layer_idx].vol_mat[i][j][k - 1]
										&& dog_value < dog_pyramid[layer_idx].vol_mat[i][j][k + 1]
										&& dog_value < dog_pyramid[layer_idx - 1].vol_mat[i][j][k]
										&& dog_value < dog_pyramid[layer_idx + 1].vol_mat[i][j][k]))
								{
									Keypoint3D keypoint_candidate;
									keypoint_candidate.coor_layer.x = k;
									keypoint_candidate.coor_layer.y = j;
									keypoint_candidate.coor_layer.z = i;
									keypoint_candidate.layer = n; //position of layer in its octave
									keypoint_candidate.octave = m;
									keypoint_candidate.scale = dog_pyramid[layer_idx].scale;
									kp_queue.push_back(keypoint_candidate);
								}
							}
						}
					}
				}
			}
		}
	}

	void SIFT3D::assignOrientation(std::vector<Keypoint3D>& kp_queue, std::vector<Layer3D>& gaussian_pyramid)
	{
		std::vector<Keypoint3D> kp_candidate(kp_queue);
		int candidate_amount = (int)kp_candidate.size();
#pragma omp parallel for
		for (int m = 0; m < candidate_amount; m++)
		{
			int g_idx = kp_candidate[m].layer + kp_candidate[m].octave * (sift_config.n_octave_layers + 3);
			bool available_kp = true;

			float sigma_w = 1.5f * kp_candidate[m].scale;
			float window_radius = 3.f * sigma_w;

			//estimate the boundary of spherical window in image
			int x_min = floor(kp_candidate[m].coor_layer.x - window_radius / gaussian_pyramid[g_idx].unit_xyz[0]);
			x_min = x_min > IMG_BORDER ? x_min : IMG_BORDER;
			int x_max = ceil(kp_candidate[m].coor_layer.x + window_radius / gaussian_pyramid[g_idx].unit_xyz[0]);
			x_max = x_max < gaussian_pyramid[g_idx].dim_xyz[0] - IMG_BORDER ? x_max : gaussian_pyramid[g_idx].dim_xyz[0] - IMG_BORDER;

			int y_min = floor(kp_candidate[m].coor_layer.y - window_radius / gaussian_pyramid[g_idx].unit_xyz[1]);
			y_min = y_min > IMG_BORDER ? y_min : IMG_BORDER;
			int y_max = ceil(kp_candidate[m].coor_layer.y + window_radius * gaussian_pyramid[g_idx].unit_xyz[1]);
			y_max = y_max < gaussian_pyramid[g_idx].dim_xyz[1] - IMG_BORDER ? y_max : gaussian_pyramid[g_idx].dim_xyz[1] - IMG_BORDER;

			int z_min = floor(kp_candidate[m].coor_layer.z - window_radius / gaussian_pyramid[g_idx].unit_xyz[2]);
			z_min = z_min > IMG_BORDER ? z_min : IMG_BORDER;
			int z_max = ceil(kp_candidate[m].coor_layer.z + window_radius / gaussian_pyramid[g_idx].unit_xyz[2]);
			z_max = z_max < gaussian_pyramid[g_idx].dim_xyz[2] - IMG_BORDER ? z_max : gaussian_pyramid[g_idx].dim_xyz[2] - IMG_BORDER;

			//directional derivative d(x)
			float d_mat_x = 0.f;
			float d_mat_y = 0.f;
			float d_mat_z = 0.f;

			//initialize structure tensor
			float structure_tensor[9] = { 0.f };

			//construct the structure tensor
			for (int i = z_min; i < z_max; i++)
			{
				for (int j = y_min; j < y_max; j++)
				{
					for (int k = x_min; k < x_max; k++)
					{
						Point3D img_coor(k, j, i); //location with voxel unit
						Point3D phys_corr; //local coordinate using physical unit
						phys_corr.x = (img_coor.x - kp_candidate[m].coor_layer.x) * gaussian_pyramid[g_idx].unit_xyz[0];
						phys_corr.y = (img_coor.y - kp_candidate[m].coor_layer.y) * gaussian_pyramid[g_idx].unit_xyz[1];
						phys_corr.z = (img_coor.z - kp_candidate[m].coor_layer.z) * gaussian_pyramid[g_idx].unit_xyz[2];

						//check if the position is out of the sphere
						if (phys_corr.vectorNorm() <= window_radius)
						{
							float cur_weight = exp(-0.5f * pow(phys_corr.vectorNorm() / sigma_w, 2.f));

							//calculate the gradient components, with respect to physical coordinates
							float grad_x = 0.5 * (gaussian_pyramid[g_idx].vol_mat[i][j][k + 1] - gaussian_pyramid[g_idx].vol_mat[i][j][k - 1]) / gaussian_pyramid[g_idx].unit_xyz[0];
							float grad_y = 0.5 * (gaussian_pyramid[g_idx].vol_mat[i][j + 1][k] - gaussian_pyramid[g_idx].vol_mat[i][j - 1][k]) / gaussian_pyramid[g_idx].unit_xyz[1];
							float grad_z = 0.5 * (gaussian_pyramid[g_idx].vol_mat[i + 1][j][k] - gaussian_pyramid[g_idx].vol_mat[i - 1][j][k]) / gaussian_pyramid[g_idx].unit_xyz[2];

							//calculate the elements of structure tensor
							structure_tensor[0] += grad_x * grad_x * cur_weight;
							structure_tensor[1] += grad_x * grad_y * cur_weight;
							structure_tensor[2] += grad_x * grad_z * cur_weight;
							structure_tensor[4] += grad_y * grad_y * cur_weight;
							structure_tensor[5] += grad_y * grad_z * cur_weight;
							structure_tensor[8] += grad_z * grad_z * cur_weight;

							//calculate the elements of directional derivative d(x)
							d_mat_x += grad_x * cur_weight;
							d_mat_y += grad_y * cur_weight;
							d_mat_z += grad_z * cur_weight;
						}
					}
				}
			}
			structure_tensor[3] = structure_tensor[1];
			structure_tensor[6] = structure_tensor[2];
			structure_tensor[7] = structure_tensor[5];

			//check if the squared norm of directional derivative d is too small
			if ((d_mat_x * d_mat_x + d_mat_y * d_mat_y + d_mat_z * d_mat_z) < sift_config.gradient_threshold)
			{
				available_kp = false;
			}
			else
			{
				//eigendecomposition of structure tensor, s_mat = Q * A * Q'
				Eigen::Matrix3f s_mat;
				s_mat(0, 0) = structure_tensor[0];
				s_mat(0, 1) = structure_tensor[1];
				s_mat(0, 2) = structure_tensor[2];
				s_mat(1, 0) = structure_tensor[3];
				s_mat(1, 1) = structure_tensor[4];
				s_mat(1, 2) = structure_tensor[5];
				s_mat(2, 0) = structure_tensor[6];
				s_mat(2, 1) = structure_tensor[7];
				s_mat(2, 2) = structure_tensor[8];

				Eigen::EigenSolver<Eigen::Matrix3f> es(s_mat);
				Eigen::Matrix3f s_eigenvalue = es.pseudoEigenvalueMatrix();
				Eigen::Matrix3f s_eigenvector = es.pseudoEigenvectors();

				//sort the eigenvalue and eigenvector in a descending order
				EigenMatrix e_m[3];
				e_m[0].eigen_value = s_eigenvalue(0, 0);
				e_m[1].eigen_value = s_eigenvalue(1, 1);
				e_m[2].eigen_value = s_eigenvalue(2, 2);

				e_m[0].eigen_vector[0] = s_eigenvector(0, 0);
				e_m[0].eigen_vector[1] = s_eigenvector(1, 0);
				e_m[0].eigen_vector[2] = s_eigenvector(2, 0);

				e_m[1].eigen_vector[0] = s_eigenvector(0, 1);
				e_m[1].eigen_vector[1] = s_eigenvector(1, 1);
				e_m[1].eigen_vector[2] = s_eigenvector(2, 1);

				e_m[2].eigen_vector[0] = s_eigenvector(0, 2);
				e_m[2].eigen_vector[1] = s_eigenvector(1, 2);
				e_m[2].eigen_vector[2] = s_eigenvector(2, 2);

				std::sort(std::begin(e_m), std::end(e_m), sortByEigenvalue);

				//check if there are ambiguous eigenvalues or the eigenvalues are too small
				if ((e_m[1].eigen_value / e_m[0].eigen_value) > sift_config.beta
					|| (e_m[2].eigen_value / e_m[1].eigen_value) > sift_config.beta
					|| fabs(e_m[0].eigen_value - e_m[1].eigen_value) < FLT_EPSILON
					|| fabs(e_m[1].eigen_value - e_m[2].eigen_value) < FLT_EPSILON
					|| fabs(e_m[2].eigen_value - e_m[0].eigen_value) < FLT_EPSILON)
				{
					available_kp = false;
				}
				else
				{
					Point3D d_vec(d_mat_x, d_mat_y, d_mat_z);
					float cos_phi = FLT_MAX;
					for (int i = 0; i < 2; i++)
					{
						Point3D q_vec(e_m[i].eigen_vector[0], e_m[i].eigen_vector[1], e_m[i].eigen_vector[2]);

						//dot product of q and d
						float q_d = q_vec * d_vec;

						//keep the smallest cos_phi
						float abs_cos_phi = fabs(q_d / (q_vec.vectorNorm() * d_vec.vectorNorm()));
						cos_phi = cos_phi < abs_cos_phi ? cos_phi : abs_cos_phi;

						//determine the sign of eigenvector
						float sgn = q_d > 0 ? 1.f : -1.f;

						//adjust the sign of eigenvector
						e_m[i].eigen_vector[0] *= sgn;
						e_m[i].eigen_vector[1] *= sgn;
						e_m[i].eigen_vector[2] *= sgn;
					}

					//check if the angle between the two vectors q and d is too small
					if (cos_phi < sift_config.gamma)
					{
						available_kp = false;
					}
					else
					{
						//build rotation matrix
						Point3D r1(e_m[0].eigen_vector[0], e_m[0].eigen_vector[1], e_m[0].eigen_vector[2]);
						Point3D r2(e_m[1].eigen_vector[0], e_m[1].eigen_vector[1], e_m[1].eigen_vector[2]);

						//cross product of r1 and r2
						Point3D rc = r1 / r2;

						//inverse of rotation matrix = tansposition of rotation matrix
						kp_candidate[m].rotation_matrix[0] = r1.x;
						kp_candidate[m].rotation_matrix[3] = r2.x;
						kp_candidate[m].rotation_matrix[6] = rc.x;
						kp_candidate[m].rotation_matrix[1] = r1.y;
						kp_candidate[m].rotation_matrix[4] = r2.y;
						kp_candidate[m].rotation_matrix[7] = rc.y;
						kp_candidate[m].rotation_matrix[2] = r1.z;
						kp_candidate[m].rotation_matrix[5] = r2.z;
						kp_candidate[m].rotation_matrix[8] = rc.z;
					}
				}
			}

			kp_candidate[m].coor_img.x = available_kp ? 1 : -1;
		}

		//fill kp_queue with validated keypoints
		std::vector<Keypoint3D>().swap(kp_queue);
		candidate_amount = (int)kp_candidate.size();
		for (int i = 0; i < candidate_amount; i++)
		{
			if (kp_candidate[i].coor_img.x > 0)
			{
				//set the coordinates of keypoint in the original image
				float scale_factor = pow(2.f, kp_candidate[i].octave);
				kp_candidate[i].coor_img = kp_candidate[i].coor_layer * scale_factor;
				kp_queue.push_back(kp_candidate[i]);
			}
		}
	}

	void SIFT3D::constructDescriptor(std::vector<Keypoint3D>& kp_queue, std::vector<Layer3D>& gaussian_pyramid, float** descriptor)
	{
		int kp_amount = (int)kp_queue.size();
		float sqrt_2 = sqrt(2.f);

#pragma omp parallel for
		for (int m = 0; m < kp_amount; m++)
		{
			int g_idx = kp_queue[m].layer + kp_queue[m].octave * (sift_config.n_octave_layers + 3);

			float sigma = 5.f * sqrt_2 * kp_queue[m].scale;
			float sphere_radius = 2.f * sigma;
			float cube_radius = sphere_radius / sqrt_2; //subregion is an inscribed cube of the sphere

			//estimate the circumscibed cubic boundary of spherical window in image
			int x_min = floor(kp_queue[m].coor_layer.x - sphere_radius / gaussian_pyramid[g_idx].unit_xyz[0]);
			x_min = x_min > IMG_BORDER ? x_min : IMG_BORDER;
			int x_max = ceil(kp_queue[m].coor_layer.x + sphere_radius / gaussian_pyramid[g_idx].unit_xyz[0]);
			x_max = x_max < gaussian_pyramid[g_idx].dim_xyz[0] - IMG_BORDER ? x_max : gaussian_pyramid[g_idx].dim_xyz[0] - IMG_BORDER;

			int y_min = floor(kp_queue[m].coor_layer.y - sphere_radius / gaussian_pyramid[g_idx].unit_xyz[1]);
			y_min = y_min > IMG_BORDER ? y_min : IMG_BORDER;
			int y_max = ceil(kp_queue[m].coor_layer.y + sphere_radius / gaussian_pyramid[g_idx].unit_xyz[1]);
			y_max = y_max < gaussian_pyramid[g_idx].dim_xyz[1] - IMG_BORDER ? y_max : gaussian_pyramid[g_idx].dim_xyz[1] - IMG_BORDER;

			int z_min = floor(kp_queue[m].coor_layer.z - sphere_radius / gaussian_pyramid[g_idx].unit_xyz[2]);
			z_min = z_min > IMG_BORDER ? z_min : IMG_BORDER;
			int z_max = ceil(kp_queue[m].coor_layer.z + sphere_radius / gaussian_pyramid[g_idx].unit_xyz[2]);
			z_max = z_max < gaussian_pyramid[g_idx].dim_xyz[2] - IMG_BORDER ? z_max : gaussian_pyramid[g_idx].dim_xyz[2] - IMG_BORDER;

			//construct the descriptor
			for (int i = z_min; i < z_max; i++)
			{
				for (int j = y_min; j < y_max; j++)
				{
					for (int k = x_min; k < x_max; k++)
					{
						Point3D img_coor(k, j, i); //location with voxel unit

						//get local coordinates in a keypoint-centered coordinate system, using physical unit
						Point3D phys_coor = img_coor - kp_queue[m].coor_layer;
						phys_coor.x *= gaussian_pyramid[g_idx].unit_xyz[0];
						phys_coor.y *= gaussian_pyramid[g_idx].unit_xyz[1];
						phys_coor.z *= gaussian_pyramid[g_idx].unit_xyz[2];

						float cur_distance = phys_coor.vectorNorm(); //distance to the kp

						//check if the location is inside the spherical region
						if (cur_distance > sphere_radius)
						{
							continue;
						}

						//rotate the local coordinate
						Point3D rotated_coor;
						rotated_coor.x = kp_queue[m].rotation_matrix[0] * phys_coor.x + kp_queue[m].rotation_matrix[1] * phys_coor.y + kp_queue[m].rotation_matrix[2] * phys_coor.z;
						rotated_coor.y = kp_queue[m].rotation_matrix[3] * phys_coor.x + kp_queue[m].rotation_matrix[4] * phys_coor.y + kp_queue[m].rotation_matrix[5] * phys_coor.z;
						rotated_coor.z = kp_queue[m].rotation_matrix[6] * phys_coor.x + kp_queue[m].rotation_matrix[7] * phys_coor.y + kp_queue[m].rotation_matrix[8] * phys_coor.z;

						//get normalized coordinate in 4x4x4-cube subregion
						Point3D subregion_coor;
						subregion_coor.x = 2 * (rotated_coor.x + cube_radius) / cube_radius;
						subregion_coor.y = 2 * (rotated_coor.y + cube_radius) / cube_radius;
						subregion_coor.z = 2 * (rotated_coor.z + cube_radius) / cube_radius;

						//let the keypoint is at [1.5, 1.5, 1.5], the diagonal corners of subregion are [-0.5, -0.5, -0.5] and [3.5, 3.5, 3.5]
						subregion_coor.x -= 0.5f;
						subregion_coor.y -= 0.5f;
						subregion_coor.z -= 0.5f;

						//the locations falling in the area between the sphere and its incribed cube are not involved in descriptor construction
						if (subregion_coor.x <= -0.5f || subregion_coor.y <= -0.5f || subregion_coor.z <= -0.5f
							|| subregion_coor.x >= 3.5f || subregion_coor.y >= 3.5f || subregion_coor.z >= 3.5f)
						{
							continue;
						}

						//calculate the gradient, using physical unit
						float cur_weight = exp(-0.5 * pow(cur_distance / sigma, 2));
						Point3D gradient;
						gradient.x = 0.5 * (gaussian_pyramid[g_idx].vol_mat[i][j][k + 1] - gaussian_pyramid[g_idx].vol_mat[i][j][k - 1]) / gaussian_pyramid[g_idx].unit_xyz[0];
						gradient.y = 0.5 * (gaussian_pyramid[g_idx].vol_mat[i][j + 1][k] - gaussian_pyramid[g_idx].vol_mat[i][j - 1][k]) / gaussian_pyramid[g_idx].unit_xyz[1];
						gradient.z = 0.5 * (gaussian_pyramid[g_idx].vol_mat[i + 1][j][k] - gaussian_pyramid[g_idx].vol_mat[i - 1][j][k]) / gaussian_pyramid[g_idx].unit_xyz[2];

						//modify the vector with Gaussian weight
						gradient = gradient * cur_weight;

						//rotate the gradient vector
						Point3D rotated_gradient;
						rotated_gradient.x = kp_queue[m].rotation_matrix[0] * gradient.x + kp_queue[m].rotation_matrix[1] * gradient.y + kp_queue[m].rotation_matrix[2] * gradient.z;
						rotated_gradient.y = kp_queue[m].rotation_matrix[3] * gradient.x + kp_queue[m].rotation_matrix[4] * gradient.y + kp_queue[m].rotation_matrix[5] * gradient.z;
						rotated_gradient.z = kp_queue[m].rotation_matrix[6] * gradient.x + kp_queue[m].rotation_matrix[7] * gradient.y + kp_queue[m].rotation_matrix[8] * gradient.z;

						//check if the vector of rotated gradient is too small
						float gradient_magnitude = rotated_gradient.vectorNorm();
						if (gradient_magnitude * gradient_magnitude < FLT_EPSILON * 10.f)
						{
							continue;
						}

						//search the triangle tile of the icosahedron intersecting with the ray of gradient vector
						int intersect_idx = -1;
						Point3D bary_coor;
						for (int n = 0; n < 20; n++)
						{
							intersect_idx = cartisan2Barycentric(rotated_gradient, bary_coor, icosahedron[n]);

							//check if locate the triangle successfully
							if (intersect_idx > 0)
							{
								intersect_idx = n;
								break;
							}
						}

						//in strange case that no triangle tile is located
						if (intersect_idx < 0)
						{
							continue;
						}

						//get the decimal part of subregion_coor
						Point3D decimal_coor;
						decimal_coor.x = subregion_coor.x - floor(subregion_coor.x);
						decimal_coor.y = subregion_coor.y - floor(subregion_coor.y);
						decimal_coor.z = subregion_coor.z - floor(subregion_coor.z);

						//distribute the gradient value at a point to the histograms of the 8 adjacent cubes in subregion
						for (int dz = 0; dz < 2; dz++)
						{
							for (int dy = 0; dy < 2; dy++)
							{
								for (int dx = 0; dx < 2; dx++)
								{
									int local_x = (int)subregion_coor.x + dx;
									int local_y = (int)subregion_coor.y + dy;
									int local_z = (int)subregion_coor.z + dz;

									//exlcude the voxels close to the boundaries of subregion
									if (local_x < 0 || local_y < 0 || local_z < 0 || local_x >= 4 || local_y >= 4 || local_z >= 4)
									{
										continue;
									}

									//get the index of cube in a subregion
									int cube_idx = local_x + local_y * 4 + local_z * 16;

									//calculate the weight of trilinear interpolation
									float interp_weight = ((dx == 0) ? (1.f - decimal_coor.x) : decimal_coor.x)
										* ((dy == 0) ? (1.f - decimal_coor.y) : decimal_coor.y)
										* ((dz == 0) ? (1.f - decimal_coor.z) : decimal_coor.z);

									//locate the histogram bins
									int offset_v1 = cube_idx * 12 + icosahedron[intersect_idx].vertex_idx[0];
									int offset_v2 = cube_idx * 12 + icosahedron[intersect_idx].vertex_idx[1];
									int offset_v3 = cube_idx * 12 + icosahedron[intersect_idx].vertex_idx[2];

									//accumulate the gradients to the bins
									descriptor[m][offset_v1] += gradient_magnitude * interp_weight * bary_coor.x;
									descriptor[m][offset_v2] += gradient_magnitude * interp_weight * bary_coor.y;
									descriptor[m][offset_v3] += gradient_magnitude * interp_weight * bary_coor.z;
								}
							}
						}
					}
				}
			}

			//normalize the descriptor
			float sq_sum = 0;
			for (int i = 0; i < 768; i++)
			{
				sq_sum += descriptor[m][i] * descriptor[m][i];
			}
			float inv_norm = 1.f / (sqrt(sq_sum) + FLT_EPSILON);
			for (int i = 0; i < 768; i++)
			{
				descriptor[m][i] *= inv_norm;
			}

			//truncate the descriptor with a threshold = 0.2 x 128 / 768
			for (int i = 0; i < 768; i++)
			{
				descriptor[m][i] = (descriptor[m][i] < sift_config.truncate_threshold) ? descriptor[m][i] : sift_config.truncate_threshold;
			}

			//normalize the descriptor again
			sq_sum = 0;
			for (int i = 0; i < 768; i++)
			{
				sq_sum += descriptor[m][i] * descriptor[m][i];
			}
			inv_norm = 1.f / (sqrt(sq_sum) + FLT_EPSILON);
			for (int i = 0; i < 768; i++)
			{
				descriptor[m][i] *= inv_norm;
			}
		}
	}

	void SIFT3D::monodirectionalMatch(std::vector<Keypoint3D>& kp1, float** descriptor1, std::vector<Keypoint3D>& kp2, float** descriptor2, std::vector<Point3D>& matched_kp1, std::vector<Point3D>& matched_kp2)
	{
		int kp1_amount = (int)kp1.size();
		int kp2_amount = (int)kp2.size();
		float matching_ratio_square = matching_ratio * matching_ratio;

		//create a queue to check the pairing betwen ref_key and tar_kp
		KeypointChecker kp_chk;
		kp_chk.ref_idx = -1;
		kp_chk.tar_idx = -1;
		std::vector<KeypointChecker> kp_matches(kp1_amount, kp_chk);

		//match each reference keypoint with target keypoints
#pragma omp parallel for
		for (int i = 0; i < kp1_amount; i++)
		{
			//create two arrays for ratio check
			int candidate_idx[2] = { -1,-1 };//[0]: the shortest, [1]: the second shortest,
			float candidate_distance[2] = { FLT_MAX,FLT_MAX }; //[0]: the shortest, [1]: the second shortest

			for (int j = 0; j < kp2_amount; j++)
			{
				//calculate squared Euclidean distance between kp1 and kp2
				float squared_distance = 0;
				for (int k = 0; k < 768; k++)
				{
					float component_difference = descriptor1[i][k] - descriptor2[j][k];
					squared_distance += component_difference * component_difference;
				}

				//store the information of kp with the shortest distance or the second shortest distance
				if (squared_distance < candidate_distance[0])
				{
					candidate_idx[1] = candidate_idx[0];
					candidate_distance[1] = candidate_distance[0];
					candidate_idx[0] = j;
					candidate_distance[0] = squared_distance;
				}
				else if (squared_distance < candidate_distance[1])
				{
					candidate_idx[1] = j;
					candidate_distance[1] = squared_distance;
				}
			}

			//check if the matching ratio is satisfied
			if (candidate_distance[0] < matching_ratio_square * candidate_distance[1])
			{
				kp_matches[i].ref_idx = i;
				kp_matches[i].tar_idx = candidate_idx[0];
				kp_matches[i].dist = candidate_distance[0];
			}
		}

		//arrange the queue of matched keypoints in descending order of reference keypoint index, keep the ones greater than -1
		std::sort(kp_matches.begin(), kp_matches.end(), sortByRefIdx);

		int matched_amount = 0;
		for (int i = 0; i < kp1_amount; i++)
		{
			if (kp_matches[i].ref_idx == -1)
			{
				matched_amount = i;
				break;
			}
		}

		if (matched_amount > 1)
		{
			//resize the queue of matched keypoints, exclude invalid elements
			kp_matches.resize(matched_amount);

			//rearrange the queue in descending order of target keypoint index
			std::sort(kp_matches.begin(), kp_matches.end(), sortByTarIdx);

			std::vector<int> mto_tar_idx; //index of target keypoint with many-to-one correspondence
			std::vector<int> mto_tar_amount; //amount of target keypoints with many-to-one correspondence

			//check if any tar_kp is matched with multiple ref_kp
			int seg_start = 0;
			int seg_len = 0;
			for (int j = 0; j < matched_amount; j++)
			{
				if (kp_matches[j].tar_idx == kp_matches[j + 1].tar_idx)
				{
					if (j == 0)
					{
						seg_start = j;
						mto_tar_idx.push_back(seg_start);
					}
					else if (kp_matches[j].tar_idx != kp_matches[j - 1].tar_idx)
					{
						seg_start = j;
						mto_tar_idx.push_back(seg_start);
					}
					seg_len++;
				}
				else if (seg_len > 0) {
					mto_tar_amount.push_back(seg_len + 1);
					seg_len = 0;
				}
			}

			int check_length = (int)mto_tar_idx.size();
			if (check_length > 0)
			{
#pragma omp parallel for
				for (int i = 0; i < check_length; i++)
				{
					int candidate_idx[2] = { -1,-1 };//[0]: the shortest, [1]: the second shortest,
					float candidate_distance[2] = { FLT_MAX,FLT_MAX }; //[0]: the shortest, [1]: the second shortest

					for (int j = 0; j < mto_tar_amount[i]; j++)
					{
						//store the information of keypoint it has the shortest distance or the second shortest distance
						int check_idx = mto_tar_idx[i] + j;
						if (kp_matches[check_idx].dist < candidate_distance[0])
						{
							candidate_idx[1] = candidate_idx[0];
							candidate_distance[1] = candidate_distance[0];
							candidate_idx[0] = kp_matches[check_idx].ref_idx;
							candidate_distance[0] = kp_matches[check_idx].dist;
						}
						else if (kp_matches[check_idx].dist < candidate_distance[1])
						{
							candidate_idx[1] = kp_matches[check_idx].ref_idx;
							candidate_distance[1] = kp_matches[check_idx].dist;
						}
						kp_matches[check_idx].ref_idx = -1; //invalidate the inappropriate matched reference keypoints
					}
					//check if the match ratio is satisfied
					if (candidate_distance[0] < matching_ratio_square * candidate_distance[1])
					{
						//validate the first element in the segment
						kp_matches[mto_tar_idx[i]].ref_idx = candidate_idx[0];
					}
				}
			}
		}

		//assign matched kypoints to the queues
#pragma omp parallel sections
		{
#pragma omp section
			{
				for (int i = 0; i < matched_amount; i++)
				{
					if (kp_matches[i].ref_idx > -1)
					{
						Point3D cur_kp = kp1[kp_matches[i].ref_idx].coor_img;
						matched_kp1.push_back(cur_kp);
					}
				}
			}
#pragma omp section
			{
				for (int j = 0; j < matched_amount; j++)
				{
					if (kp_matches[j].ref_idx > -1)
					{
						Point3D cur_kp = kp2[kp_matches[j].tar_idx].coor_img;
						matched_kp2.push_back(cur_kp);
					}
				}
			}
		}
	}

	void SIFT3D::bidirectionalMatch(std::vector<Keypoint3D>& kp1, float** descriptor1, std::vector<Keypoint3D>& kp2, float** descriptor2, std::vector<Point3D>& matched_kp1, std::vector<Point3D>& matched_kp2)
	{
		int kp1_amount = (int)kp1.size();
		int kp2_amount = (int)kp2.size();

		//match the keypoints in the two images, using brute force search, reject the keypoints with eta >= matching_ratio
		int* r2t_idx = new int[kp1_amount]; //index of matched tar_kp
		std::fill(&r2t_idx[0], &r2t_idx[0] + kp1_amount - 1, -1); //initialze the array of index
		int r2t_number = bruteforceMatch(kp1, descriptor1, kp2, descriptor2, r2t_idx); //ref->tar

		int* t2r_idx = new int[kp2_amount]; //index of matched ref_kp
		std::fill(&t2r_idx[0], &t2r_idx[0] + kp2_amount - 1, -1); //initialze the array of index
		int t2r_number = bruteforceMatch(kp2, descriptor2, kp1, descriptor1, t2r_idx); //tar->ref

		//bidirectional check
#pragma omp parallel for
		for (int i = 0; i < kp1_amount; i++)
		{
			if (r2t_idx[i] > -1)
			{
				if (t2r_idx[r2t_idx[i]] != i)
				{
					r2t_idx[i] = -1;
				}
			}
		}

#pragma omp parallel for
		for (int j = 0; j < kp2_amount; j++)
		{
			if (t2r_idx[j] > -1)
			{
				if (r2t_idx[t2r_idx[j]] != j)
				{
					t2r_idx[j] = -1;
				}
			}
		}

		//assign the matched kypoints to the queues
#pragma omp parallel sections
		{
#pragma omp section
			{
				for (int i = 0; i < kp1_amount; i++)
				{
					if (r2t_idx[i] != -1)
					{
						matched_kp1.push_back(kp1[i].coor_img);
					}
				}
			}
#pragma omp section
			{
				for (int j = 0; j < kp1_amount; j++)
				{
					if (r2t_idx[j] != -1)
					{
						matched_kp2.push_back(kp2[r2t_idx[j]].coor_img);
					}
				}
			}
		}

		//release memory
		delete[] r2t_idx;
		delete[] t2r_idx;
	}


	bool sortByEigenvalue(const EigenMatrix& em1, const EigenMatrix& em2)
	{
		return em1.eigen_value > em2.eigen_value;
	}

	bool sortByRefIdx(const KeypointChecker& kc1, const KeypointChecker& kc2)
	{
		return kc1.ref_idx > kc2.ref_idx;
	}

	bool sortByTarIdx(const KeypointChecker& kc1, const KeypointChecker& kc2)
	{
		return kc1.tar_idx > kc2.tar_idx;
	}

	int mirrorLow(int x, int y)
	{
		int output_index = x < y ? (-1 * x) : x;

		return output_index;
	}

	int mirrorHigh(int x, int y)
	{
		int output_index = x > y ? (2 * y - x) : x;

		return output_index;
	}

} //namespace opencorr
