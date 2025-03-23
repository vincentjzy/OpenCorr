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

#ifndef _SIFT_H_
#define _SIFT_H_

#include "oc_feature.h"

#define IMG_BORDER 1 //gap to the boundary of image

namespace opencorr
{
	//the 2D part of module is the implementation of
	//D. Lowe, International Journal of Computer Vision (2004) 60(2): 91-110.
	//https://doi.org/10.1023/B:VISI.0000029664.99615.94

	struct Sift2dConfig //refer to OpenCV document for detailed information
	{
		int n_features; //number of best features to retain
		int n_octave_layers; //number of layers in each octave for feature detection
		float contrast_threshold; //contrast threshold used to filter out weak features in semi-uniform (low-contrast) regions
		float edge_threshold; //threshold used to filter out edge-like features
		float sigma; //sigma of the Gaussian applied to the input image at the octave #0
	};

	class SIFT2D : public Feature2D
	{
	protected:
		cv::Mat* ref_mat = nullptr; //pointer to ref image
		cv::Mat* tar_mat = nullptr; //pointer to tar image

		Sift2dConfig sift_config;
		float matching_ratio; //ratio of the shortest distance to the second shortest distance

	public:
		std::vector<Point2D> ref_matched_kp; //matched keypoints in ref image
		std::vector<Point2D> tar_matched_kp; //matched keypoints in tar image

		SIFT2D();
		~SIFT2D();

		Sift2dConfig getSiftConfig() const;
		float getMatchingRatio() const;
		void setSiftConfig(Sift2dConfig sift_config);
		void setMatching(float matching_ratio);
		void setThreads(int thread_number);

		void prepare();
		void compute();

		void clear(); //clear the queues of matched keypoints
	};


	//the 3D part of module is the implementation of
	//B. Rister et al, IEEE Transactions on Image Processing (2017) 26(10): 4900-4910.
	//https://doi.org/10.1109/TIP.2017.2722689

	struct Sift3dConfig
	{
		int n_octave_layers; //number of layers in each octave for extrema detection
		int n_octave; //number of octave in a pyramid
		int min_dimension; //lower limit of the image dimension for downsampling
		float alpha; //threshold used to filter out the insufficiently distinct keypoints in DoG pyramid
		float beta; //threshold used to filter out the keypoints with ambigous eigenvalues
		float gamma; //threshold used to filter out the keypoints with small angle between the eigenvectors
		float sigma_source; //scale of the Gaussian blur kernel supposed to be included in the input image
		float sigma_base; //base scale of the pyramid
		float gradient_threshold; //threshold of Gaussian weighted gradient
		float truncate_threshold; //threshold used to truncate the descriptor components
	};

	struct Layer3D
	{
		float*** vol_mat;
		int dim_xyz[3]; //{ dim_x, dim_y, dim_z }
		float unit_xyz[3]; //{ unit_x, unit_y, unit_z }
		int octave;
		float max_abs; //maximum absolute DoG value in the layer
		float scale;
		float sigma; //sigma used to blur the current layer based on the previous layer
	};

	struct Keypoint3D
	{
		Point3D coor_layer; //coordinate in current layer
		Point3D coor_img; //coordinate in original image
		int octave, layer;
		float scale;
		float rotation_matrix[9]; //dimension: 3x3, column first
	};

	struct EigenMatrix
	{
		float eigen_value;
		float eigen_vector[3];
	};

	struct TriangleTile
	{
		Point3D vertices[3]; //vertices of triangle
		int vertex_idx[3]; //index of vertex

		TriangleTile() {}
		TriangleTile(Point3D v1, int vi1, Point3D v2, int vi2, Point3D v3, int vi3)
		{
			vertices[0] = v1;
			vertices[1] = v2;
			vertices[2] = v3;
			vertex_idx[0] = vi1;
			vertex_idx[1] = vi2;
			vertex_idx[2] = vi3;
		}
	};

	struct KeypointChecker
	{
		int ref_idx;
		int tar_idx;
		float dist;
	};

	class SIFT3D : public Feature3D
	{
	private:
		TriangleTile icosahedron[20]; //triangles in an icosahedron

	protected:
		Sift3dConfig sift_config;
		float matching_ratio; //ratio of the shortest distance to the second shortest distance
		float physical_unit[3]; //physical unit per image voxel

	public:
		std::vector<Point3D> ref_matched_kp; //matched keypoints in ref image
		std::vector<Point3D> tar_matched_kp; //matched keypoints in tar image

		SIFT3D();
		~SIFT3D();

		Sift3dConfig getSiftConfig() const;
		float getPhysicalUnit(int dim) const; //dim = 0,1,2 means x,y,z respectively
		float getMatchingRatio() const;
		void setSiftConfig(Sift3dConfig sift_config);
		void setPhysicalUnit(float unit_x, float unit_y, float unit_z);
		void setMatchingRatio(float matching_ratio);

		void prepare();
		void compute();

		void clear(); //clear the queues of matched keypoints

		void initializeIcosahedron(TriangleTile* icosahedron);
		void gaussianBlur(float*** src_img, float*** dst_img, int* dim_xyz, float* unit_xyz, float sigma); //dim_xyz[] = { x, y, z }
		void downSampling(float*** src_img, float*** dst_img, int* dst_dim_xyz); //dim_xyz[] = { x, y, z }
		void clearPyramid(std::vector<Layer3D>& pyramid);
		int cartisan2Barycentric(Point3D& cart_coor, Point3D& bary_coor, TriangleTile& triangle); //convert the Cartisian coordinates of intersection point to barycentric coordinate system in regular triangle
		int bruteforceMatch(std::vector<Keypoint3D>& kp1, float** descriptor1, std::vector<Keypoint3D>& kp2, float** descriptor2, int* matched_idx);

		void createGaussianPyramid(Image3D* vol_img, std::vector<Layer3D>& pyramid);
		void createDogPyramid(std::vector<Layer3D>& gaussian_pyramid, std::vector<Layer3D>& dog_pyramid);
		void detectExtrema(std::vector<Layer3D>& dog_pyramid, std::vector<Keypoint3D>& kp_queue);
		void assignOrientation(std::vector<Keypoint3D>& kp_queue, std::vector<Layer3D>& gaussian_pyramid);
		void constructDescriptor(std::vector<Keypoint3D>& kp_queue, std::vector<Layer3D>& gaussian_pyramid, float** descriptor);
		void monodirectionalMatch(std::vector<Keypoint3D>& kp1, float** descriptor1, std::vector<Keypoint3D>& kp2, float** descriptor2,
			std::vector<Point3D>& matched_kp1, std::vector<Point3D>& matched_kp2); //monodirectional matching, but many-to-one correspondences are eliminated through reverse matching
		void bidirectionalMatch(std::vector<Keypoint3D>& kp1, float** descriptor1, std::vector<Keypoint3D>& kp2, float** descriptor2,
			std::vector<Point3D>& matched_kp1, std::vector<Point3D>& matched_kp2); //rigorous bidirectional matching
	};

	bool sortByEigenvalue(const EigenMatrix& em1, const EigenMatrix& em2);

	bool sortByRefIdx(const KeypointChecker& kc1, const KeypointChecker& kc2);
	bool sortByTarIdx(const KeypointChecker& kc1, const KeypointChecker& kc2);

	int mirrorLow(int x, int y); //deal the case that the index crosses the lower boundary 
	int mirrorHigh(int x, int y); //deal the case that the index crosses the higher boundary 

}//namespace opencorr

#endif //_SIFT_H_