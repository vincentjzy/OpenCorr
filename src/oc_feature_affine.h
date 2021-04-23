/*
 * This file is part of OpenCorr, an open-source C++ library for
 * study and development of 2D, 3D/stereo and volumetric
 * digital image correlation.
 *
 * Copyright (C) 2021, Zhenyu Jiang <zhenyujiang@scut.edu.cn>
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * More information about OpenCorr can be found at https://www.opencorr.org/
 */

#pragma once

#ifndef _FEATURE_AFFINE_H_
#define _FEATURE_AFFINE_H_
#include <omp.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "oc_array.h"
#include "oc_dic.h"
#include "oc_image.h"
#include "oc_poi.h"
#include "oc_point.h"
#include "oc_subset.h"

namespace opencorr
{
	struct RANSACconfig {
		int trial_number; // maximum number of trials in RANSAC
		int sample_mumber; // number of samples in every trial
		float error_threshold; //error threshold in RANSAC
	};

	class FeatureAffine2D : public DIC
	{
	protected:
		float neighbor_search_radius; // seaching radius for mached keypoints around a POI
		int essential_neighbor_number; // minimum number of neighbors required by RANSAC
		RANSACconfig RANSAC_parameters;
		Eigen::Matrix3f affine_matrix;

	public:
		std::vector<Point2D> ref_keypoints;
		std::vector<Point2D> tar_keypoints;

		FeatureAffine2D(int subset_radius_x, int subset_radius_y);
		~FeatureAffine2D();

		void setKeypointPair(std::vector<Point2D>& ref_keypoints, std::vector<Point2D>& tar_keypoints);
		void prepare();
		void compute(POI2D* POI);

		RANSACconfig getRANSACparameters() const;
		float getSearchRadius() const;
		int getEssentialNeighborNumber() const;
		void setSubsetRadii(int subset_radius_x, int subset_radius_y);
		void setSearchParameters(float neighbor_search_radius, int essential_neighbor_number);
		void setRANSAC(RANSACconfig RANSAC_parameters);
	};

}//namespace opencorr

#endif //_FEATURE_AFFINE_H_