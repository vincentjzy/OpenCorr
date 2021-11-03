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

#pragma once

#ifndef _FEATURE_AFFINE_H_
#define _FEATURE_AFFINE_H_
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
		int trial_number; //maximum number of trials in RANSAC
		int sample_mumber; //number of samples in every trial
		float error_threshold; //error threshold in RANSAC
	};

	struct KeypointIndex {
		int index_in_queue; //index in the keypoint queue
		float distance_to_poi; //Euclidean distance to the processed POI
	};

	class FeatureAffine2D : public DIC
	{
	protected:
		float neighbor_search_radius; //seaching radius for mached keypoints around a POI
		int min_neighbor_num; //minimum number of neighbors required by RANSAC
		RANSACconfig ransac_config;

	public:
		std::vector<Point2D> ref_kp; //matched keypoints in ref image
		std::vector<Point2D> tar_kp; //matched keypoints in tar image

		FeatureAffine2D(int subset_radius_x, int subset_radius_y);
		~FeatureAffine2D();

		void setKeypointPair(std::vector<Point2D>& ref_kp, std::vector<Point2D>& tar_kp);
		void prepare();
		void compute(POI2D* poi);
		void compute(std::vector<POI2D>& poi_queue);

		RANSACconfig getRANSAC() const;
		float getSearchRadius() const;
		int getMinimumNeighborNumber() const;

		void setSearchParameters(float neighbor_search_radius, int min_neighbor_num);
		void setRANSAC(RANSACconfig ransac_config);
	};

	bool sortByDistance(const KeypointIndex& kp1, const KeypointIndex& kp2);

}//namespace opencorr

#endif //_FEATURE_AFFINE_H_