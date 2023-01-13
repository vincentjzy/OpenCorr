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

#pragma once

#ifndef _FEATURE_AFFINE_H_
#define _FEATURE_AFFINE_H_

#include "oc_array.h"
#include "oc_dic.h"
#include "oc_image.h"
#include "oc_nearest_neighbor.h"
#include "oc_poi.h"
#include "oc_point.h"

namespace opencorr
{
	//parameters in RANSAC
	struct RansacConfig
	{
		int trial_number; //maximum number of trials in RANSAC
		int sample_mumber; //number of samples in every trial
		float error_threshold; //error threshold in RANSAC
	};

	//the 2D part of module is the implementation of
	//J. Yang et al, Optics and Lasers in Engineering (2020) 127: 105964.
	//https://doi.org/10.1016/j.optlaseng.2019.105964

	class FeatureAffine2D : public DIC
	{
	private:
		std::vector<NearestNeighbor*> instance_pool;
		NearestNeighbor* getInstance(int tid);

	protected:
		float neighbor_search_radius; //seaching radius for mached keypoints around a POI
		int min_neighbor_num; //minimum number of neighbors required by RANSAC
		RansacConfig ransac_config;

	public:
		std::vector<Point2D> ref_kp; //matched keypoints in ref image
		std::vector<Point2D> tar_kp; //matched keypoints in tar image

		FeatureAffine2D(int radius_x, int radius_y, int thread_number);
		~FeatureAffine2D();

		RansacConfig getRansacConfig() const;
		float getSearchRadius() const;
		int getMinNeighborNumber() const;

		void setSearchParameters(float neighbor_search_radius, int min_neighbor_num);
		void setRansacConfig(RansacConfig ransac_config);

		void setKeypointPair(std::vector<Point2D>& ref_kp, std::vector<Point2D>& tar_kp);
		void prepare();
		void compute(POI2D* poi);
		void compute(std::vector<POI2D>& poi_queue);

		//functions for self-adaptive subset
		void compute(POI2D* poi, int neighbor_k, int min_radius);
		void compute(std::vector<POI2D>& poi_queue, int neighbor_k, int min_radius);
	};


	//the 3D part of module is the implementation of
	//J. Yang et al, Optics and Lasers in Engineering (2021) 136: 106323.
	//https://doi.org/10.1016/j.optlaseng.2020.106323

	class FeatureAffine3D : public DVC
	{
	private:
		std::vector<NearestNeighbor*> instance_pool;
		NearestNeighbor* getInstance(int tid);

	protected:
		float neighbor_search_radius; //seaching radius for mached keypoints around a POI
		int min_neighbor_num; //minimum number of neighbors required by RANSAC
		RansacConfig ransac_config;

	public:
		std::vector<Point3D> ref_kp; //matched keypoints in ref image
		std::vector<Point3D> tar_kp; //matched keypoints in tar image

		FeatureAffine3D(int radius_x, int radius_y, int radius_z, int thread_number);
		~FeatureAffine3D();

		RansacConfig getRansacConfig() const;
		float getSearchRadius() const;
		int getMinNeighborNumber() const;

		void setSearchParameters(float neighbor_search_radius, int min_neighbor_num);
		void setRansacConfig(RansacConfig ransac_config);

		void setKeypointPair(std::vector<Point3D>& ref_kp, std::vector<Point3D>& tar_kp);
		void prepare();
		void compute(POI3D* poi);
		void compute(std::vector<POI3D>& poi_queue);
	};

}//namespace opencorr

#endif //_FEATURE_AFFINE_H_