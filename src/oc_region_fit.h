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

#ifndef _REGION_FIT_H_
#define _REGION_FIT_H_

#include "oc_dic.h"
#include "oc_nearest_neighbor.h"

namespace opencorr
{
	//2D
	class RegionFit2D : public DIC
	{
	private:
		std::vector<std::unique_ptr<NearestNeighbor>> instance_pool;
		std::unique_ptr<NearestNeighbor>& getInstance(int tid);

	protected:
		std::vector<POI2D>* neighbor_reliable; //POIs to create cloud for deformation estimation
		float neighbor_search_radius; //seaching radius for mached keypoints around a POI
		int neighbor_number_min; //minimum number of neighbors required by RANSAC

	public:
		RegionFit2D(float neighbor_search_radius, int neighbor_number_min, int thread_number);
		~RegionFit2D();

		float getSearchRadius() const;
		int getNeighborMin() const;

		void setSearchRadius(float neighbor_search_radius);
		void setNeighborMin(int neighbor_number_min);
		void setNeighbor(std::vector<POI2D>& reliable_pois);

		void prepare();
		void compute(POI2D* poi);
		void compute(std::vector<POI2D>& poi_queue);
	};

	//3D
	class RegionFit3D : public DVC
	{
	private:
		std::vector<std::unique_ptr<NearestNeighbor>> instance_pool;
		std::unique_ptr<NearestNeighbor>& getInstance(int tid);

	protected:
		std::vector<POI3D>* neighbor_reliable; //POIs to create cloud for deformation estimation
		float neighbor_search_radius; //seaching radius for mached keypoints around a POI
		int neighbor_number_min; //minimum number of neighbors required by RANSAC

	public:
		RegionFit3D(float neighbor_search_radius, int neighbor_number_min, int thread_number);
		~RegionFit3D();

		float getSearchRadius() const;
		int getNeighborMin() const;

		void setSearchRadius(float neighbor_search_radius);
		void setNeighborMin(int neighbor_number_min);
		void setNeighbor(std::vector<POI3D>& reliable_pois);

		void prepare();
		void compute(POI3D* poi);
		void compute(std::vector<POI3D>& poi_queue);
	};

}//namespace opencorr

#endif //_REGION_FIT_H_