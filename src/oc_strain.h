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

#ifndef _STRAIN_H_
#define _STRAIN_H_

#include "oc_array.h"
#include "oc_nearest_neighbor.h"

namespace opencorr
{
	struct PointIndex //structure for brute force search
	{
		int poi_idx; //index the poi queue
		float distance; //Euclidean distance to the processed POI
	};

	//calculation of Green-Lagrangian strain
	class Strain
	{
	private:
		std::vector<NearestNeighbor*> instance_pool;
		NearestNeighbor* getInstance(int tid);

	protected:
		float subregion_radius; //radius of subregion
		int neighbor_number_min; //minimum number of neighbor POI required by fitting
		float zncc_threshold; //POI with ZNCC above this threshold is regarded available
		int description; //description of strain, 1 for Lagranian and 2 for Eulerian
		int approximation; //approximation of strain, 1 for Cauchy strain and 2 for Green strain
		int thread_number; //CPU thread number

	public:

		Strain(float subregion_radius, int neighbor_number_min, int thread_number);
		~Strain();

		float getSubregionRadius() const;
		int getNeighborMin() const;
		float getZnccThreshold() const;

		void setSubregionRadius(float subregion_radius);
		void setNeighborMin(int neighbor_number_min);
		void setZnccThreshold(float zncc_threshold);
		void setDescription(int description); //"1" for Lagrangian, "2" for Eulerian
		void setApproximation(int approximation); //"1" for Cauchy strain, "2" for Green strain

		void prepare(std::vector<POI2D>& poi_queue);
		void prepare(std::vector<POI2DS>& poi_queue);
		void prepare(std::vector<POI3D>& poi_queue);

		void compute(POI2D* poi, std::vector<POI2D>& poi_queue);
		void compute(POI2DS* poi, std::vector<POI2DS>& poi_queue);
		void compute(POI3D* poi, std::vector<POI3D>& poi_queue);

		void compute(std::vector<POI2D>& poi_queue);
		void compute(std::vector<POI2DS>& poi_queue);
		void compute(std::vector<POI3D>& poi_queue);
	};


	bool sortByDistance(const PointIndex& p1, const PointIndex& p2);

}//namespace opencorr

#endif //_STRAIN_H_