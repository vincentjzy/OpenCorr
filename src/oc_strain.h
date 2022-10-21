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

#ifndef _STRAIN_H_
#define _STRAIN_H_

#include <vector>

#include "oc_array.h"
#include "oc_nearest_neighbor.h"
#include "oc_poi.h"
#include "oc_point.h"
#include "oc_subset.h"

namespace opencorr
{

	struct PointIndex
	{
		int index_in_queue; //index in the keypoint queue
		float distance_to_poi; //Euclidean distance to the processed POI
	};

	//strain calculation for 2D and stereo DIC
	class Strain
	{
	private:
		std::vector<NearestNeighbor*> instance_pool;
		NearestNeighbor* getInstance(int tid);

	protected:
		int subregion_radius; //radius of subregion
		int min_neighbor_num; //minimum number of neighbor POI required by fitting
		float zncc_threshold; //The POI with zncc above this threshold is regarded available
		int description; //description of strain, 1 for Lagranian and 2 for Eulerian
		int thread_number; //OpenMP thread number

		std::vector<POI2D> poi2d_queue; //POI queue for processing with 2D DIC results
		std::vector<POI2DS> poi2ds_queue; //POI queue for processing with stereo DIC results
		std::vector<POI3D> poi3d_queue; //POI queue for processing with DVC results

	public:

		Strain(int subregion_radius, int min_neighbor_num, std::vector<POI2D>& poi_queue, int thread_number);
		Strain(int subregion_radius, int min_neighbor_num, std::vector<POI2DS>& poi_queue, int thread_number);
		Strain(int subregion_radius, int min_neighbor_num, std::vector<POI3D>& poi_queue, int thread_number);
		~Strain();

		int getSubregionRadius() const;
		int getMinNeighborNumber() const;
		float getZNCCThreshold() const;

		void setSubregionRadius(int subregion_radius);
		void setMinNeighborNumer(int min_neighbor_num);
		void setZnccThreshold(float zncc_threshold);
		void setDescription(int description); //"1" for Lagrangian, "2" for Eulerian

		void setPOIQueue(std::vector<POI2D>& poi_queue);
		void setPOIQueue(std::vector<POI2DS>& poi_queue);
		void setPOIQueue(std::vector<POI3D>& poi_queue);

		void prepare(std::vector<POI2D>& poi_queue);
		void prepare(std::vector<POI2DS>& poi_queue);
		void prepare(std::vector<POI3D>& poi_queue);

		void compute(POI2D* poi);
		void compute(POI2DS* poi);
		void compute(POI3D* poi);

		void compute(std::vector<POI2D>& poi_queue);
		void compute(std::vector<POI2DS>& poi_queue);
		void compute(std::vector<POI3D>& poi_queue);
	};


	bool sortByDistance(const PointIndex& p1, const PointIndex& p2);

}//namespace opencorr

#endif //_STRAIN_H_


