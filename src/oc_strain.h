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

#ifndef _STRAIN_H_
#define _STRAIN_H_

#include <vector>
#include <algorithm>
#include "oc_array.h"
#include "oc_poi.h"
#include "oc_point.h"
#include "oc_subset.h"

namespace opencorr{

	class Strain2D
	{
	protected:
		Point2D topleft_point; //topleft corner of ROI
		int subregion_radius; //radius of subregion
		int grid_space; //grid space of POIs

	public:
		Eigen::MatrixXf u_map; //map of u-component
		Eigen::MatrixXf v_map; //map of v-component

		Strain2D(int subregion_radius, int grid_space);
		~Strain2D();

		int getSubregionRadius() const;
		int getGridSpace() const;

		void setSubregionRadius(int subregion_radius);
		void setGridSpace(int grid_space);
		void setDisplacement(std::vector<POI2D>& POI_queue);

		void prepare();
		virtual void compute(POI2D* POI) = 0;
	};

	bool sortByX(const POI2D& p1, const POI2D& p2);
	bool sortByY(const POI2D& p1, const POI2D& p2);

	//Savitzky-Golay filter based method
	class SGFilter : public Strain2D
	{
	public:
		SGFilter(int radius, int grid);
		~SGFilter();
		void compute(POI2D* poi);
		void compute(std::vector<POI2D>& poi_queue);
	};

	//regional least square fitting based method
	class LSFitting : public Strain2D
	{
	public:
		LSFitting(int radius, int grid);
		~LSFitting();
		void compute(POI2D* poi);
		void compute(std::vector<POI2D>& poi_queue);
	};

}//namespace opencorr

#endif //_STRAIN_H_


