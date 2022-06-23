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

#ifndef _NEAREST_NEIGHBORS_H_
#define _NEAREST_NEIGHBORS_H_

#include <vector>
#include <nanoflann.hpp>

#include "oc_poi.h"
#include "oc_point.h"

namespace opencorr
{
	struct Point {
		float x, y, z;
	};

	struct PointCloud {

		using coord_t = float;  //!< The type of each coordinate

		std::vector<Point> pts;

		//return the number of points
		inline size_t kdtree_get_point_count() const { return pts.size(); }

		//return the dim'th component of the idx'th point
		inline float kdtree_get_pt(const size_t idx, const size_t dim) const
		{
			if (dim == 0)
				return pts[idx].x;
			else if (dim == 1)
				return pts[idx].y;
			else
				return pts[idx].z;
		}

		//optional bounding-box computation
		template <class BBOX>
		bool kdtree_get_bbox(BBOX& /* bb */) const
		{
			return false;
		}
	};

	class NearestNeighbor {
	protected:
		PointCloud point_cloud;
		float search_radius;
		int search_k;
		float query_coor[3] = { 0 };

		nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloud>, PointCloud, 3 /* dim */>* kdt_index;

	public:
		NearestNeighbor();
		~NearestNeighbor();

		void assignPoints(std::vector<Point2D>& point_queue);
		void assignPoints(std::vector<POI2D>& poi_queue);
		void assignPoints(std::vector<Point3D>& point_queue);
		void assignPoints(std::vector<POI3D>& poi_queue);

		float getSearchRadius() const;
		int getSearchK() const;
		void setSearchRadius(float search_radius);
		void setSearchK(int search_k);

		void constructKdTree();
		int radiusSearch(Point3D query_point, std::vector<std::pair<uint32_t, float>>& matches);
		int knnSearch(Point3D query_point, std::vector<uint32_t>& k_neighbors_idx, std::vector<float>& kp_squared_distance);
	};
}//namespace opencorr

#endif //_NEAREST_NEIGHBORS_H_