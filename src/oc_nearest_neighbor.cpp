/*
 * This file is part of OpenCorr, an open source C++ library for
 * study and development of 2D, 3D/stereo and volumetric
 * digital image correlation.
 *
 * Copyright (C) 2021-2024, Zhenyu Jiang <zhenyujiang@scut.edu.cn>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one from http://mozilla.org/MPL/2.0/.
 *
 * More information about OpenCorr can be found at https://www.opencorr.org/
 */

#include "oc_nearest_neighbor.h"

namespace opencorr
{
	NearestNeighbor::NearestNeighbor() {}

	NearestNeighbor::~NearestNeighbor()
	{
		if (kdt_index != nullptr)
		{
			delete kdt_index;
		}
	}

	void NearestNeighbor::assignPoints(std::vector<Point2D>& point_queue)
	{
		int queue_length = (int)point_queue.size();
		point_cloud.pts.resize(queue_length);
#pragma omp parallel for
		for (int i = 0; i < queue_length; i++)
		{
			point_cloud.pts[i].x = point_queue[i].x;
			point_cloud.pts[i].y = point_queue[i].y;
			point_cloud.pts[i].z = 0.f;
		}
	}

	void NearestNeighbor::assignPoints(std::vector<POI2D>& poi_queue)
	{
		int queue_length = (int)poi_queue.size();
		point_cloud.pts.resize(queue_length);
#pragma omp parallel for
		for (int i = 0; i < queue_length; i++)
		{
			point_cloud.pts[i].x = poi_queue[i].x;
			point_cloud.pts[i].y = poi_queue[i].y;
			point_cloud.pts[i].z = 0.f;
		}
	}

	void NearestNeighbor::assignPoints(std::vector<Point3D>& point_queue)
	{
		int queue_length = (int)point_queue.size();
		point_cloud.pts.resize(queue_length);
#pragma omp parallel for
		for (int i = 0; i < queue_length; i++)
		{
			point_cloud.pts[i].x = point_queue[i].x;
			point_cloud.pts[i].y = point_queue[i].y;
			point_cloud.pts[i].z = point_queue[i].z;
		}
	}

	void NearestNeighbor::assignPoints(std::vector<POI3D>& poi_queue)
	{
		int queue_length = (int)poi_queue.size();
		point_cloud.pts.resize(queue_length);
#pragma omp parallel for
		for (int i = 0; i < queue_length; i++)
		{
			point_cloud.pts[i].x = poi_queue[i].x;
			point_cloud.pts[i].y = poi_queue[i].y;
			point_cloud.pts[i].z = poi_queue[i].z;
		}
	}

	float NearestNeighbor::getSearchRadius() const
	{
		return search_radius;
	}

	int NearestNeighbor::getSearchK() const
	{
		return search_k;
	}

	void NearestNeighbor::setSearchRadius(float search_radius)
	{
		this->search_radius = search_radius;
	}

	void NearestNeighbor::setSearchK(int search_k)
	{
		this->search_k = search_k;
	}

	void NearestNeighbor::constructKdTree()
	{
		// construct a kd-tree index
		using kdTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloud>, PointCloud, 3>;

		kdt_index = new kdTree(3 /*dim*/, point_cloud, { 10 /* max leaf */ });
	}

	int NearestNeighbor::radiusSearch(Point3D query_point, std::vector<nanoflann::ResultItem<uint32_t, float>>& matches)
	{
		float squared_radius = search_radius * search_radius;

		query_coor[0] = query_point.x;
		query_coor[1] = query_point.y;
		query_coor[2] = query_point.z;

		nanoflann::SearchParameters params;
		params.sorted = false;

		int num_matches = (int)kdt_index->radiusSearch(&query_coor[0], squared_radius, matches, params);

		return num_matches;
	}

	int NearestNeighbor::radiusSearch(Point3D query_point, float search_radius, std::vector<nanoflann::ResultItem<uint32_t, float>>& matches)
	{
		float squared_radius = search_radius * search_radius;

		query_coor[0] = query_point.x;
		query_coor[1] = query_point.y;
		query_coor[2] = query_point.z;

		nanoflann::SearchParameters params;
		params.sorted = false;

		int num_matches = (int)kdt_index->radiusSearch(&query_coor[0], squared_radius, matches, params);

		return num_matches;
	}

	int NearestNeighbor::knnSearch(Point3D query_point, std::vector<uint32_t>& k_neighbors_idx, std::vector<float>& kp_squared_distance)
	{
		k_neighbors_idx.resize(search_k);
		kp_squared_distance.resize(search_k);

		query_coor[0] = query_point.x;
		query_coor[1] = query_point.y;
		query_coor[2] = query_point.z;

		int num_matches = (int)kdt_index->knnSearch(&query_coor[0], search_k, &k_neighbors_idx[0], &kp_squared_distance[0]);

		//in case of insufficient keypoints in the tree than requested
		k_neighbors_idx.resize(num_matches);
		kp_squared_distance.resize(num_matches);

		return num_matches;
	}

	int NearestNeighbor::knnSearch(Point3D query_point, int search_k, std::vector<uint32_t>& k_neighbors_idx, std::vector<float>& kp_squared_distance)
	{
		k_neighbors_idx.resize(search_k);
		kp_squared_distance.resize(search_k);

		query_coor[0] = query_point.x;
		query_coor[1] = query_point.y;
		query_coor[2] = query_point.z;

		int num_matches = (int)kdt_index->knnSearch(&query_coor[0], search_k, &k_neighbors_idx[0], &kp_squared_distance[0]);

		//in case of insufficient keypoints in the tree than requested
		k_neighbors_idx.resize(num_matches);
		kp_squared_distance.resize(num_matches);

		return num_matches;
	}

}//namespace opencorr
