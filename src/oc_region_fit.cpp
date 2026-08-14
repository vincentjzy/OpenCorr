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

 #include <omp.h>
 
 #include "oc_region_fit.h"

namespace opencorr
{
	//2D
	std::unique_ptr<NearestNeighbor>& RegionFit2D::getInstance(int tid)
	{
		if (tid >= instance_pool.size())
		{
			throw std::string("CPU thread ID over limit");
		}

		return instance_pool[tid];
	}

	RegionFit2D::RegionFit2D(float neighbor_search_radius, int neighbor_number_min, int thread_number)
	{
		this->neighbor_search_radius = neighbor_search_radius;
		this->neighbor_number_min = neighbor_number_min;
		this->thread_number = thread_number;

		instance_pool.resize(thread_number);
#pragma omp parallel for num_threads(thread_number)
		for (int i = 0; i < thread_number; i++)
		{
			instance_pool[i] = std::make_unique<NearestNeighbor>();
		}
	}

	RegionFit2D::~RegionFit2D()
	{
		for (auto& instance : instance_pool)
		{
			instance.reset();
		}
		std::vector<std::unique_ptr<NearestNeighbor>>().swap(instance_pool);
	}

	float RegionFit2D::getSearchRadius() const
	{
		return neighbor_search_radius;
	}

	int RegionFit2D::getNeighborMin() const
	{
		return neighbor_number_min;
	}

	void RegionFit2D::setSearchRadius(float neighbor_search_radius)
	{
		this->neighbor_search_radius = neighbor_search_radius;
	}

	void RegionFit2D::setNeighborMin(int neighbor_number_min)
	{
		this->neighbor_number_min = neighbor_number_min;
	}

	void RegionFit2D::setNeighbor(std::vector<POI2D>& reliable_pois)
	{
		neighbor_reliable = &reliable_pois;
	}

	void RegionFit2D::prepare()
	{
#pragma omp parallel for num_threads(thread_number)
		for (int i = 0; i < thread_number; i++)
		{
			instance_pool[i]->clear();

			instance_pool[i]->assignPoints(*neighbor_reliable);
			instance_pool[i]->setSearchRadius(neighbor_search_radius);
			instance_pool[i]->setSearchK(neighbor_number_min);
			instance_pool[i]->constructKdTree();
		}
	}

	void RegionFit2D::compute(POI2D* poi)
	{
		//get instance of NearestNeighbor according to thread id
		std::unique_ptr<NearestNeighbor>& neighbor_search = getInstance(omp_get_thread_num());

		//3D point for approximation of nearest neighbors
		Point3D current_point(poi->x, poi->y, 0.f);

		//POI queue for displacment field fitting
		std::vector<POI2D> pois_fit;

		//search the neighbor POIs in a region of given radius
		std::vector<nanoflann::ResultItem<uint32_t, float>> current_matches;
		int neighbor_num = neighbor_search->radiusSearch(current_point, current_matches);

		if (neighbor_num >= neighbor_number_min)
		{
			for (int i = 0; i < neighbor_num; i++)
			{
				pois_fit.emplace_back((*neighbor_reliable)[current_matches[i].first]);
			}
		}
		else //try KNN search if the obtained neighbor POIs are not enough
		{
			std::vector<uint32_t> k_neighbor_idx;
			std::vector<float> squared_distance;
			neighbor_num = neighbor_search->knnSearch(current_point, k_neighbor_idx, squared_distance);
			for (int i = 0; i < neighbor_num; i++)
			{
				pois_fit.emplace_back((*neighbor_reliable)[k_neighbor_idx[i]]);
			}
		}
		neighbor_num = (int)pois_fit.size();

		//terminate the procedure if the reliable neighbor POIs are not enough
		if (neighbor_num >= neighbor_number_min)
		{
			//create matrices of displacments
			Eigen::VectorXf u_vector(neighbor_num);
			Eigen::VectorXf v_vector(neighbor_num);

			//create the matrix of local coordinates
			Eigen::MatrixXf local_corr_matrix = Eigen::MatrixXf::Zero(neighbor_num, 3);

			//fill local coordinate matrix, u_vector and v_vector
			for (int i = 0; i < neighbor_num; i++)
			{
				local_corr_matrix(i, 0) = 1.f;
				local_corr_matrix(i, 1) = pois_fit[i].x - poi->x;
				local_corr_matrix(i, 2) = pois_fit[i].y - poi->y;

				u_vector(i) = pois_fit[i].deformation.u;
				v_vector(i) = pois_fit[i].deformation.v;
			}

			//solve the equations to obtain gradients of u and v
			Eigen::VectorXf u_gradient = local_corr_matrix.colPivHouseholderQr().solve(u_vector);
			Eigen::VectorXf v_gradient = local_corr_matrix.colPivHouseholderQr().solve(v_vector);

			poi->deformation.u = u_gradient(0);
			poi->deformation.ux = u_gradient(1);
			poi->deformation.uy = u_gradient(2);

			poi->deformation.v = v_gradient(0);
			poi->deformation.vx = v_gradient(1);
			poi->deformation.vy = v_gradient(2);

			//reset zncc
			poi->result.zncc = 0;
		}
	}

	void RegionFit2D::compute(std::vector<POI2D>& poi_queue)
	{
		auto queue_length = poi_queue.size();
#pragma omp parallel for num_threads(thread_number)
		for (int i = 0; i < queue_length; i++)
		{
			compute(&poi_queue[i]);
		}
	}



	//3D
	std::unique_ptr<NearestNeighbor>& RegionFit3D::getInstance(int tid)
	{
		if (tid >= (int)instance_pool.size())
		{
			throw std::string("CPU thread ID over limit");
		}

		return instance_pool[tid];
	}

	RegionFit3D::RegionFit3D(float neighbor_search_radius, int neighbor_number_min, int thread_number)
	{
		this->neighbor_search_radius = neighbor_search_radius;
		this->neighbor_number_min = neighbor_number_min;
		this->thread_number = thread_number;

		instance_pool.resize(thread_number);
#pragma omp parallel for num_threads(thread_number)
		for (int i = 0; i < thread_number; i++)
		{
			instance_pool[i] = std::make_unique<NearestNeighbor>();
		}
	}

	RegionFit3D::~RegionFit3D()
	{
		for (auto& instance : instance_pool)
		{
			instance.reset();
		}
		std::vector<std::unique_ptr<NearestNeighbor>>().swap(instance_pool);
	}

	float RegionFit3D::getSearchRadius() const
	{
		return neighbor_search_radius;
	}

	int RegionFit3D::getNeighborMin() const
	{
		return neighbor_number_min;
	}

	void RegionFit3D::setSearchRadius(float neighbor_search_radius)
	{
		this->neighbor_search_radius = neighbor_search_radius;
	}

	void RegionFit3D::setNeighborMin(int neighbor_number_min)
	{
		this->neighbor_number_min = neighbor_number_min;
	}

	void RegionFit3D::setNeighbor(std::vector<POI3D>& reliable_pois)
	{
		neighbor_reliable = &reliable_pois;
	}

	void RegionFit3D::prepare()
	{
#pragma omp parallel for num_threads(thread_number)
		for (int i = 0; i < thread_number; i++)
		{
			instance_pool[i]->clear();

			instance_pool[i]->assignPoints(*neighbor_reliable);
			instance_pool[i]->setSearchRadius(neighbor_search_radius);
			instance_pool[i]->setSearchK(neighbor_number_min);
			instance_pool[i]->constructKdTree();
		}
	}

	void RegionFit3D::compute(POI3D* poi)
	{
		//get instance of NearestNeighbor according to thread id
		std::unique_ptr<NearestNeighbor>& neighbor_search = getInstance(omp_get_thread_num());

		//3D point for approximation of nearest neighbors
		Point3D current_point(poi->x, poi->y, poi->z);

		//POI queue for displacment field fitting
		std::vector<POI3D> pois_fit;

		//search the neighbor POIs in a region of given radius
		std::vector<nanoflann::ResultItem<uint32_t, float>> current_matches;
		int neighbor_num = neighbor_search->radiusSearch(current_point, current_matches);

		if (neighbor_num >= neighbor_number_min)
		{
			for (int i = 0; i < neighbor_num; i++)
			{
				pois_fit.emplace_back((*neighbor_reliable)[current_matches[i].first]);
			}
		}
		else //try KNN search if the obtained neighbor POIs are not enough
		{
			std::vector<uint32_t> k_neighbor_idx;
			std::vector<float> squared_distance;
			neighbor_num = neighbor_search->knnSearch(current_point, k_neighbor_idx, squared_distance);
			for (int i = 0; i < neighbor_num; i++)
			{
				pois_fit.emplace_back((*neighbor_reliable)[k_neighbor_idx[i]]);
			}
		}
		neighbor_num = (int)pois_fit.size();

		//terminate the procedure if the reliable neighbor POIs are not enough
		if (neighbor_num >= neighbor_number_min)
		{
			//create matrices of displacments
			Eigen::VectorXf u_vector(neighbor_num);
			Eigen::VectorXf v_vector(neighbor_num);
			Eigen::VectorXf w_vector(neighbor_num);

			//create the matrix of local coordinates
			Eigen::MatrixXf local_corr_matrix = Eigen::MatrixXf::Zero(neighbor_num, 4);

			//fill local coordinate matrix, u_vector and v_vector
			for (int i = 0; i < neighbor_num; i++)
			{
				local_corr_matrix(i, 0) = 1.f;
				local_corr_matrix(i, 1) = pois_fit[i].x - poi->x;
				local_corr_matrix(i, 2) = pois_fit[i].y - poi->y;
				local_corr_matrix(i, 3) = pois_fit[i].z - poi->z;

				u_vector(i) = pois_fit[i].deformation.u;
				v_vector(i) = pois_fit[i].deformation.v;
				w_vector(i) = pois_fit[i].deformation.w;
			}

			//solve the equations to obtain gradients of u and v
			Eigen::VectorXf u_gradient = local_corr_matrix.colPivHouseholderQr().solve(u_vector);
			Eigen::VectorXf v_gradient = local_corr_matrix.colPivHouseholderQr().solve(v_vector);
			Eigen::VectorXf w_gradient = local_corr_matrix.colPivHouseholderQr().solve(w_vector);

			poi->deformation.u = u_gradient(0);
			poi->deformation.ux = u_gradient(1);
			poi->deformation.uy = u_gradient(2);
			poi->deformation.uz = u_gradient(3);

			poi->deformation.v = v_gradient(0);
			poi->deformation.vx = v_gradient(1);
			poi->deformation.vy = v_gradient(2);
			poi->deformation.vz = v_gradient(3);

			poi->deformation.w = w_gradient(0);
			poi->deformation.wx = w_gradient(1);
			poi->deformation.wy = w_gradient(2);
			poi->deformation.wz = w_gradient(3);

			//reset zncc
			poi->result.zncc = 0.f;
		}
	}

	void RegionFit3D::compute(std::vector<POI3D>& poi_queue)
	{
		auto queue_length = poi_queue.size();
#pragma omp parallel for num_threads(thread_number)
		for (int i = 0; i < queue_length; i++)
		{
			compute(&poi_queue[i]);
		}
	}

}//namespace opencorr