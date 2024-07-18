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

#include "oc_strain.h"

namespace opencorr
{
	NearestNeighbor* Strain::getInstance(int tid)
	{
		if (tid >= (int)instance_pool.size())
		{
			throw std::string("CPU thread ID over limit");
		}

		return instance_pool[tid];
	}

	Strain::Strain(float subregion_radius, int neighbor_number_min, int thread_number)
	{
		setSubregionRadius(subregion_radius);
		setNeighborMin(neighbor_number_min);

		setZnccThreshold(0.9f);
		setDescription(1);
		setApproximation(1);

		this->thread_number = thread_number;
		for (int i = 0; i < thread_number; i++)
		{
			NearestNeighbor* instance = new NearestNeighbor();
			instance_pool.push_back(instance);
		}
	}

	Strain::~Strain()
	{
		for (auto& instance : instance_pool)
		{
			delete instance;
		}
		std::vector<NearestNeighbor*>().swap(instance_pool);
	}

	float Strain::getSubregionRadius() const
	{
		return subregion_radius;
	}

	int Strain::getNeighborMin() const
	{
		return neighbor_number_min;
	}

	float Strain::getZnccThreshold() const
	{
		return zncc_threshold;
	}

	void Strain::setSubregionRadius(float subregion_radius)
	{
		this->subregion_radius = subregion_radius;
	}

	void Strain::setNeighborMin(int neighbor_number_min)
	{
		this->neighbor_number_min = neighbor_number_min;
	}

	void Strain::setZnccThreshold(float zncc_threshold)
	{
		this->zncc_threshold = zncc_threshold;
	}

	void Strain::setDescription(int description)
	{
		this->description = description;
	}

	void Strain::setApproximation(int approximation)
	{
		this->approximation = approximation;
	}

	void Strain::prepare(std::vector<POI2D>& poi_queue)
	{
#pragma omp parallel for
		for (int i = 0; i < thread_number; i++)
		{
			instance_pool[i]->clear();

			instance_pool[i]->assignPoints(poi_queue);
			instance_pool[i]->setSearchRadius(subregion_radius);
			instance_pool[i]->setSearchK(neighbor_number_min);
			instance_pool[i]->constructKdTree();
		}
	}

	void Strain::prepare(std::vector<POI2DS>& poi_queue)
	{
		int queue_size = (int)poi_queue.size();
		std::vector<Point2D> pt_queue;
		pt_queue.resize(queue_size);
#pragma omp parallel for
		for (int i = 0; i < queue_size; i++)
		{
			pt_queue[i].x = poi_queue[i].x;
			pt_queue[i].y = poi_queue[i].y;
		}

#pragma omp parallel for
		for (int i = 0; i < thread_number; i++)
		{
			instance_pool[i]->clear();

			instance_pool[i]->assignPoints(pt_queue);
			instance_pool[i]->setSearchRadius(subregion_radius);
			instance_pool[i]->setSearchK(neighbor_number_min);
			instance_pool[i]->constructKdTree();
		}
	}

	void Strain::prepare(std::vector<POI3D>& poi_queue)
	{
#pragma omp parallel for
		for (int i = 0; i < thread_number; i++)
		{
			instance_pool[i]->clear();

			instance_pool[i]->assignPoints(poi_queue);
			instance_pool[i]->setSearchRadius(subregion_radius);
			instance_pool[i]->setSearchK(neighbor_number_min);
			instance_pool[i]->constructKdTree();
		}
	}

	void Strain::compute(POI2D* poi, std::vector<POI2D>& poi_queue)
	{
		//get instance of NearestNeighbor according to thread id
		NearestNeighbor* neighbor_search = getInstance(omp_get_thread_num());

		//3D point for approximation of nearest neighbors
		Point3D current_point(poi->x, poi->y, 0.f);

		//POI queue for displacment field fitting
		std::vector<POI2D> pois_fit;

		//search the neighbor POIs in a subregion of given radius
		std::vector<nanoflann::ResultItem<uint32_t, float>> current_matches;
		int neighbor_num = neighbor_search->radiusSearch(current_point, current_matches);
		if (neighbor_num >= neighbor_number_min)
		{
			for (int i = 0; i < neighbor_num; i++)
			{
				if (poi_queue[current_matches[i].first].result.zncc >= zncc_threshold)
				{
					pois_fit.push_back(poi_queue[current_matches[i].first]);
				}
			}
		}
		else //try KNN search if the obtained neighbor POIs are not enough
		{
			std::vector<POI2D>().swap(pois_fit);

			std::vector<uint32_t> k_neighbors_idx;
			std::vector<float> squared_distance;
			neighbor_num = neighbor_search->knnSearch(current_point, k_neighbors_idx, squared_distance);

			for (int i = 0; i < neighbor_num; i++)
			{
				if (poi_queue[k_neighbors_idx[i]].result.zncc >= zncc_threshold)
				{
					pois_fit.push_back(poi_queue[k_neighbors_idx[i]]);
				}
			}
		}
		neighbor_num = (int)pois_fit.size();

		//terminate the procedure if there are insufficient neighbors for strain calculation
		if (neighbor_num >= neighbor_number_min)
		{
			//create matrices of displacments
			Eigen::VectorXf u_vector(neighbor_num);
			Eigen::VectorXf v_vector(neighbor_num);

			//construct coefficient matrix
			Eigen::MatrixXf coefficient_matrix = Eigen::MatrixXf::Zero(neighbor_num, 3);

			//fill coefficient matrix, u_vector and v_vector
			for (int i = 0; i < neighbor_num; i++)
			{
				coefficient_matrix(i, 0) = 1.f;
				coefficient_matrix(i, 1) = pois_fit[i].x - poi->x;
				coefficient_matrix(i, 2) = pois_fit[i].y - poi->y;

				u_vector(i) = pois_fit[i].deformation.u;
				v_vector(i) = pois_fit[i].deformation.v;
			}

			//solve the equations to obtain gradients of u and v
			Eigen::VectorXf u_gradient = coefficient_matrix.colPivHouseholderQr().solve(u_vector);
			Eigen::VectorXf v_gradient = coefficient_matrix.colPivHouseholderQr().solve(v_vector);
			float ux = u_gradient(1, 0);
			float uy = u_gradient(2, 0);
			float vx = v_gradient(1, 0);
			float vy = v_gradient(2, 0);

			if (approximation == 1)
			{
				//calculate the Cauchy strain and save them for output
				poi->strain.exx = ux;
				poi->strain.eyy = vy;
				poi->strain.exy = 0.5f * (uy + vx);
			}
			if (approximation == 2)
			{
				//calculate the Green strain and save them for output
				poi->strain.exx = ux + 0.5f * (ux * ux + vx * vx);
				poi->strain.eyy = vy + 0.5f * (uy * uy + vy * vy);
				poi->strain.exy = 0.5f * (uy + vx + uy * ux + vy * vx);
			}
		}
	}

	void Strain::compute(std::vector<POI2D>& poi_queue)
	{
		int queue_length = (int)poi_queue.size();
#pragma omp parallel for
		for (int i = 0; i < queue_length; i++)
		{
			if (poi_queue[i].result.zncc >= zncc_threshold)
			{
				compute(&poi_queue[i], poi_queue);
			}
		}
	}

	void Strain::compute(POI2DS* poi, std::vector<POI2DS>& poi_queue)
	{
		//get instance of NearestNeighbor according to thread id
		NearestNeighbor* neighbor_search = getInstance(omp_get_thread_num());

		//3D point for approximation of nearest neighbors
		Point3D current_point(poi->x, poi->y, 0.f);

		//POI queue for displacment field fitting
		std::vector<POI2DS> pois_fit;

		//search the neighbor keypoints in a subregion of given radius
		std::vector<nanoflann::ResultItem<uint32_t, float>> current_matches;
		int neighbor_num = neighbor_search->radiusSearch(current_point, current_matches);
		if (neighbor_num >= neighbor_number_min)
		{
			for (int i = 0; i < neighbor_num; i++)
			{
				if (poi_queue[current_matches[i].first].result.r1r2_zncc >= zncc_threshold
					&& poi_queue[current_matches[i].first].result.r1t1_zncc >= zncc_threshold
					&& poi_queue[current_matches[i].first].result.r1t2_zncc >= zncc_threshold)
				{
					pois_fit.push_back(poi_queue[current_matches[i].first]);
				}
			}
		}
		else //try KNN search if the obtained neighbor POIs are not enough
		{
			std::vector<POI2DS>().swap(pois_fit);

			std::vector<uint32_t> k_neighbors_idx;
			std::vector<float> squared_distance;
			neighbor_num = neighbor_search->knnSearch(current_point, k_neighbors_idx, squared_distance);

			for (int i = 0; i < neighbor_num; i++)
			{
				if (poi_queue[k_neighbors_idx[i]].result.r1r2_zncc >= zncc_threshold
					&& poi_queue[k_neighbors_idx[i]].result.r1t1_zncc >= zncc_threshold
					&& poi_queue[k_neighbors_idx[i]].result.r1t2_zncc >= zncc_threshold)
				{
					pois_fit.push_back(poi_queue[k_neighbors_idx[i]]);
				}
			}
		}
		neighbor_num = (int)pois_fit.size();

		//terminate the procedure if there are insufficient neighbors for strain calculation
		if (neighbor_num >= neighbor_number_min)
		{
			//create matrices of displacments
			Eigen::VectorXf u_vector(neighbor_num);
			Eigen::VectorXf v_vector(neighbor_num);
			Eigen::VectorXf w_vector(neighbor_num);

			//construct coefficient matrix
			Eigen::MatrixXf coefficient_matrix = Eigen::MatrixXf::Zero(neighbor_num, 4);

			//fill coefficient matrix, u_vector, v_vector, and w_vector
			for (int i = 0; i < neighbor_num; i++)
			{
				Point3D current_pt_3d = pois_fit[i].ref_coor - poi->ref_coor;
				coefficient_matrix(i, 0) = 1.f;
				coefficient_matrix(i, 1) = current_pt_3d.x;
				coefficient_matrix(i, 2) = current_pt_3d.y;
				coefficient_matrix(i, 3) = current_pt_3d.z;

				u_vector(i) = pois_fit[i].deformation.u;
				v_vector(i) = pois_fit[i].deformation.v;
				w_vector(i) = pois_fit[i].deformation.w;
			}

			//solve the equations to obtain gradients of u, v, and w
			Eigen::VectorXf u_gradient = coefficient_matrix.colPivHouseholderQr().solve(u_vector);
			Eigen::VectorXf v_gradient = coefficient_matrix.colPivHouseholderQr().solve(v_vector);
			Eigen::VectorXf w_gradient = coefficient_matrix.colPivHouseholderQr().solve(w_vector);
			float ux = u_gradient(1, 0);
			float uy = u_gradient(2, 0);
			float uz = u_gradient(3, 0);
			float vx = v_gradient(1, 0);
			float vy = v_gradient(2, 0);
			float vz = v_gradient(3, 0);
			float wx = w_gradient(1, 0);
			float wy = w_gradient(2, 0);
			float wz = w_gradient(3, 0);

			if (approximation == 1)
			{
				//calculate the Cauchy strain and save them for output
				poi->strain.exx = ux;
				poi->strain.eyy = vy;
				poi->strain.ezz = wz;
				poi->strain.exy = 0.5f * (uy + vx);
				poi->strain.eyz = 0.5f * (vz + wy);
				poi->strain.ezx = 0.5f * (wx + uz);
			}
			if (approximation == 2)
			{
				poi->strain.exx = ux + 0.5f * (ux * ux + vx * vx + wx * wx);
				poi->strain.eyy = vy + 0.5f * (uy * uy + vy * vy + wy * wy);
				poi->strain.ezz = wz + 0.5f * (uz * uz + vz * vz + wz * wz);
				poi->strain.exy = 0.5f * (uy + vx + uy * ux + vy * vx + wy * wx);
				poi->strain.eyz = 0.5f * (vz + wy + uz * uy + vz * vy + wz * wy);
				poi->strain.ezx = 0.5f * (wx + uz + ux * uz + vx * vz + wx * wz);
			}
		}
	}

	void Strain::compute(std::vector<POI2DS>& poi_queue)
	{
		int queue_length = (int)poi_queue.size();
#pragma omp parallel for
		for (int i = 0; i < queue_length; i++)
		{
			if (poi_queue[i].result.r1r2_zncc >= zncc_threshold
				&& poi_queue[i].result.r1t1_zncc >= zncc_threshold
				&& poi_queue[i].result.r1t2_zncc >= zncc_threshold)
			{
				compute(&poi_queue[i], poi_queue);
			}
		}
	}

	void Strain::compute(POI3D* poi, std::vector<POI3D>& poi_queue)
	{
		//get instance of NearestNeighbor according to thread id
		NearestNeighbor* neighbor_search = getInstance(omp_get_thread_num());

		//3D point for approximation of nearest neighbors
		Point3D current_point(poi->x, poi->y, poi->z);

		//POI queue for displacment field fitting
		std::vector<POI3D> pois_fit;

		//search the neighbor keypoints in a subregion of given radius
		std::vector<nanoflann::ResultItem<uint32_t, float>> current_matches;
		int neighbor_num = neighbor_search->radiusSearch(current_point, current_matches);
		if (neighbor_num >= neighbor_number_min)
		{
			for (int i = 0; i < neighbor_num; i++)
			{
				if (poi_queue[current_matches[i].first].result.zncc >= zncc_threshold)
				{
					pois_fit.push_back(poi_queue[current_matches[i].first]);
				}
			}
		}
		else //try KNN search if the obtained neighbor POIs are not enough
		{
			std::vector<POI3D>().swap(pois_fit);

			std::vector<uint32_t> k_neighbors_idx;
			std::vector<float> squared_distance;
			neighbor_num = neighbor_search->knnSearch(current_point, k_neighbors_idx, squared_distance);

			for (int i = 0; i < neighbor_num; i++)
			{
				if (poi_queue[k_neighbors_idx[i]].result.zncc >= zncc_threshold)
				{
					pois_fit.push_back(poi_queue[k_neighbors_idx[i]]);
				}
			}
		}
		neighbor_num = (int)pois_fit.size();

		//terminate the procedure if there are insufficient neighbors for strain calculation
		if (neighbor_num >= neighbor_number_min)
		{
			//create matrices of displacments
			Eigen::VectorXf u_vector(neighbor_num);
			Eigen::VectorXf v_vector(neighbor_num);
			Eigen::VectorXf w_vector(neighbor_num);

			//construct coefficient matrix
			Eigen::MatrixXf coefficient_matrix = Eigen::MatrixXf::Zero(neighbor_num, 4);

			//fill coefficient matrix, u_vector, v_vector, and w_vector
			for (int i = 0; i < neighbor_num; i++)
			{
				coefficient_matrix(i, 0) = 1.f;
				coefficient_matrix(i, 1) = pois_fit[i].x - poi->x;
				coefficient_matrix(i, 2) = pois_fit[i].y - poi->y;
				coefficient_matrix(i, 3) = pois_fit[i].z - poi->z;

				u_vector(i) = pois_fit[i].deformation.u;
				v_vector(i) = pois_fit[i].deformation.v;
				w_vector(i) = pois_fit[i].deformation.w;
			}

			//solve the equations to obtain gradients of u, v, and w
			Eigen::VectorXf u_gradient = coefficient_matrix.colPivHouseholderQr().solve(u_vector);
			Eigen::VectorXf v_gradient = coefficient_matrix.colPivHouseholderQr().solve(v_vector);
			Eigen::VectorXf w_gradient = coefficient_matrix.colPivHouseholderQr().solve(w_vector);
			float ux = u_gradient(1, 0);
			float uy = u_gradient(2, 0);
			float uz = u_gradient(3, 0);
			float vx = v_gradient(1, 0);
			float vy = v_gradient(2, 0);
			float vz = v_gradient(3, 0);
			float wx = w_gradient(1, 0);
			float wy = w_gradient(2, 0);
			float wz = w_gradient(3, 0);

			if (approximation == 1)
			{
				//calculate the Cauchy strain and save them for output
				poi->strain.exx = ux;
				poi->strain.eyy = vy;
				poi->strain.ezz = wz;
				poi->strain.exy = 0.5f * (uy + vx);
				poi->strain.eyz = 0.5f * (vz + wy);
				poi->strain.ezx = 0.5f * (wx + uz);
			}
			if (approximation == 2)
			{
				poi->strain.exx = ux + 0.5f * (ux * ux + vx * vx + wx * wx);
				poi->strain.eyy = vy + 0.5f * (uy * uy + vy * vy + wy * wy);
				poi->strain.ezz = wz + 0.5f * (uz * uz + vz * vz + wz * wz);
				poi->strain.exy = 0.5f * (uy + vx + uy * ux + vy * vx + wy * wx);
				poi->strain.eyz = 0.5f * (vz + wy + uz * uy + vz * vy + wz * wy);
				poi->strain.ezx = 0.5f * (wx + uz + ux * uz + vx * vz + wx * wz);
			}
		}
	}

	void Strain::compute(std::vector<POI3D>& poi_queue)
	{
		int queue_length = (int)poi_queue.size();
#pragma omp parallel for
		for (int i = 0; i < queue_length; i++)
		{
			if (poi_queue[i].result.zncc >= zncc_threshold)
			{
				compute(&poi_queue[i], poi_queue);
			}
		}
	}


	bool sortByDistance(const PointIndex& p1, const PointIndex& p2)
	{
		return p1.distance < p2.distance;
	}

}//namespace opencorr
