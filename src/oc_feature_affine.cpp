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

#include <numeric>

#include "oc_feature_affine.h"

namespace opencorr
{
	//2D implementation
	NearestNeighbor* FeatureAffine2D::getInstance(int tid)
	{
		if (tid >= (int)instance_pool.size())
		{
			throw std::string("CPU thread ID over limit");
		}

		return instance_pool[tid];
	}

	FeatureAffine2D::FeatureAffine2D(int radius_x, int radius_y, int thread_number)
	{
		this->subset_radius_x = radius_x;
		this->subset_radius_y = radius_y;
		neighbor_search_radius = sqrt((float)(radius_x * radius_x + radius_y * radius_y));
		neighbor_number_min = 7;
		ransac_config.error_threshold = 1.5f;
		ransac_config.sample_mumber = 3;
		ransac_config.trial_number = 20;
		this->thread_number = thread_number;

		self_adaptive = false;
		subset_feature_min = 14;
		subset_radius_min = 10;

		for (int i = 0; i < thread_number; i++)
		{
			NearestNeighbor* instance = new NearestNeighbor();
			instance_pool.push_back(instance);
		}
	}

	FeatureAffine2D::~FeatureAffine2D()
	{
		for (auto& instance : instance_pool)
		{
			delete instance;
		}
		std::vector<NearestNeighbor*>().swap(instance_pool);
	}

	RansacConfig FeatureAffine2D::getRansacConfig() const
	{
		return ransac_config;
	}

	float FeatureAffine2D::getSearchRadius() const
	{
		return neighbor_search_radius;
	}

	int FeatureAffine2D::getMinNeighborNumber() const
	{
		return neighbor_number_min;
	}

	void FeatureAffine2D::setSearchParameters(float neighbor_search_radius, int neighbor_number_min)
	{
		this->neighbor_search_radius = neighbor_search_radius;
		this->neighbor_number_min = neighbor_number_min;
	}

	void FeatureAffine2D::setRansacConfig(RansacConfig ransac_config)
	{
		this->ransac_config = ransac_config;
	}

	void FeatureAffine2D::setSubsetAdjustment(int feature_min, int radius_min)
	{
		subset_feature_min = feature_min;
		subset_radius_min = radius_min;
	}

	void FeatureAffine2D::setKeypointPair(std::vector<Point2D>& ref_kp, std::vector<Point2D>& tar_kp)
	{
		this->ref_kp = ref_kp;
		this->tar_kp = tar_kp;
	}

	void FeatureAffine2D::prepare()
	{
#pragma omp parallel for
		for (int i = 0; i < thread_number; i++)
		{
			instance_pool[i]->clear();

			instance_pool[i]->assignPoints(ref_kp);
			instance_pool[i]->setSearchRadius(neighbor_search_radius);
			instance_pool[i]->setSearchK(neighbor_number_min);
			instance_pool[i]->constructKdTree();
		}
	}

	void FeatureAffine2D::compute(POI2D* poi)
	{
		//set instance w.r.t. thread id 
		NearestNeighbor* neighbor_search = getInstance(omp_get_thread_num());

		Point3D current_point(poi->x, poi->y, 0.f);
		std::vector<Point2D> ref_candidates, tar_candidates;

		int neighbor_num = 0;

		if (self_adaptive)
		{
			float x_min = ref_img->width;
			float x_max = -1.f;
			float y_min = ref_img->height;
			float y_max = -1.f;

			//search the neighbor keypoints in a region of given radius
			std::vector<uint32_t> k_neighbor_idx;
			std::vector<float> k_squared_distance;

			neighbor_num = neighbor_search->knnSearch(current_point, subset_feature_min, k_neighbor_idx, k_squared_distance);

			if (neighbor_num < ransac_config.sample_mumber)
			{
				poi->result.zncc = -1.f;
				return;
			}
			else
			{
				for (int i = 0; i < neighbor_num; i++)
				{
					ref_candidates.push_back(ref_kp[k_neighbor_idx[i]]);
					tar_candidates.push_back(tar_kp[k_neighbor_idx[i]]);

					x_min = ref_candidates[i].x < x_min ? ref_candidates[i].x : x_min;
					x_max = ref_candidates[i].x > x_max ? ref_candidates[i].x : x_max;
					y_min = ref_candidates[i].y < y_min ? ref_candidates[i].y : y_min;
					y_max = ref_candidates[i].y > y_max ? ref_candidates[i].y : y_max;
				}

				//modify POI and subset size
				if (poi->x >= x_min && poi->x <= x_max && poi->y >= y_min && poi->y <= y_max)
				{
					//if the POI is within the rectangle, set subset radius according to the farthest edges of rectangle
					poi->subset_radius.x = abs(x_max - poi->x) > abs(poi->x - x_min) ? (int)abs(x_max - poi->x) : (int)abs(poi->x - x_min);
					poi->subset_radius.y = abs(y_max - poi->y) > abs(poi->y - y_min) ? (int)abs(y_max - poi->y) : (int)abs(poi->y - y_min);
				}
				else
				{
					//if the POI is out of the rectangle, set the center of rectangle as the POI
					poi->x = (int)(0.5f * (x_max + x_min));
					poi->y = (int)(0.5f * (y_max + y_min));
					poi->subset_radius.x = (int)(0.5f * (x_max - x_min));
					poi->subset_radius.y = (int)(0.5f * (y_max - y_min));
				}

				//check if the radius is below the pre-defined minimum
				poi->subset_radius.x = poi->subset_radius.x < subset_radius_min ? subset_radius_min : poi->subset_radius.x;
				poi->subset_radius.y = poi->subset_radius.y < subset_radius_min ? subset_radius_min : poi->subset_radius.y;
			}
		}
		else
		{
			//search the neighbor keypoints in a region of given radius
			std::vector<nanoflann::ResultItem<uint32_t, float>> current_matches;
			neighbor_num = neighbor_search->radiusSearch(current_point, current_matches);

			if (neighbor_num < ransac_config.sample_mumber)
			{
				poi->result.zncc = -1.f;
				return;
			}
			else
			{
				ref_candidates.resize(neighbor_num);
				tar_candidates.resize(neighbor_num);

				if (neighbor_num >= neighbor_number_min)
				{
					for (int i = 0; i < neighbor_num; i++)
					{
						ref_candidates[i] = ref_kp[current_matches[i].first];
						tar_candidates[i] = tar_kp[current_matches[i].first];
					}
				}
				else //try KNN search if the obtained neighbor keypoints are not enough
				{
					std::vector<Point2D>().swap(ref_candidates);
					std::vector<Point2D>().swap(tar_candidates);

					std::vector<uint32_t> k_neighbor_idx;
					std::vector<float> k_squared_distance;

					neighbor_num = neighbor_search->knnSearch(current_point, k_neighbor_idx, k_squared_distance);

					ref_candidates.resize(neighbor_num);
					tar_candidates.resize(neighbor_num);
					for (int i = 0; i < neighbor_num; i++)
					{
						ref_candidates[i] = ref_kp[k_neighbor_idx[i]];
						tar_candidates[i] = tar_kp[k_neighbor_idx[i]];
					}
				}

				//use brutal force search for the last chance
				if (neighbor_num < neighbor_number_min)
				{
					std::vector<Point2D>().swap(ref_candidates);
					std::vector<Point2D>().swap(tar_candidates);

					//sort the kp queue in an ascending order of distance to the POI
					std::vector<KeypointIndex> ref_sorted_index;
					int queue_size = (int)ref_kp.size();
					for (int i = 0; i < queue_size; i++)
					{
						Point2D distance = ref_kp[i] - (Point2D)*poi;
						KeypointIndex current_kp_idx;
						current_kp_idx.kp_idx = i;
						current_kp_idx.distance = distance.vectorNorm();
						ref_sorted_index.push_back(current_kp_idx);
					}

					std::sort(ref_sorted_index.begin(), ref_sorted_index.end(), sortByDistance);

					//pick the keypoints for RANSAC procedure
					int i = 0;
					while (ref_sorted_index[i].distance < neighbor_search_radius || ref_candidates.size() < neighbor_number_min)
					{
						ref_candidates.push_back(ref_kp[ref_sorted_index[i].kp_idx]);
						tar_candidates.push_back(tar_kp[ref_sorted_index[i].kp_idx]);
						i++;
					}
					neighbor_num = (int)ref_candidates.size();
				}
			}
		}

		//convert global coordinates to the POI-centered local coordinates
		for (int i = 0; i < neighbor_num; i++)
		{
			ref_candidates[i] = ref_candidates[i] - (Point2D)*poi;
			tar_candidates[i] = tar_candidates[i] - (Point2D)*poi;
		}

		//RANSAC procedure
		std::vector<int> candidate_index(neighbor_num);
		std::iota(candidate_index.begin(), candidate_index.end(), 0); //initialize the candidate_index with the integers in ascending order

		int trial_counter = 0; //trial counter
		float location_mean_error;
		std::vector<int> max_set;

		//random number generator
		std::random_device rd;
		std::mt19937_64 gen64(rd());
		do
		{
			//randomly select samples
			std::shuffle(candidate_index.begin(), candidate_index.end(), gen64);

			Eigen::MatrixXf ref_neighbors(ransac_config.sample_mumber, 3);
			Eigen::MatrixXf tar_neighbors(ransac_config.sample_mumber, 3);
			for (int j = 0; j < ransac_config.sample_mumber; j++)
			{
				ref_neighbors(j, 0) = ref_candidates[candidate_index[j]].x;
				ref_neighbors(j, 1) = ref_candidates[candidate_index[j]].y;
				ref_neighbors(j, 2) = 1.f;
				tar_neighbors(j, 0) = tar_candidates[candidate_index[j]].x;
				tar_neighbors(j, 1) = tar_candidates[candidate_index[j]].y;
				tar_neighbors(j, 2) = 1.f;
			}
			//ref * affine = tar, thus affine is the permutation of affine matrix in the paper, where Ax=x'
			Eigen::Matrix3f affine_matrix = ref_neighbors.colPivHouseholderQr().solve(tar_neighbors);

			//concensus
			std::vector<int> trial_set;
			location_mean_error = 0;
			float delta_x, delta_y, estimation_error;
			for (int j = 0; j < neighbor_num; j++)
			{
				delta_x = (float)(ref_candidates[candidate_index[j]].x * affine_matrix(0, 0)
					+ ref_candidates[candidate_index[j]].y * affine_matrix(1, 0) + affine_matrix(2, 0))
					- tar_candidates[candidate_index[j]].x;
				delta_y = (float)(ref_candidates[candidate_index[j]].x * affine_matrix(0, 1)
					+ ref_candidates[candidate_index[j]].y * affine_matrix(1, 1) + affine_matrix(2, 1))
					- tar_candidates[candidate_index[j]].y;
				Point2D cur_point(delta_x, delta_y);
				estimation_error = cur_point.vectorNorm();
				//check if the error is acceptable, keep the "good" points
				if (estimation_error < ransac_config.error_threshold)
				{
					trial_set.push_back(candidate_index[j]);
					location_mean_error += estimation_error;
				}
			}
			//replace max_set with current trial_set if the latter is larger
			if (trial_set.size() > max_set.size())
			{
				max_set.assign(trial_set.begin(), trial_set.end());
			}

			trial_counter++;
			location_mean_error /= trial_set.size();
		} while (trial_counter < ransac_config.trial_number &&
			(max_set.size() < neighbor_number_min || location_mean_error > ransac_config.error_threshold / neighbor_number_min));

		//calculate affine matrix according to the results of concensus
		int max_set_size = (int)max_set.size();
		if (max_set_size < 3) //essential condition to solve the equation
		{
			poi->result.zncc = -2.f;
			return;
		}
		else
		{
			Eigen::MatrixXf ref_neighbors(max_set_size, 3);
			Eigen::MatrixXf tar_neighbors(max_set_size, 3);

			for (int i = 0; i < max_set_size; i++)
			{
				ref_neighbors(i, 0) = ref_candidates[max_set[i]].x;
				ref_neighbors(i, 1) = ref_candidates[max_set[i]].y;
				ref_neighbors(i, 2) = 1.f;
				tar_neighbors(i, 0) = tar_candidates[max_set[i]].x;
				tar_neighbors(i, 1) = tar_candidates[max_set[i]].y;
				tar_neighbors(i, 2) = 1.f;
			}
			//the method of least squares
			Eigen::Matrix3f affine_matrix = ref_neighbors.colPivHouseholderQr().solve(tar_neighbors);

			//calculate the 1st order deformation according to the equivalence between affine matrix and the 1st order shape function
			poi->deformation.u = affine_matrix(2, 0);
			poi->deformation.ux = affine_matrix(0, 0) - 1.f;
			poi->deformation.uy = affine_matrix(1, 0);
			poi->deformation.v = affine_matrix(2, 1);
			poi->deformation.vx = affine_matrix(0, 1);
			poi->deformation.vy = affine_matrix(1, 1) - 1.f;

			//store results of RANSAC procedure
			poi->result.iteration = (float)trial_counter;
			poi->result.feature = (float)max_set_size;

			poi->result.zncc = 0.f;
		}
	}

	void FeatureAffine2D::compute(std::vector<POI2D>& poi_queue)
	{
		int queue_length = (int)poi_queue.size();
#pragma omp parallel for
		for (int i = 0; i < queue_length; i++)
		{
			compute(&poi_queue[i]);
		}
	}



	//3D implementation
	NearestNeighbor* FeatureAffine3D::getInstance(int tid)
	{
		if (tid >= (int)instance_pool.size())
		{
			throw std::string("CPU thread ID over limit");
		}

		return instance_pool[tid];
	}

	FeatureAffine3D::FeatureAffine3D(int radius_x, int radius_y, int radius_z, int thread_number)
	{
		this->subset_radius_x = radius_x;
		this->subset_radius_y = radius_y;
		neighbor_search_radius = sqrt((float)(radius_x * radius_x + radius_y * radius_y + radius_z * radius_z));
		neighbor_number_min = 16;
		ransac_config.error_threshold = 3.2f;
		ransac_config.sample_mumber = 4;
		ransac_config.trial_number = 32;
		this->thread_number = thread_number;

		for (int i = 0; i < thread_number; i++)
		{
			NearestNeighbor* instance = new NearestNeighbor();
			instance_pool.push_back(instance);
		}
	}

	FeatureAffine3D::~FeatureAffine3D()
	{
		for (auto& instance : instance_pool)
		{
			delete instance;
		}
		std::vector<NearestNeighbor*>().swap(instance_pool);
	}

	RansacConfig FeatureAffine3D::getRansacConfig() const
	{
		return ransac_config;
	}

	float FeatureAffine3D::getSearchRadius() const
	{
		return neighbor_search_radius;
	}

	int FeatureAffine3D::getMinNeighborNumber() const
	{
		return neighbor_number_min;
	}

	void FeatureAffine3D::setSearchParameters(float neighbor_search_radius, int neighbor_number_min)
	{
		this->neighbor_search_radius = neighbor_search_radius;
		this->neighbor_number_min = neighbor_number_min;
	}

	void FeatureAffine3D::setRansacConfig(RansacConfig ransac_config)
	{
		this->ransac_config = ransac_config;
	}

	void FeatureAffine3D::setKeypointPair(std::vector<Point3D>& ref_kp, std::vector<Point3D>& tar_kp)
	{
		this->ref_kp = ref_kp;
		this->tar_kp = tar_kp;
	}

	void FeatureAffine3D::prepare()
	{
#pragma omp parallel for
		for (int i = 0; i < thread_number; i++)
		{
			instance_pool[i]->clear();

			instance_pool[i]->assignPoints(ref_kp);
			instance_pool[i]->setSearchRadius(neighbor_search_radius);
			instance_pool[i]->setSearchK(neighbor_number_min);
			instance_pool[i]->constructKdTree();
		}
	}

	void FeatureAffine3D::compute(POI3D* poi)
	{
		//set instance w.r.t. thread id 
		NearestNeighbor* neighbor_search = getInstance(omp_get_thread_num());

		Point3D current_point(poi->x, poi->y, poi->z);
		std::vector<Point3D> ref_candidates, tar_candidates;

		//search the neighbor keypoints in a region of given radius
		std::vector<nanoflann::ResultItem<uint32_t, float>> current_matches;
		int neighbor_num = neighbor_search->radiusSearch(current_point, current_matches);

		if (neighbor_num < ransac_config.sample_mumber)
		{
			poi->result.zncc = -1.f;
			return;
		}
		else
		{
			ref_candidates.resize(neighbor_num);
			tar_candidates.resize(neighbor_num);

			if (neighbor_num >= neighbor_number_min)
			{
				for (int i = 0; i < neighbor_num; i++)
				{
					ref_candidates[i] = ref_kp[current_matches[i].first];
					tar_candidates[i] = tar_kp[current_matches[i].first];
				}
			}
			else //try KNN search if the obtained neighbor keypoints are not enough
			{
				std::vector<Point3D>().swap(ref_candidates);
				std::vector<Point3D>().swap(tar_candidates);

				std::vector<uint32_t> k_neighbor_idx;
				std::vector<float> k_squared_distance;

				neighbor_num = neighbor_search->knnSearch(current_point, k_neighbor_idx, k_squared_distance);

				ref_candidates.resize(neighbor_num);
				tar_candidates.resize(neighbor_num);
				for (int i = 0; i < neighbor_num; i++)
				{
					ref_candidates[i] = ref_kp[k_neighbor_idx[i]];
					tar_candidates[i] = tar_kp[k_neighbor_idx[i]];
				}
			}

			//use brutal force search for the last chance
			if (neighbor_num < neighbor_number_min)
			{
				std::vector<Point3D>().swap(ref_candidates);
				std::vector<Point3D>().swap(tar_candidates);

				//sort the kp queue in an ascending order of distance to the POI
				std::vector<KeypointIndex> ref_sorted_index;
				int queue_size = (int)ref_kp.size();
				for (int i = 0; i < queue_size; i++)
				{
					Point3D distance = ref_kp[i] - (Point3D)*poi;
					KeypointIndex current_kp_idx;
					current_kp_idx.kp_idx = i;
					current_kp_idx.distance = distance.vectorNorm();
					ref_sorted_index.push_back(current_kp_idx);
				}

				std::sort(ref_sorted_index.begin(), ref_sorted_index.end(), sortByDistance);

				//pick the keypoints for RANSAC procedure
				int i = 0;
				while (ref_sorted_index[i].distance < neighbor_search_radius || ref_candidates.size() < neighbor_number_min)
				{
					ref_candidates.push_back(ref_kp[ref_sorted_index[i].kp_idx]);
					tar_candidates.push_back(tar_kp[ref_sorted_index[i].kp_idx]);
					i++;
				}
				neighbor_num = (int)ref_candidates.size();
			}

			//convert global coordinates to the POI-centered local coordinates
			for (int i = 0; i < neighbor_num; i++)
			{
				ref_candidates[i] = ref_candidates[i] - (Point3D)*poi;
				tar_candidates[i] = tar_candidates[i] - (Point3D)*poi;
			}

			//RANSAC procedure
			std::vector<int> candidate_index(neighbor_num);
			std::iota(candidate_index.begin(), candidate_index.end(), 0); //initialize the candidate_queue with value in ascending order

			int trial_counter = 0; //trial counter
			float location_mean_error;
			std::vector<int> max_set;

			//random number generator
			std::random_device rd;
			std::mt19937_64 gen64(rd());
			do
			{
				//randomly select samples
				std::shuffle(candidate_index.begin(), candidate_index.end(), gen64);

				Eigen::MatrixXf tar_neighbors(ransac_config.sample_mumber, 4);
				Eigen::MatrixXf ref_neighbors(ransac_config.sample_mumber, 4);
				for (int j = 0; j < ransac_config.sample_mumber; j++) {
					tar_neighbors(j, 0) = tar_candidates[candidate_index[j]].x;
					tar_neighbors(j, 1) = tar_candidates[candidate_index[j]].y;
					tar_neighbors(j, 2) = tar_candidates[candidate_index[j]].z;
					tar_neighbors(j, 3) = 1.f;

					ref_neighbors(j, 0) = ref_candidates[candidate_index[j]].x;
					ref_neighbors(j, 1) = ref_candidates[candidate_index[j]].y;
					ref_neighbors(j, 2) = ref_candidates[candidate_index[j]].z;
					ref_neighbors(j, 3) = 1.f;
				}
				//ref * affine = tar, thus affine is the permutation of affine matrix in the paper, where Ax=x'
				Eigen::Matrix4f affine_matrix = ref_neighbors.colPivHouseholderQr().solve(tar_neighbors);

				//concensus
				std::vector<int> trial_set;
				location_mean_error = 0;
				float delta_x, delta_y, delta_z, estimation_error;
				for (int j = 0; j < neighbor_num; j++) {
					delta_x = (float)(ref_candidates[candidate_index[j]].x * affine_matrix(0, 0)
						+ ref_candidates[candidate_index[j]].y * affine_matrix(1, 0)
						+ ref_candidates[candidate_index[j]].z * affine_matrix(2, 0) + affine_matrix(3, 0))
						- tar_candidates[candidate_index[j]].x;
					delta_y = (float)(ref_candidates[candidate_index[j]].x * affine_matrix(0, 1)
						+ ref_candidates[candidate_index[j]].y * affine_matrix(1, 1)
						+ ref_candidates[candidate_index[j]].z * affine_matrix(2, 1) + affine_matrix(3, 1))
						- tar_candidates[candidate_index[j]].y;
					delta_z = (float)(ref_candidates[candidate_index[j]].x * affine_matrix(0, 2)
						+ ref_candidates[candidate_index[j]].y * affine_matrix(1, 2)
						+ ref_candidates[candidate_index[j]].z * affine_matrix(2, 2) + affine_matrix(3, 2))
						- tar_candidates[candidate_index[j]].z;
					Point3D point(delta_x, delta_y, delta_z);
					estimation_error = point.vectorNorm();
					//check if the error is acceptable, keep the "good" points
					if (estimation_error < ransac_config.error_threshold) {
						trial_set.push_back(candidate_index[j]);
						location_mean_error += estimation_error;
					}
				}
				//replace max_set with current trial_set if the latter is larger
				if (trial_set.size() > max_set.size()) {
					max_set.assign(trial_set.begin(), trial_set.end());
				}
				trial_counter++;
				location_mean_error /= trial_set.size();
			} while (trial_counter < ransac_config.trial_number &&
				(max_set.size() < neighbor_number_min || location_mean_error > ransac_config.error_threshold / neighbor_number_min));

			//calculate affine matrix according to the results of concensus
			int max_set_size = (int)max_set.size();
			if (max_set_size < 4) //essential condition to solve the equation
			{
				poi->result.zncc = -2.f;
				return;
			}

			Eigen::MatrixXf tar_neighbors(max_set_size, 4);
			Eigen::MatrixXf ref_neighbors(max_set_size, 4);

			for (int i = 0; i < max_set_size; i++)
			{
				ref_neighbors(i, 0) = ref_candidates[max_set[i]].x;
				ref_neighbors(i, 1) = ref_candidates[max_set[i]].y;
				ref_neighbors(i, 2) = ref_candidates[max_set[i]].z;
				ref_neighbors(i, 3) = 1.f;
				tar_neighbors(i, 0) = tar_candidates[max_set[i]].x;
				tar_neighbors(i, 1) = tar_candidates[max_set[i]].y;
				tar_neighbors(i, 2) = tar_candidates[max_set[i]].z;
				tar_neighbors(i, 3) = 1.f;
			}
			//the method of least squares
			Eigen::Matrix4f affine_matrix = ref_neighbors.colPivHouseholderQr().solve(tar_neighbors);

			//calculate the 1st order deformation according to the equivalence between affine matrix and 1st order shape function
			poi->deformation.u = affine_matrix(3, 0);
			poi->deformation.ux = affine_matrix(0, 0) - 1.f;
			poi->deformation.uy = affine_matrix(1, 0);
			poi->deformation.uz = affine_matrix(2, 0);
			poi->deformation.v = affine_matrix(3, 1);
			poi->deformation.vx = affine_matrix(0, 1);
			poi->deformation.vy = affine_matrix(1, 1) - 1.f;
			poi->deformation.vz = affine_matrix(2, 1);
			poi->deformation.w = affine_matrix(3, 2);
			poi->deformation.wx = affine_matrix(0, 2);
			poi->deformation.wy = affine_matrix(1, 2);
			poi->deformation.wy = affine_matrix(2, 2) - 1.f;

			//store results of RANSAC procedure
			poi->result.iteration = (float)trial_counter;
			poi->result.feature = (float)max_set_size;

			poi->result.zncc = 0.f;
		}
	}

	void FeatureAffine3D::compute(std::vector<POI3D>& poi_queue)
	{
		int queue_length = (int)poi_queue.size();
#pragma omp parallel for
		for (int i = 0; i < queue_length; i++)
		{
			compute(&poi_queue[i]);
		}
	}

}//namespace opencorr
