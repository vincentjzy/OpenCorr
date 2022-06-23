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

#include <algorithm>
#include <math.h>
#include <numeric>
#include <omp.h>
#include <random>

#include "oc_feature_affine.h"

namespace opencorr
{
	//2D case----------------------------------------------------------------------------------
	FeatureAffine2D::FeatureAffine2D(int radius_x, int radius_y) {
		this->subset_radius_x = radius_x;
		this->subset_radius_y = radius_y;
		neighbor_search_radius = sqrt((float)(radius_x * radius_x + radius_y * radius_y));
		min_neighbor_num = 14;
		ransac_config.error_threshold = 1.5f;
		ransac_config.sample_mumber = 5;
		ransac_config.trial_number = 10;

		neighbor_search = new NearestNeighbor();
	}

	FeatureAffine2D::~FeatureAffine2D() {
		if (neighbor_search != nullptr) {
			delete neighbor_search;
		}
	}

	void FeatureAffine2D::prepare() {
		neighbor_search->assignPoints(ref_kp);
		neighbor_search->setSearchRadius(neighbor_search_radius);
		neighbor_search->setSearchK(min_neighbor_num);
		neighbor_search->constructKdTree();
	}

	void FeatureAffine2D::compute(POI2D* poi) {
		Point3D current_point(poi->x, poi->y, 0.f);
		std::vector<Point2D> ref_candidates, tar_candidates;

		//search the neighbor keypoints in a region of given radius
		std::vector<std::pair<uint32_t, float>> current_matches;
		int num_candidate = neighbor_search->radiusSearch(current_point, current_matches);

		ref_candidates.resize(num_candidate);
		tar_candidates.resize(num_candidate);
		if (num_candidate >= min_neighbor_num) {
			for (int i = 0; i < num_candidate; i++) {
				ref_candidates[i] = ref_kp[current_matches[i].first];
				tar_candidates[i] = tar_kp[current_matches[i].first];
			}
		}
		//try KNN search if the obtained neighbor keypoints are not enough
		else {
			std::vector<Point2D>().swap(ref_candidates);
			std::vector<Point2D>().swap(tar_candidates);

			std::vector<uint32_t> k_neighbors_idx;
			std::vector<float> kp_squared_distance;

			num_candidate = neighbor_search->knnSearch(current_point, k_neighbors_idx, kp_squared_distance);

			ref_candidates.resize(num_candidate);
			tar_candidates.resize(num_candidate);
			for (int i = 0; i < num_candidate; i++) {
				ref_candidates[i] = ref_kp[k_neighbors_idx[i]];
				tar_candidates[i] = tar_kp[k_neighbors_idx[i]];
			}
		}

		//use the brutal force search for those POIs with too few keypoints nearby
		if (num_candidate < min_neighbor_num) {
			std::vector<Point2D>().swap(ref_candidates);
			std::vector<Point2D>().swap(tar_candidates);

			//sort the kp queue in an ascending order of distance to the POI
			std::vector<KeypointIndex> ref_sorted_index;
			int queue_size = (int)ref_kp.size();
			for (int i = 0; i < queue_size; ++i) {
				Point2D distance = ref_kp[i] - (Point2D)*poi;
				KeypointIndex current_kp_idx;
				current_kp_idx.idx_in_queue = i;
				current_kp_idx.distance_to_poi = distance.vectorNorm();
				ref_sorted_index.push_back(current_kp_idx);
			}

			std::sort(ref_sorted_index.begin(), ref_sorted_index.end(), sortByDistance);

			//pick the keypoints for RANSAC procedure
			int i = 0;
			while (ref_sorted_index[i].distance_to_poi < neighbor_search_radius || ref_candidates.size() < min_neighbor_num) {
				ref_candidates.push_back(ref_kp[ref_sorted_index[i].idx_in_queue]);
				tar_candidates.push_back(tar_kp[ref_sorted_index[i].idx_in_queue]);
				i++;
			}
			num_candidate = (int)ref_candidates.size();
		}

		//convert global coordinates to the POI-centered local coordinates
		for (int i = 0; i < num_candidate; ++i) {
			ref_candidates[i] = ref_candidates[i] - (Point2D)*poi;
			tar_candidates[i] = tar_candidates[i] - (Point2D)*poi;
		}

		//RANSAC procedure, refer to Yang et al. Opt Laser Eng (2020), 127, 105964 for details
		std::vector<int> candidate_index(num_candidate);
		std::iota(candidate_index.begin(), candidate_index.end(), 0);

		int trial_number = 0;
		float location_mean_error;
		std::vector<int> max_set;

		//random number generator
		std::random_device rd;
		std::mt19937_64 gen64(rd());
		do {
			//randomly select samples
			std::shuffle(candidate_index.begin(), candidate_index.end(), gen64);

			Eigen::MatrixXf ref_neighbors(ransac_config.sample_mumber, 3);
			Eigen::MatrixXf tar_neighbors(ransac_config.sample_mumber, 3);
			for (int j = 0; j < ransac_config.sample_mumber; j++) {
				ref_neighbors(j, 0) = ref_candidates[candidate_index[j]].x;
				ref_neighbors(j, 1) = ref_candidates[candidate_index[j]].y;
				ref_neighbors(j, 2) = 1.f;
				tar_neighbors(j, 0) = tar_candidates[candidate_index[j]].x;
				tar_neighbors(j, 1) = tar_candidates[candidate_index[j]].y;
				tar_neighbors(j, 2) = 1.f;
			}
			Eigen::Matrix3f affine_matrix = ref_neighbors.colPivHouseholderQr().solve(tar_neighbors);

			//concensus
			std::vector<int> trial_set;
			location_mean_error = 0;
			float px, py, estimated_error;
			for (int j = 0; j < num_candidate; j++) {
				px = (float)(ref_candidates[candidate_index[j]].x * affine_matrix(0, 0)
					+ ref_candidates[candidate_index[j]].y * affine_matrix(1, 0) + affine_matrix(2, 0))
					- tar_candidates[candidate_index[j]].x;
				py = (float)(ref_candidates[candidate_index[j]].x * affine_matrix(0, 1)
					+ ref_candidates[candidate_index[j]].y * affine_matrix(1, 1) + affine_matrix(2, 1))
					- tar_candidates[candidate_index[j]].y;
				Point2D point(px, py);
				estimated_error = point.vectorNorm();
				if (estimated_error < ransac_config.error_threshold) {
					trial_set.push_back(candidate_index[j]);
					location_mean_error += estimated_error;
				}
			}
			if (trial_set.size() > max_set.size()) {
				max_set.assign(trial_set.begin(), trial_set.end());
			}
			trial_number++;
			location_mean_error /= trial_set.size();
		} while (trial_number < ransac_config.trial_number
			&& (max_set.size() < min_neighbor_num
				|| location_mean_error > ransac_config.error_threshold / min_neighbor_num));

		//calculate affine matrix according to the results of concensus
		int max_set_size = (int)max_set.size();
		if (max_set_size < 3) {
//			std::cerr << "Insufficient available neighbor keypoints around: " << (Point2D)*poi << std::endl;
			poi->result.zncc = 0;
		}

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
		Eigen::Matrix3f affine_matrix = ref_neighbors.colPivHouseholderQr().solve(tar_neighbors);

		//calculate the 1st order deformation according to the equivalence between affine matrix and 1st order shape function
		poi->deformation.u = affine_matrix(2, 0);
		poi->deformation.ux = affine_matrix(0, 0) - 1.f;
		poi->deformation.uy = affine_matrix(1, 0);
		poi->deformation.v = affine_matrix(2, 1);
		poi->deformation.vx = affine_matrix(0, 1);
		poi->deformation.vy = affine_matrix(1, 1) - 1.f;

		//store results of RANSAC procedure
		poi->result.iteration = (float)trial_number;
		poi->result.feature = (float)max_set_size;
	}

	void FeatureAffine2D::compute(std::vector<POI2D>& poi_queue) {
#pragma omp parallel for
		for (int i = 0; i < poi_queue.size(); ++i) {
			compute(&poi_queue[i]);
		}
	}

	RansacConfig FeatureAffine2D::getRansacConfig() const {
		return ransac_config;
	}

	float FeatureAffine2D::getSearchRadius() const {
		return neighbor_search_radius;
	}

	int FeatureAffine2D::getMinNeighborNumber() const {
		return min_neighbor_num;
	}

	void FeatureAffine2D::setSearchParameters(float neighbor_search_radius, int min_neighbor_num) {
		this->neighbor_search_radius = neighbor_search_radius;
		this->min_neighbor_num = min_neighbor_num;
	}

	void FeatureAffine2D::setRansacConfig(RansacConfig ransac_config) {
		this->ransac_config = ransac_config;
	}

	void FeatureAffine2D::setKeypointPair(std::vector<Point2D>& ref_kp, std::vector<Point2D>& tar_kp) {
		this->ref_kp = ref_kp;
		this->tar_kp = tar_kp;
	}

	//3D case----------------------------------------------------------------------------------
	FeatureAffine3D::FeatureAffine3D(int radius_x, int radius_y, int radius_z) {
		this->subset_radius_x = radius_x;
		this->subset_radius_y = radius_y;
		neighbor_search_radius = sqrt((float)(radius_x * radius_x + radius_y * radius_y + radius_z * radius_z));
		min_neighbor_num = 16;
		ransac_config.error_threshold = 3.2f;
		ransac_config.sample_mumber = 6;
		ransac_config.trial_number = 30;
	}

	FeatureAffine3D::~FeatureAffine3D() {
	}

	void FeatureAffine3D::prepare() {
	}

	void FeatureAffine3D::compute(POI3D* poi) {
		//brutal force search for neighbor keypoints
		std::vector<KeypointIndex> ref_sorted_index;
		std::vector<Point3D> tar_candidates, ref_candidates;

		//sort the kp queue in a descendent order of distance to the POI
		int queue_size = (int)ref_kp.size();
		for (int i = 0; i < queue_size; ++i) {
			Point3D distance = ref_kp[i] - (Point3D)*poi;
			KeypointIndex current_kp_idx;
			current_kp_idx.idx_in_queue = i;
			current_kp_idx.distance_to_poi = distance.vectorNorm();
			ref_sorted_index.push_back(current_kp_idx);
		}

		std::sort(ref_sorted_index.begin(), ref_sorted_index.end(), sortByDistance);

		//pick the keypoints for RANSAC procedure
		int i = 0;
		while (ref_sorted_index[i].distance_to_poi < neighbor_search_radius || ref_candidates.size() <= min_neighbor_num) {
			tar_candidates.push_back(tar_kp[ref_sorted_index[i].idx_in_queue]);
			ref_candidates.push_back(ref_kp[ref_sorted_index[i].idx_in_queue]);
			i++;
		}

		//convert global coordinates to the POI-centered local coordinates
		int candidate_number = (int)ref_candidates.size();
		for (int i = 0; i < candidate_number; ++i) {
			tar_candidates[i] = tar_candidates[i] - (Point3D)*poi;
			ref_candidates[i] = ref_candidates[i] - (Point3D)*poi;
		}

		//RANSAC procedure, refer to Yang et al. Opt Laser Eng (2020), 127, 105964 for details
		std::vector<int> candidate_index(candidate_number);
		std::iota(candidate_index.begin(), candidate_index.end(), 0);

		int trial_number = 0;
		float location_mean_error;
		std::vector<int> max_set;

		//random number generator
		std::random_device rd;
		std::mt19937_64 gen64(rd());
		do {
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
			Eigen::Matrix4f affine_matrix = ref_neighbors.colPivHouseholderQr().solve(tar_neighbors);

			//concensus
			std::vector<int> trial_set;
			location_mean_error = 0;
			float px, py, pz, estimated_error;
			for (int j = 0; j < candidate_number; j++) {
				px = (float)(ref_candidates[candidate_index[j]].x * affine_matrix(0, 0)
					+ ref_candidates[candidate_index[j]].y * affine_matrix(1, 0)
					+ ref_candidates[candidate_index[j]].z * affine_matrix(2, 0) + affine_matrix(3, 0))
					- tar_candidates[candidate_index[j]].x;
				py = (float)(ref_candidates[candidate_index[j]].x * affine_matrix(0, 1)
					+ ref_candidates[candidate_index[j]].y * affine_matrix(1, 1)
					+ ref_candidates[candidate_index[j]].z * affine_matrix(2, 1) + affine_matrix(3, 1))
					- tar_candidates[candidate_index[j]].y;
				pz = (float)(ref_candidates[candidate_index[j]].x * affine_matrix(0, 2)
					+ ref_candidates[candidate_index[j]].y * affine_matrix(1, 2)
					+ ref_candidates[candidate_index[j]].z * affine_matrix(2, 2) + affine_matrix(3, 2))
					- tar_candidates[candidate_index[j]].z;
				Point3D point(px, py, pz);
				estimated_error = point.vectorNorm();
				if (estimated_error < ransac_config.error_threshold) {
					trial_set.push_back(candidate_index[j]);
					location_mean_error += estimated_error;
				}
			}
			if (trial_set.size() > max_set.size()) {
				max_set.assign(trial_set.begin(), trial_set.end());
			}
			trial_number++;
			location_mean_error /= trial_set.size();
		} while (trial_number < ransac_config.trial_number
			&& (max_set.size() < min_neighbor_num
				|| location_mean_error > ransac_config.error_threshold / min_neighbor_num));

		//calculate affine matrix according to the results of concensus
		int max_set_size = (int)max_set.size();
		if (max_set_size < 4) {
			std::cerr << "Insufficient available neighbor keypoints around: " << (Point3D)*poi << std::endl;
			poi->result.zncc = 0;
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
		poi->result.iteration = (float)trial_number;
		poi->result.feature = (float)max_set_size;
	}

	void FeatureAffine3D::compute(std::vector<POI3D>& poi_queue) {
		//#pragma omp parallel for
		for (int i = 0; i < poi_queue.size(); ++i) {
			compute(&poi_queue[i]);
		}
	}

	RansacConfig FeatureAffine3D::getRansacConfig() const {
		return ransac_config;
	}

	float FeatureAffine3D::getSearchRadius() const {
		return neighbor_search_radius;
	}

	int FeatureAffine3D::getMinNeighborNumber() const {
		return min_neighbor_num;
	}

	void FeatureAffine3D::setSearchParameters(float neighbor_search_radius, int min_neighbor_num) {
		this->neighbor_search_radius = neighbor_search_radius;
		this->min_neighbor_num = min_neighbor_num;
	}

	void FeatureAffine3D::setRansacConfig(RansacConfig ransac_config) {
		this->ransac_config = ransac_config;
	}

	void FeatureAffine3D::setKeypointPair(std::vector<Point3D>& ref_kp, std::vector<Point3D>& tar_kp) {
		this->ref_kp = ref_kp;
		this->tar_kp = tar_kp;
	}

	bool sortByDistance(const KeypointIndex& kp1, const KeypointIndex& kp2) {
		return kp1.distance_to_poi < kp2.distance_to_poi;
	}

}//namespace opencorr
