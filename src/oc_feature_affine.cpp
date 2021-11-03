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

#include "oc_feature_affine.h"
#include <random>
#include <omp.h>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <numeric>

namespace opencorr
{

	FeatureAffine2D::FeatureAffine2D(int subset_radius_x, int subset_radius_y) {
		this->subset_radius_x = subset_radius_x;
		this->subset_radius_y = subset_radius_y;
		neighbor_search_radius = sqrtf(float(subset_radius_x * subset_radius_x + subset_radius_y * subset_radius_y));
		min_neighbor_num = 14;
		ransac_config.error_threshold = 1.5f;
		ransac_config.sample_mumber = 5;
		ransac_config.trial_number = 10;
	}

	FeatureAffine2D::~FeatureAffine2D() {
	}

	void FeatureAffine2D::prepare() {
	}

	void FeatureAffine2D::compute(POI2D* poi) {
		//brutal force search for neighbor keypoints
		std::vector<KeypointIndex> ref_sorted_index;
		std::vector<Point2D> tar_candidates, ref_candidates;

		//sort the kp queue in a descendent order of distance to the POI
		int queue_size = (int)ref_kp.size();
		for (int i = 0; i < queue_size; ++i) {
			Point2D distance = ref_kp[i] - (Point2D)*poi;
			KeypointIndex current_kp_idx;
			current_kp_idx.index_in_queue = i;
			current_kp_idx.distance_to_poi = distance.vectorNorm();
			ref_sorted_index.push_back(current_kp_idx);
		}

		std::sort(ref_sorted_index.begin(), ref_sorted_index.end(), sortByDistance);

		//pick the keypoints for RANSAC procedure
		int i = 0;
		while (ref_sorted_index[i].distance_to_poi < neighbor_search_radius || ref_candidates.size() <= min_neighbor_num) {
			tar_candidates.push_back(tar_kp[ref_sorted_index[i].index_in_queue]);
			ref_candidates.push_back(ref_kp[ref_sorted_index[i].index_in_queue]);
			i++;
		}

		//convert global coordinates to the POI-centered local coordinates
		int candidate_number = (int)ref_candidates.size();
		for (int i = 0; i < candidate_number; ++i) {
			tar_candidates[i] = tar_candidates[i] - (Point2D)*poi;
			ref_candidates[i] = ref_candidates[i] - (Point2D)*poi;
		}

		//RANSAC procedure, refer to Yang et al. Opt Laser Eng (2020), 127, 105964 for details
		std::vector<int> candidate_index(candidate_number);
		std::iota(candidate_index.begin(), candidate_index.end(), 0);

		int trial_number = 0;
		float location_mean_error;
		std::vector<int> max_set;
		do {
			//randomly select samples
			std::random_shuffle(candidate_index.begin(), candidate_index.end());

			Eigen::MatrixXf tar_neighbors(ransac_config.sample_mumber, 3);
			Eigen::MatrixXf ref_neighbors(ransac_config.sample_mumber, 3);
			for (int j = 0; j < ransac_config.sample_mumber; j++) {
				tar_neighbors(j, 0) = tar_candidates[candidate_index[j]].x;
				tar_neighbors(j, 1) = tar_candidates[candidate_index[j]].y;
				tar_neighbors(j, 2) = 1.f;

				ref_neighbors(j, 0) = ref_candidates[candidate_index[j]].x;
				ref_neighbors(j, 1) = ref_candidates[candidate_index[j]].y;
				ref_neighbors(j, 2) = 1.f;
			}
			Eigen::Matrix3f affine_matrix = ref_neighbors.colPivHouseholderQr().solve(tar_neighbors);

			//concensus
			std::vector<int> trial_set;
			location_mean_error = 0;
			float px, py, estimated_error;
			for (int j = 0; j < candidate_number; j++) {
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
			std::cerr << "Insufficient available neighbor keypoints around: " << (Point2D)*poi << std::endl;
			poi->result.zncc = 0;
		}

		Eigen::MatrixXf tar_neighbors(max_set_size, 3);
		Eigen::MatrixXf ref_neighbors(max_set_size, 3);

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

	RANSACconfig FeatureAffine2D::getRANSAC() const {
		return ransac_config;
	}

	float FeatureAffine2D::getSearchRadius() const {
		return neighbor_search_radius;
	}

	int FeatureAffine2D::getMinimumNeighborNumber() const {
		return min_neighbor_num;
	}

	void FeatureAffine2D::setSearchParameters(float neighbor_search_radius, int min_neighbor_num) {
		this->neighbor_search_radius = neighbor_search_radius;
		this->min_neighbor_num = min_neighbor_num;
	}

	void FeatureAffine2D::setRANSAC(RANSACconfig ransac_config) {
		this->ransac_config = ransac_config;
	}

	void FeatureAffine2D::setKeypointPair(std::vector<Point2D>& ref_kp, std::vector<Point2D>& tar_kp) {
		this->ref_kp = ref_kp;
		this->tar_kp = tar_kp;
	}

	bool sortByDistance(const KeypointIndex& kp1, const KeypointIndex& kp2) {
		return kp1.distance_to_poi < kp2.distance_to_poi;
	}

}//namespace opencorr
