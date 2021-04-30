/*
 * This file is part of OpenCorr, an open source C++ library for
 * study and development of 2D, 3D/stereo and volumetric
 * digital image correlation.
 *
 * Copyright (C) 2021, Zhenyu Jiang <zhenyujiang@scut.edu.cn>
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * More information about OpenCorr can be found at https://www.opencorr.org/
 */

#include <vector>
#include <algorithm>
#include <omp.h>

#include "oc_epipolar_search.h"

namespace opencorr
{
	EpipolarSearch::EpipolarSearch(Calibration& left_cam, Calibration& right_cam, int thread_number) {
		this->left_cam = left_cam;
		this->right_cam = right_cam;
		this->thread_number = thread_number;
	}

	EpipolarSearch::~EpipolarSearch() {
		delete this->icgn1;
	}

	int EpipolarSearch::getSearchRadius() const {
		return this->search_radius;
	}

	int EpipolarSearch::getSearchStep() const {
		return this->search_step;
	}

	void EpipolarSearch::setSearch(int search_radius, int search_step) {
		if (search_step >= search_radius) {
			std::cerr << "search radius is less than search step";
			exit(1);
		}
		this->search_radius = search_radius;
		this->search_step = search_step;
	}

	void EpipolarSearch::setICGN(int subset_radius_x, int subset_radius_y, float conv_criterion, float stop_condition) {
		this->icgn_sr_x = subset_radius_x;
		this->icgn_sr_y = subset_radius_y;
		this->icgn_conv = conv_criterion;
		this->icgn_stop = stop_condition;
		this->icgn1->setImages(*this->ref_img, *this->tar_img);
	}

	void EpipolarSearch::updateCameraParameters(Calibration& left_cam, Calibration& right_cam) {
		this->left_cam = left_cam;
		this->right_cam = right_cam;
	}

	void EpipolarSearch::updateFundementalMatrix() {
		Calibration cam_r = this->right_cam;
		//creat transposed inverse intrinsic matrix of right camera
		Eigen::Matrix3f right_invK_t = cam_r.intrinsic_matrix.inverse().transpose();

		//create an anti-symmetric matrix of translation vector of right camera
		Eigen::Matrix3f right_t_antisymmetric;
		right_t_antisymmetric << 0, -cam_r.translation_vector(2), cam_r.translation_vector(1),
			cam_r.translation_vector(2), 0, -cam_r.translation_vector(0),
			-cam_r.translation_vector(1), cam_r.translation_vector(0), 0;

		//create essential matrix of right camera
		Eigen::Matrix3f right_E = right_t_antisymmetric * cam_r.rotation_matrix;

		//creat inversed intrinsic matrix of left camera
		Eigen::Matrix3f left_K = this->left_cam.intrinsic_matrix.inverse();

		this->fundamental_matrix = right_invK_t * right_E * left_K;
	}

	void EpipolarSearch::prepare() {
		this->left_cam.updateIntrinsicMatrix();
		this->left_cam.updateRotationMatrix();
		this->left_cam.updateTranslationVector();
		this->left_cam.updateProjectionMatrix();
		this->right_cam.updateIntrinsicMatrix();
		this->right_cam.updateRotationMatrix();
		this->right_cam.updateTranslationVector();
		this->right_cam.updateProjectionMatrix();
		this->updateFundementalMatrix();

		this->icgn1 = new ICGN2D1(this->icgn_sr_x, this->icgn_sr_y,
			this->icgn_conv, this->icgn_stop, this->thread_number);
		this->icgn1->prepare();
	}

	void EpipolarSearch::compute(POI2D* POI)	{
		//convert locatoin of left POI to a vector
		Eigen::Vector3f left_vector;
		left_vector << (POI->x + POI->deformation.u), (POI->y + POI->deformation.v), 1;

		//get the projection of POI in left view on the epipolar line in right view
		Eigen::Vector3f right_epipolar = this->fundamental_matrix * left_vector;
		float line_slope = -right_epipolar(0) / right_epipolar(1);
		float line_intercept = -right_epipolar(2) / right_epipolar(1);
		int x_right = (int)((line_slope * (POI->y - line_intercept) + POI->x) / (line_slope * line_slope + 1));
		int y_right = (int)(line_slope * x_right + line_intercept);

		std::vector<POI2D> POI_candidates;
		POI2D current_POI(POI->x, POI->y);
		current_POI.deformation.u = x_right - POI->x;
		current_POI.deformation.v = y_right - POI->y;
		POI_candidates.push_back(current_POI);

		int x_trial, y_trial;
		for (int i = this->search_step; i < this->search_radius; i += this->search_step) {
			x_trial = x_right + i;
			y_trial = (int)(line_slope * x_trial + line_intercept);
			current_POI.deformation.u = x_trial - POI->x;
			current_POI.deformation.v = y_trial - POI->y;
			POI_candidates.push_back(current_POI);

			x_trial = x_right - i;
			y_trial = (int)(line_slope * x_trial + line_intercept);
			current_POI.deformation.u = x_trial - POI->x;
			current_POI.deformation.v = y_trial - POI->y;
			POI_candidates.push_back(current_POI);
		}

		this->icgn1->compute(POI_candidates);

		std::sort(POI_candidates.begin(), POI_candidates.end(), sortByZNCC);

		POI->result = POI_candidates[0].result;
		POI->result = POI_candidates[0].result;
	}

	void EpipolarSearch::compute(std::vector<POI2D>& POI_queue) {
#pragma omp parallel for
		for (int i = 0; i < POI_queue.size(); ++i) {
			this->compute(&POI_queue[i]);
		}
	}

	bool sortByZNCC(const POI2D& p1, const POI2D& p2) {
		return p1.result.ZNCC > p2.result.ZNCC;
	}
}//namespace opencorr