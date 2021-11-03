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

#include <vector>
#include <algorithm>
#include <omp.h>

#include "oc_epipolar_search.h"

namespace opencorr
{
	EpipolarSearch::EpipolarSearch(Calibration& view1_cam, Calibration& view2_cam, int thread_number) {
		this->view1_cam = view1_cam;
		this->view2_cam = view2_cam;
		this->thread_number = thread_number;
	}

	EpipolarSearch::~EpipolarSearch() {
		if (icgn1 != nullptr)
			destoryICGN();
	}

	int EpipolarSearch::getSearchRadius() const {
		return search_radius;
	}

	int EpipolarSearch::getSearchStep() const {
		return search_step;
	}

	Point2D EpipolarSearch::getParallax() const {
		return parallax;
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
		icgn_sr_x = subset_radius_x;
		icgn_sr_y = subset_radius_y;
		icgn_conv = conv_criterion;
		icgn_stop = stop_condition;

		icgn1 = new ICGN2D1(icgn_sr_x, icgn_sr_y, icgn_conv, icgn_stop, thread_number);
	}

	void EpipolarSearch::destoryICGN() {
		delete icgn1;
	}

	void EpipolarSearch::setParallax(Point2D parallax) {
		this->parallax = parallax;
	}

	void EpipolarSearch::updateCameras(Calibration& view1_cam, Calibration& view2_cam) {
		this->view1_cam = view1_cam;
		this->view2_cam = view2_cam;
	}

	void EpipolarSearch::updateFundementalMatrix() {
		Calibration cam_r = view2_cam;
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
		Eigen::Matrix3f left_K = view1_cam.intrinsic_matrix.inverse();

		fundamental_matrix = right_invK_t * right_E * left_K;
	}

	void EpipolarSearch::prepare() {
		view1_cam.updateIntrinsicMatrix();
		view1_cam.updateRotationMatrix();
		view1_cam.updateTranslationVector();
		view1_cam.updateProjectionMatrix();
		view2_cam.updateIntrinsicMatrix();
		view2_cam.updateRotationMatrix();
		view2_cam.updateTranslationVector();
		view2_cam.updateProjectionMatrix();
		updateFundementalMatrix();

		icgn1->setImages(*ref_img, *tar_img);
		icgn1->prepare();
	}

	void EpipolarSearch::compute(POI2D* poi) {
		//convert locatoin of left POI to a vector
		Eigen::Vector3f left_vector;
		left_vector << (poi->x + poi->deformation.u), (poi->y + poi->deformation.v), 1;

		//get the projection of POI in left view on the epipolar line in right view
		Eigen::Vector3f right_epipolar = fundamental_matrix * left_vector;
		float line_slope = -right_epipolar(0) / right_epipolar(1);
		float line_intercept = -right_epipolar(2) / right_epipolar(1);
		int x_right = (int)((line_slope * (poi->y + poi->deformation.v + parallax.y - line_intercept) + poi->x + poi->deformation.u + parallax.x) / (line_slope * line_slope + 1));
		int y_right = (int)(line_slope * x_right + line_intercept);

		std::vector<POI2D> poi_candidates;
		POI2D current_poi(poi->x, poi->y);
		current_poi.deformation.u = x_right - poi->x;
		current_poi.deformation.v = y_right - poi->y;
		poi_candidates.push_back(current_poi);

		int img_height = icgn1->ref_img->height;
		int img_width = icgn1->ref_img->width;
		int x_trial, y_trial;
		for (int i = search_step; i < search_radius; i += search_step) {
			x_trial = x_right + i;
			y_trial = (int)(line_slope * x_trial + line_intercept);
			current_poi.deformation.u = x_trial - poi->x;
			current_poi.deformation.v = y_trial - poi->y;
			if (x_trial - icgn_sr_x > 0 && x_trial + icgn_sr_x < icgn1->ref_img->width - 1 && y_trial - icgn_sr_y > 0 && y_trial + icgn_sr_y < icgn1->ref_img->height - 1) {
				poi_candidates.push_back(current_poi);
			}

			x_trial = x_right - i;
			y_trial = (int)(line_slope * x_trial + line_intercept);
			current_poi.deformation.u = x_trial - poi->x;
			current_poi.deformation.v = y_trial - poi->y;
			if (x_trial - icgn_sr_x > 0 && x_trial + icgn_sr_x < icgn1->ref_img->width - 1 && y_trial - icgn_sr_y > 0 && y_trial + icgn_sr_y < icgn1->ref_img->height - 1) {
				poi_candidates.push_back(current_poi);
			}
		}

		icgn1->compute(poi_candidates);

		std::sort(poi_candidates.begin(), poi_candidates.end(), sortByZNCC);

		poi->deformation = poi_candidates[0].deformation;
		poi->deformation = poi_candidates[0].deformation;
		poi->result = poi_candidates[0].result;
		poi->result = poi_candidates[0].result;
	}

	void EpipolarSearch::compute(std::vector<POI2D>& poi_queue) {
		//do not run it using multiple threads, as the internal icgn instance is already in multi-thread mode
		for (int i = 0; i < poi_queue.size(); ++i) {
			compute(&poi_queue[i]);
		}
	}

	bool sortByZNCC(const POI2D& p1, const POI2D& p2) {
		return p1.result.zncc > p2.result.zncc;
	}

}//namespace opencorr