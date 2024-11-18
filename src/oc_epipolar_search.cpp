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

#include "oc_epipolar_search.h"

namespace opencorr
{
	EpipolarSearch::EpipolarSearch(Calibration& view1_cam, Calibration& view2_cam, int thread_number)
	{
		this->view1_cam = view1_cam;
		this->view2_cam = view2_cam;
		this->thread_number = thread_number;
	}

	EpipolarSearch::~EpipolarSearch()
	{
		destoryICGN();
	}

	int EpipolarSearch::getSearchRadius() const
	{
		return search_radius;
	}

	int EpipolarSearch::getSearchStep() const
	{
		return search_step;
	}

	void EpipolarSearch::setSearch(int search_radius, int search_step)
	{
		if (search_radius < search_step)
		{
			std::cerr << "Search radius is less than search step" << std::endl;
			return;
		}
		this->search_radius = search_radius;
		this->search_step = search_step;
	}

	void EpipolarSearch::createICGN(int subset_radius_x, int subset_radius_y, float conv_criterion, float stop_condition)
	{
		icgn1 = new ICGN2D1(subset_radius_x, subset_radius_y, conv_criterion, stop_condition, thread_number);
	}

	void EpipolarSearch::prepareICGN()
	{
		icgn1->setImages(*ref_img, *tar_img);
		icgn1->prepare();
	}

	void EpipolarSearch::destoryICGN()
	{
		if (icgn1 != nullptr)
		{
			delete icgn1;
		}
	}

	void EpipolarSearch::setParallax(Point2D parallax)
	{
		this->parallax = parallax;

		parallax_x[0] = 0;
		parallax_x[1] = 0;
		parallax_x[2] = parallax.x;

		parallax_y[0] = 0;
		parallax_y[1] = 0;
		parallax_y[2] = parallax.y;
	}

	void EpipolarSearch::setParallax(float coefficient_x[3], float coefficient_y[3])
	{
		parallax_x[0] = coefficient_x[0];
		parallax_x[1] = coefficient_x[1];
		parallax_x[2] = coefficient_x[2];

		parallax_y[0] = coefficient_y[0];
		parallax_y[1] = coefficient_y[1];
		parallax_y[2] = coefficient_y[2];
	}

	void EpipolarSearch::updateCameras(Calibration& view1_cam, Calibration& view2_cam)
	{
		this->view1_cam = view1_cam;
		this->view2_cam = view2_cam;
	}

	void EpipolarSearch::updateFundementalMatrix()
	{
		//creat transposed inverse intrinsic matrix of right camera
		Eigen::Matrix3f right_invK_t = view2_cam.intrinsic_matrix.inverse().transpose();

		//create an anti-symmetric matrix of translation vector of right camera
		Eigen::Matrix3f right_t_antisymmetric;
		right_t_antisymmetric << 0, -view2_cam.translation_vector(2), view2_cam.translation_vector(1),
			view2_cam.translation_vector(2), 0, -view2_cam.translation_vector(0),
			-view2_cam.translation_vector(1), view2_cam.translation_vector(0), 0;

		//create essential matrix of right camera
		Eigen::Matrix3f right_E = right_t_antisymmetric * view2_cam.rotation_matrix;

		//creat inversed intrinsic matrix of left camera
		Eigen::Matrix3f left_K = view1_cam.intrinsic_matrix.inverse();

		fundamental_matrix = right_invK_t * right_E * left_K;
	}

	void EpipolarSearch::prepare()
	{
		view1_cam.updateMatrices();
		view2_cam.updateMatrices();

		updateFundementalMatrix();

		prepareICGN();
	}

	void EpipolarSearch::compute(POI2D* poi)
	{
		//estimate parallax
		parallax.x = parallax_x[0] * (poi->x - int(ref_img->width / 2)) + parallax_x[1] * (poi->y - int(ref_img->height / 2)) + parallax_x[2];
		parallax.y = parallax_y[0] * (poi->x - int(ref_img->width / 2)) + parallax_y[1] * (poi->y - int(ref_img->height / 2)) + parallax_y[2];

		//convert locatoin of left POI to a vector
		Eigen::Vector3f view1_vector;
		view1_vector << (poi->x + poi->deformation.u), (poi->y + poi->deformation.v), 1;

		//get the projection of POI in the primary view on the epipolar line in the secondary view
		Eigen::Vector3f view2_epipolar = fundamental_matrix * view1_vector;
		float line_slope = -view2_epipolar(0) / view2_epipolar(1);
		float line_intercept = -view2_epipolar(2) / view2_epipolar(1);
		int x_view2 = (int)((line_slope * (poi->y + poi->deformation.v + parallax.y - line_intercept)
			+ poi->x + poi->deformation.u + parallax.x) / (line_slope * line_slope + 1));
		int y_view2 = (int)(line_slope * x_view2 + line_intercept);

		//get the center of searching region
		std::vector<POI2D> poi_candidates;
		POI2D current_poi(poi->x, poi->y);
		current_poi.deformation.u = x_view2 - poi->x;
		current_poi.deformation.v = y_view2 - poi->y;
		poi_candidates.push_back(current_poi);

		//get the other trial locations in searching region
		int x_trial, y_trial;
		for (int i = search_step; i < search_radius; i += search_step) {
			x_trial = x_view2 + i;
			y_trial = (int)(line_slope * x_trial + line_intercept);
			current_poi.deformation.u = x_trial - poi->x;
			current_poi.deformation.v = y_trial - poi->y;
			if (x_trial - icgn1->subset_radius_x > 0 && x_trial + icgn1->subset_radius_x < icgn1->ref_img->width - 1
				&& y_trial - icgn1->subset_radius_y > 0 && y_trial + icgn1->subset_radius_y < icgn1->ref_img->height - 1)
			{
				poi_candidates.push_back(current_poi);
			}

			x_trial = x_view2 - i;
			y_trial = (int)(line_slope * x_trial + line_intercept);
			current_poi.deformation.u = x_trial - poi->x;
			current_poi.deformation.v = y_trial - poi->y;
			if (x_trial - icgn1->subset_radius_x > 0 && x_trial + icgn1->subset_radius_x < icgn1->ref_img->width - 1
				&& y_trial - icgn1->subset_radius_y > 0 && y_trial + icgn1->subset_radius_y < icgn1->ref_img->height - 1)
			{
				poi_candidates.push_back(current_poi);
			}
		}

		//coarse check using ICGN1
		int queue_size = (int)poi_candidates.size();
#pragma omp parallel for
		for (int i = 0; i < queue_size; i++)
		{
			icgn1->compute(&poi_candidates[i]);
		}

		//take the one with the highest ZNCC value
		std::sort(poi_candidates.begin(), poi_candidates.end(), sortByZNCC);

		poi->deformation = poi_candidates[0].deformation;
		poi->result = poi_candidates[0].result;
	}

	void EpipolarSearch::compute(std::vector<POI2D>& poi_queue)
	{
		int queue_length = (int)poi_queue.size();
		//CAUTION: no need to use OMP parallel for, as the parallelism has been implemented in the processing of each POI
		for (int i = 0; i < queue_length; i++)
		{
			compute(&poi_queue[i]);
		}
	}

}//namespace opencorr