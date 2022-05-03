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

#ifndef _EPIPOLAR_SEARCH_H_
#define _EPIPOLAR_SEARCH_H_

#include "oc_array.h"
#include "oc_calibration.h"
#include "oc_dic.h"
#include "oc_icgn.h"
#include "oc_image.h"
#include "oc_poi.h"
#include "oc_point.h"

namespace opencorr
{
	class EpipolarSearch : public DIC
	{
	protected:
		int search_radius; //seaching radius along the epipolar
		int search_step; //step of search along the epipolar
		Calibration view1_cam; //intrinsics and extrinsics of the principal camera
		Calibration view2_cam; //intrinsics and extrinsics of the secondary camera
		Eigen::Matrix3f fundamental_matrix; //fundamental mattrix of stereovision system
		Point2D parallax; //parallax in view2 with respect to view1 
		float parallax_x[3], parallax_y[3]; //linear regression coefficients of parallax with respect to coordinates

	public:
		ICGN2D1* icgn1;

		EpipolarSearch(Calibration& view1_cam, Calibration& view2_cam, int thread_number);
		~EpipolarSearch();

		int getSearchRadius() const;
		int getSearchStep() const;
		void setSearch(int search_radius, int search_step);
		void createICGN(int subset_radius_x, int subset_radius_y, float conv_criterion, float stop_condition);
		void prepareICGN();
		void destoryICGN();

		void setParallax(Point2D parallax);
		void setParallax(float coefficient_x[3], float coefficient_y[3]);

		void updateCameras(Calibration& view1_cam, Calibration& view2_cam);
		void updateFundementalMatrix();

		void prepare();
		void compute(POI2D* poi);
		void compute(std::vector<POI2D>& poi_queue);
	};

	bool sortByZNCC(const POI2D& p1, const POI2D& p2);

}//namespace opencorr

#endif //_EPIPOLAR_SEARCH_H_