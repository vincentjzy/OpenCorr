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
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace opencorr
{
	class EpipolarSearch : public DIC
	{
	protected:
		int search_radius; //seaching radius along the epipolar
		int search_step; //step of search along the epipolar
		Calibration left_cam; //intrinsics and extrinsics of left camera
		Calibration right_cam; //intrinsics and extrinsics of right camera
		Eigen::Matrix3f fundamental_matrix; //fundamental mattrix of stereovision system
		int icgn_sr_x, icgn_sr_y; //subset radius of ICGN
		float icgn_conv; //convergence criterion of ICGN
		float icgn_stop; //stop condition of ICGN

	public:
		ICGN2D1* icgn1;

		EpipolarSearch(Calibration& left_cam, Calibration& right_cam, int thread_number);
		~EpipolarSearch();

		int getSearchRadius() const;
		int getSearchStep() const;
		void setSearch(int search_radius, int search_step);
		void setICGN(int subset_radius_x, int subset_radius_y, float conv_criterion, float stop_condition);

		void updateCameraParameters(Calibration& left_cam, Calibration& right_cam);
		void updateFundementalMatrix();

		void prepare();
		void compute(POI2D* POI);
		void compute(std::vector<POI2D>& POI_queue);
	};

	bool sortByZNCC(const POI2D& p1, const POI2D& p2);

}//namespace opencorr

#endif //_EPIPOLAR_SEARCH_H_