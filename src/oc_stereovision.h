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

#ifndef _STEREOVISION_H_
#define _STEREOVISION_H_

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
	class Stereovision
	{
	protected:
		Calibration left_cam; //intrinsics and extrinsics of left camera
		Calibration right_cam; //intrinsics and extrinsics of right camera
		int thread_number; //OpenMP thread number

	public:
		Point2D left_2d_pt; //2D point in left view
		Point2D right_2d_pt; //2D point in right view
		Point3D space_pt; //3D point in space

		Stereovision(Calibration& left_cam, Calibration& right_cam, int thread_number);
		~Stereovision();

		void updateCameraParameters(Calibration& left_cam, Calibration& right_cam);
		void setPointPair(Point2D& left_point, Point2D& right_point);
		void prepare();

		Point3D reconstruct(Point2D& left_point, Point2D& right_point);

	};
}//namespace opencorr
#endif //_STEREOVISION_H_
