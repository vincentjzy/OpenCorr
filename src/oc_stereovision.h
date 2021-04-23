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
		int search_radius;
		int search_step;

	public:
		Image2D* left_view_image;
		Image2D* right_view_image;
		Calibration* left_camera;
		Calibration* right_camera;
		Eigen::Matrix3f fundamental_matrix;
		ICGN2D1* icgn1;
		ICGN2D2* icgn2;
		int CPU_thread_number;

		Stereovision(Calibration& left_camera, Calibration& right_camera);
		~Stereovision();

		int getSearchRadius() const;
		int getSearchStep() const;
		void setImages(Image2D& left_view_image, Image2D& right_view_image);
		void setSearch(int search_radius, int search_step);
		void setThreadNumber(int CPU_thread_number);
		void prepare();

		void fundementalMatrix();
		Point2D epoipolarMatch(POI2D& left_POI);
		Point3D reconstruct(Point2D& left_point, Point2D& right_point);
	};
	bool sortByZNCC(const POI2D& p1, const POI2D& p2);

}//namespace opencorr
#endif //_STEREOVISION_H_
