/*
 * This file is part of OpenCorr, an open-source C++ library for
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

#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

#include "oc_array.h"
#include "oc_point.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace opencorr
{
	union CameraIntrisics {
		struct {
			float fx, fy, fs;
			float cx, cy;
			float k1, k2, k3, k4, k5, k6;
			float p1, p2;
		};
		float cam_i[13];
	};

	union CameraExtrisics {
		struct {
			float tx, ty, tz;
			float pitch, roll, yaw;
		};
		float cam_e[5];
	};

	class Calibration
	{
	public:
		CameraIntrisics intrisics;
		CameraExtrisics extrisics;
		float convergence;
		int iteration;

		Eigen::Matrix3f intrisic_matrix; //camera intrisic matrix of camera
		Eigen::Matrix3f rotation_matrix; //camera rotation matrix of camera
		Eigen::Vector3f translation_vector; //camera translation matrix of camera
		Eigen::MatrixXf projection_matrix; //projection matrix of camera

		Calibration(CameraIntrisics& intrisics, CameraExtrisics& extrisics);
		~Calibration();

		void setIntrisicMatrix();
		void setRotationMatrix();
		void setTranslationVector();
		void setProjectionMatrix();
		void setCorrection(float convergence, int iteration);
		Point2D distort(Point2D& point);
		Point2D correct(Point2D& point);
	};

} //opencorr
#endif //_CALIBRATION_H_
