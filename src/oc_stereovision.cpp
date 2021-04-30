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

#include "oc_stereovision.h"

namespace opencorr
{
	Stereovision::Stereovision(Calibration& left_cam, Calibration& right_cam, int thread_number) {
		this->left_cam = left_cam;
		this->right_cam = right_cam;
		this->thread_number = thread_number;
	}

	Stereovision::~Stereovision() {
	}

	void Stereovision::updateCameraParameters(Calibration& left_cam, Calibration& right_cam) {
		this->left_cam = left_cam;
		this->right_cam = right_cam;
	}

	void Stereovision::setPointPair(Point2D& left_point, Point2D& right_point) {
		this->left_2d_pt = left_point;
		this->right_2d_pt = right_point;
	}

	void Stereovision::prepare() {
		this->left_cam.updateIntrinsicMatrix();
		this->left_cam.updateRotationMatrix();
		this->left_cam.updateTranslationVector();
		this->left_cam.updateProjectionMatrix();
		this->right_cam.updateIntrinsicMatrix();
		this->right_cam.updateRotationMatrix();
		this->right_cam.updateTranslationVector();
		this->right_cam.updateProjectionMatrix();
	}

	Point3D Stereovision::reconstruct(Point2D& left_point, Point2D& right_point) {
		Point2D left_coor = this->left_cam.correct(left_point);
		Point2D right_coor = this->right_cam.correct(right_point);

		float x_left = left_coor.x;
		float y_left = left_coor.y;
		float x_right = right_coor.x;
		float y_right = right_coor.y;

		//left side of equations
		Eigen::MatrixXf left_matrix(4, 3);
		//column 1
		left_matrix(0, 0) = x_left * this->left_cam.projection_matrix(2, 0) - this->left_cam.projection_matrix(0, 0);
		left_matrix(1, 0) = y_left * this->left_cam.projection_matrix(2, 0) - this->left_cam.projection_matrix(1, 0);
		left_matrix(2, 0) = x_right * this->right_cam.projection_matrix(2, 0) - this->right_cam.projection_matrix(0, 0);
		left_matrix(3, 0) = y_right * this->right_cam.projection_matrix(2, 0) - this->right_cam.projection_matrix(1, 0);

		//column 2
		left_matrix(0, 1) = x_left * this->left_cam.projection_matrix(2, 1) - this->left_cam.projection_matrix(0, 1);
		left_matrix(1, 1) = y_left * this->left_cam.projection_matrix(2, 1) - this->left_cam.projection_matrix(1, 1);
		left_matrix(2, 1) = x_right * this->right_cam.projection_matrix(2, 1) - this->right_cam.projection_matrix(0, 1);
		left_matrix(3, 1) = y_right * this->right_cam.projection_matrix(2, 1) - this->right_cam.projection_matrix(1, 1);

		//column 3
		left_matrix(0, 2) = x_left * this->left_cam.projection_matrix(2, 2) - this->left_cam.projection_matrix(0, 2);
		left_matrix(1, 2) = y_left * this->left_cam.projection_matrix(2, 2) - this->left_cam.projection_matrix(1, 2);
		left_matrix(2, 2) = x_right * this->right_cam.projection_matrix(2, 2) - this->right_cam.projection_matrix(0, 2);
		left_matrix(3, 2) = y_right * this->right_cam.projection_matrix(2, 2) - this->right_cam.projection_matrix(1, 2);

		//right side of equations
		Eigen::Vector4f right_matrix;
		right_matrix(0) = this->left_cam.projection_matrix(0, 3) - x_left * this->left_cam.projection_matrix(2, 3);
		right_matrix(1) = this->left_cam.projection_matrix(0, 3) - y_left * this->left_cam.projection_matrix(2, 3);
		right_matrix(2) = this->right_cam.projection_matrix(0, 3) - x_right * this->right_cam.projection_matrix(2, 3);
		right_matrix(3) = this->right_cam.projection_matrix(0, 3) - y_right * this->right_cam.projection_matrix(2, 3);

		Eigen::Vector3f world_coor = left_matrix.colPivHouseholderQr().solve(right_matrix);

		this->space_pt.x = world_coor(0);
		this->space_pt.y = world_coor(1);
		this->space_pt.z = world_coor(2);
		return this->space_pt;
	}

}//namespace opencorr