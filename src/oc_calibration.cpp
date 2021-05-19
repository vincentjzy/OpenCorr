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

#include <cmath>
#include <opencv2/calib3d.hpp>
#include "oc_calibration.h"

namespace opencorr
{
	Calibration::Calibration() {
	}

	Calibration::Calibration(CameraIntrinsics& intrinsics, CameraExtrinsics& extrinsics) {
		updateCalibration(intrinsics, extrinsics);

		this->convergence = 0.001f;
		this->iteration = 10;
	}

	Calibration::~Calibration() {
	}

	void Calibration::updateIntrinsicMatrix() {
		this->intrinsic_matrix.setIdentity();
		this->intrinsic_matrix(0, 0) = this->intrinsics.fx;
		this->intrinsic_matrix(0, 1) = this->intrinsics.fs;
		this->intrinsic_matrix(0, 2) = this->intrinsics.cx;
		this->intrinsic_matrix(1, 1) = this->intrinsics.fy;
		this->intrinsic_matrix(1, 2) = this->intrinsics.cy;
		if (this->intrinsic_matrix.isIdentity()) {
			throw std::string("Null intrinsics matrix");
		}
	}

	void Calibration::updateRotationMatrix() {
		cv::Mat cv_rotation_matrix;
		cv::Mat cv_rotation_vector = (cv::Mat_<float>(3, 1) << this->extrinsics.rx, this->extrinsics.ry, this->extrinsics.rz);
		cv::Rodrigues(cv_rotation_vector, cv_rotation_matrix);
		cv::cv2eigen(cv_rotation_matrix, this->rotation_matrix);
	}

	void Calibration::updateTranslationVector() {
		this->translation_vector(0) = this->extrinsics.tx;
		this->translation_vector(1) = this->extrinsics.ty;
		this->translation_vector(2) = this->extrinsics.tz;
	}

	void Calibration::updateProjectionMatrix() {
		Eigen::MatrixXf RT_mat(3, 4);

		RT_mat.block(0, 0, 3, 3) << this->rotation_matrix;
		RT_mat.col(3) << this->translation_vector;

		this->projection_matrix = this->intrinsic_matrix * RT_mat;
	}

	void Calibration::updateCalibration(CameraIntrinsics& intrinsics, CameraExtrinsics& extrinsics) {
		this->intrinsics = intrinsics;
		this->extrinsics = extrinsics;

		this->updateIntrinsicMatrix();
		this->updateRotationMatrix();
		this->updateTranslationVector();
		this->updateProjectionMatrix();
	}

	Point2D Calibration::distort(Point2D& point) {
		//convert to the physical image coordinates
		float physical_y = (point.y - this->intrinsics.cy) / this->intrinsics.fy;
		float physical_x = (point.x - this->intrinsics.cx - this->intrinsics.fs * physical_y) / this->intrinsics.fx;

		//prepare the variables
		float physical_xx = physical_x * physical_x;
		float physical_yy = physical_y * physical_y;
		float physical_xy = physical_x * physical_y;
		float distortion_r2 = physical_xx + physical_yy;
		float distrotion_r4 = distortion_r2 * distortion_r2;
		float distortion_r6 = distortion_r2 * distrotion_r4;

		//impose radical distortion
		float distortion_radial = (1 + this->intrinsics.k1 * distortion_r2 + this->intrinsics.k2 * distrotion_r4 + this->intrinsics.k3 * distortion_r6)
			/ (1 + this->intrinsics.k4 * distortion_r2 + this->intrinsics.k5 * distrotion_r4 + this->intrinsics.k6 * distortion_r6);
		float distorted_y = physical_y * distortion_radial;
		float distorted_x = physical_x * distortion_radial;

		//impose tangential distortion
		distorted_y += this->intrinsics.p1 * (distortion_r2 + 2 * physical_yy) + 2 * this->intrinsics.p2 * physical_xy;
		distorted_x += 2 * this->intrinsics.p1 * physical_xy + this->intrinsics.p2 * (distortion_r2 + 2 * physical_xx);

		//convert back to the pixel image coordinates
		float pixel_y = distorted_y * this->intrinsics.fy + this->intrinsics.cy;
		float pixel_x = distorted_x * this->intrinsics.fx + distorted_y * this->intrinsics.fs + this->intrinsics.cx;

		Point2D corrected_coordinate(pixel_x, pixel_y);
		return corrected_coordinate;
	}

	void Calibration::setCorrection(float convergence, int iteration) {
		this->convergence = convergence;
		this->iteration = iteration;
	}

	Point2D Calibration::correct(Point2D& point) {
		//convert to the physical image coordinates
		float physical_y = (point.y - this->intrinsics.cy) / this->intrinsics.fy;
		float physical_x = (point.x - this->intrinsics.cx - this->intrinsics.fs * physical_y) / this->intrinsics.fx;

		//initialize the coordinates for iterative procedure
		int i = 0;
		Point2D coordinate_previous(0, 0);
		Point2D coordinate_current(physical_x, physical_y);
		Point2D coordinate_increment = coordinate_current - coordinate_previous;
		//iterative procedure for undistortion
		while (i < this->iteration && (fabs(coordinate_increment.x) > this->convergence || fabs(coordinate_increment.y) > this->convergence)) {
			i++;
			coordinate_previous = coordinate_current;

			//prepare the variables
			float physical_xx = coordinate_previous.x * coordinate_previous.x;
			float physical_yy = coordinate_previous.y * coordinate_previous.y;
			float physical_xy = coordinate_previous.x * coordinate_previous.y;
			float distortion_r2 = physical_xx + physical_yy;
			float distrotion_r4 = distortion_r2 * distortion_r2;
			float distortion_r6 = distortion_r2 * distrotion_r4;

			//undistort the coordinates
			float radial_factor = (1 + this->intrinsics.k4 * distortion_r2 + this->intrinsics.k5 * distrotion_r4 + this->intrinsics.k5 * distortion_r6)
				/ (1 + this->intrinsics.k1 * distortion_r2 + this->intrinsics.k2 * distrotion_r4 + this->intrinsics.k3 * distortion_r6);
			float tangential_y = this->intrinsics.p1 * (distortion_r2 + 2 * physical_yy) + 2 * this->intrinsics.p2 * physical_xy;
			float tangential_x = 2 * this->intrinsics.p1 * physical_xy + this->intrinsics.p2 * (distortion_r2 + 2 * physical_xx);
			coordinate_current.y = (coordinate_previous.y - tangential_y) * radial_factor;
			coordinate_current.x = (coordinate_previous.x - tangential_x) * radial_factor;

			//calculate the increment
			coordinate_increment = coordinate_current - coordinate_previous;
		}

		//convert back to the pixel image coordinates
		float pixel_y = coordinate_current.y * this->intrinsics.fy + this->intrinsics.cy;
		float pixel_x = coordinate_current.x * this->intrinsics.fx + coordinate_current.y * this->intrinsics.fs + this->intrinsics.cx;

		Point2D corrected_coordinate(pixel_x, pixel_y);
		return corrected_coordinate;
	}

}//namespace opencorr
