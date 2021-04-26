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
#include <opencv2\calib3d.hpp>
#include <opencv2\opencv.hpp>

#include "oc_calibration.h"

namespace opencorr
{

	Calibration::Calibration(CameraIntrinsics& intrinsics, CameraExtrinsics& extrinsics) {
		this->intrinsics = intrinsics;
		this->extrinsics = extrinsics;

		this->setIntrinsicMatrix();
		this->setRotationMatrix();
		this->setTranslationVector();
		this->setProjectionMatrix();

		this->convergence = 0.001f;
		this->iteration = 10;
	}

	Calibration::~Calibration() {
	}

	void Calibration::setIntrinsicMatrix() {
		this->intrinsic_matrix.setIdentity();
		this->intrinsic_matrix(0, 0) = this->intrinsics.fx;
		this->intrinsic_matrix(0, 1) = this->intrinsics.fs;
		this->intrinsic_matrix(0, 2) = this->intrinsics.cx;
		this->intrinsic_matrix(1, 1) = this->intrinsics.fy;
		this->intrinsic_matrix(1, 2) = this->intrinsics.cy;
		if (this->intrinsic_matrix.isIdentity()) {
			throw std::exception(std::string("Null intrinsics matrix").data());
		}
	}

	void Calibration::setRotationMatrix() {
		cv::Mat cv_rotation_matrix;
		cv::Mat cv_rotation_vector = (cv::Mat_<float>(3, 1) << this->extrinsics.pitch, this->extrinsics.roll, this->extrinsics.yaw);
		cv::Rodrigues(cv_rotation_vector, cv_rotation_matrix);
		cv::cv2eigen(cv_rotation_matrix, this->rotation_matrix);
	}

	void Calibration::setTranslationVector() {
		this->translation_vector(0) = this->extrinsics.tx;
		this->translation_vector(1) = this->extrinsics.ty;
		this->translation_vector(2) = this->extrinsics.tz;
	}

	void Calibration::setProjectionMatrix() {
		Eigen::MatrixXf RT_mat(3, 4);

		RT_mat.block(0, 0, 3, 3) << this->rotation_matrix;
		RT_mat.col(2) << this->translation_vector;

		this->projection_matrix = this->intrinsic_matrix * RT_mat;
	}

	Point2D Calibration::distort(Point2D& point) {
		//convert to the physical image coordinates
		float physical_x = (point.x - this->intrinsics.cx - this->intrinsics.fs * point.y) / this->intrinsics.fx;
		float physical_y = (point.y - this->intrinsics.cy) / this->intrinsics.fy;

		float physical_xx = physical_x * physical_x;
		float physical_yy = physical_y * physical_y;
		float physical_xy = physical_x * physical_y;
		float distortion_r2 = physical_xx + physical_yy;
		float distrotion_r4 = distortion_r2 * distortion_r2;
		float distortion_r6 = distortion_r2 * distrotion_r4;
		//radical distortion
		float distortion_radial = 1 + this->intrinsics.k1 * distortion_r2 + this->intrinsics.k2 * distrotion_r4 + this->intrinsics.k3 * distortion_r6;
		float distorted_x = physical_x * distortion_radial;
		float distorted_y = physical_y * distortion_radial;
		//tangential distortion
		distorted_x += 2 * this->intrinsics.p1 * physical_xy + this->intrinsics.p2 * (distortion_r2 + 2 * physical_xx);
		distorted_y += this->intrinsics.p1 * (distortion_r2 + 2 * physical_yy) + 2 * this->intrinsics.p2 * physical_xy;
		//convert back to the pixel image coordinates
		physical_x = distorted_x * this->intrinsics.fx + distorted_y * this->intrinsics.fx + this->intrinsics.cx;
		physical_y = distorted_y * this->intrinsics.fy + this->intrinsics.cy;

		Point2D corrected_coordinate(physical_x, physical_y);
		return corrected_coordinate;
	}

	void Calibration::setCorrection(float convergence, int iteration) {
		this->convergence = convergence;
		this->iteration = iteration;
	}

	Point2D Calibration::correct(Point2D& point) {
		//Newton iterative method
		// correct x-coordinate
		Point2D coordinate_step1 = point;
		Point2D distorted_coordinate_step1 = distort(coordinate_step1) - point;
		Point2D coordinate_step2 = coordinate_step1 - distorted_coordinate_step1;
		Point2D distorted_coordinate_step2 = distort(coordinate_step2) - point;
		Point2D coordinate_increment = coordinate_step2 - coordinate_step1;
		Point2D coordinate_result = coordinate_step2;
		int i = 1;
		while (i < this->iteration && coordinate_increment.x > this->convergence) {
			i++;
			coordinate_result = coordinate_step2 - distorted_coordinate_step2
				/ (distorted_coordinate_step2 - distorted_coordinate_step1) * (coordinate_step2 - coordinate_step1);
			coordinate_step1 = coordinate_step2;
			coordinate_step2 = coordinate_result;
			distorted_coordinate_step1 = distorted_coordinate_step2;
			distorted_coordinate_step2 = distort(coordinate_step2) - point;
			coordinate_increment = coordinate_step2 - coordinate_step1;
		}
		float corrected_x = coordinate_result.x;

		// correct y-coordinate
		coordinate_step1 = point;
		distorted_coordinate_step1 = distort(coordinate_step1) - point;
		coordinate_step2 = coordinate_step1 - distorted_coordinate_step1;
		distorted_coordinate_step2 = distort(coordinate_step2) - point;
		coordinate_increment = coordinate_step2 - coordinate_step1;
		coordinate_result = coordinate_step2;
		i = 1;
		while (i < this->iteration && coordinate_increment.y > this->convergence) {
			i++;
			coordinate_result = coordinate_step2 - distorted_coordinate_step2
				/ (distorted_coordinate_step2 - distorted_coordinate_step1) * (coordinate_step2 - coordinate_step1);
			coordinate_step1 = coordinate_step2;
			coordinate_step2 = coordinate_result;
			distorted_coordinate_step1 = distorted_coordinate_step2;
			distorted_coordinate_step2 = distort(coordinate_step2) - point;
			coordinate_increment = coordinate_step2 - coordinate_step1;
		}
		float corrected_y = coordinate_result.y;
		//create point with corrected coordinates
		Point2D corrected_coordinate(corrected_x, corrected_y);
		return corrected_coordinate;
	}

}//namespace opencorr
