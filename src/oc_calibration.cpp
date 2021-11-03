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

#include <cmath>
#include <opencv2/calib3d.hpp>
#include "oc_calibration.h"

namespace opencorr
{
	Calibration::Calibration() {
		convergence = 0.001f;
		iteration = 40;
	}

	Calibration::Calibration(CameraIntrinsics& intrinsics, CameraExtrinsics& extrinsics) {
		updateCalibration(intrinsics, extrinsics);
		convergence = 0.001f;
		iteration = 40;
	}

	Calibration::~Calibration() {
	}

	void Calibration::updateIntrinsicMatrix() {
		intrinsic_matrix.setIdentity();
		intrinsic_matrix(0, 0) = intrinsics.fx;
		intrinsic_matrix(0, 1) = intrinsics.fs;
		intrinsic_matrix(0, 2) = intrinsics.cx;
		intrinsic_matrix(1, 1) = intrinsics.fy;
		intrinsic_matrix(1, 2) = intrinsics.cy;
		if (intrinsic_matrix.isIdentity()) {
			throw std::string("Null intrinsics matrix");
		}
	}

	void Calibration::updateRotationMatrix() {
		cv::Mat cv_rotation_matrix;
		cv::Mat cv_rotation_vector = (cv::Mat_<float>(3, 1) << extrinsics.rx, extrinsics.ry, extrinsics.rz);
		cv::Rodrigues(cv_rotation_vector, cv_rotation_matrix);
		cv::cv2eigen(cv_rotation_matrix, rotation_matrix);
	}

	void Calibration::updateTranslationVector() {
		translation_vector(0) = extrinsics.tx;
		translation_vector(1) = extrinsics.ty;
		translation_vector(2) = extrinsics.tz;
	}

	void Calibration::updateProjectionMatrix() {
		Eigen::MatrixXf rt_mat(3, 4);

		rt_mat.block(0, 0, 3, 3) << rotation_matrix;
		rt_mat.col(3) << translation_vector;

		projection_matrix = intrinsic_matrix * rt_mat;
	}

	void Calibration::updateCalibration(CameraIntrinsics& intrinsics, CameraExtrinsics& extrinsics) {
		this->intrinsics = intrinsics;
		this->extrinsics = extrinsics;

		updateIntrinsicMatrix();
		updateRotationMatrix();
		updateTranslationVector();
		updateProjectionMatrix();
	}

	void Calibration::setUndistortion(float convergence, int iteration) {
		this->convergence = convergence;
		this->iteration = iteration;
	}

	Point2D Calibration::image_to_sensor(Point2D& point) {
		float sensor_y = point.y * intrinsics.fy + intrinsics.cy;
		float sensor_x = point.x * intrinsics.fx + point.y * intrinsics.fs + intrinsics.cx;

		Point2D sensor_coordinate(sensor_x, sensor_y);
		return sensor_coordinate;
	}

	Point2D Calibration::sensor_to_image(Point2D& point) {
		float image_y = (point.y - intrinsics.cy) / intrinsics.fy;
		float image_x = (point.x - intrinsics.cx - intrinsics.fs * image_y) / intrinsics.fx;

		Point2D image_coordinate(image_x, image_y);
		return image_coordinate;
	}

	//distort the coordinate of a point in image coordinate system
	Point2D Calibration::distort(Point2D& point) {
		//prepare the variables
		float image_xx = point.x * point.x;
		float image_yy = point.y * point.y;
		float image_xy = point.x * point.y;
		float distortion_r2 = image_xx + image_yy;
		float distrotion_r4 = distortion_r2 * distortion_r2;
		float distortion_r6 = distortion_r2 * distrotion_r4;

		//impose radical distortion
		float radial_factor = (1 + intrinsics.k1 * distortion_r2 + intrinsics.k2 * distrotion_r4 + intrinsics.k3 * distortion_r6)
			/ (1 + intrinsics.k4 * distortion_r2 + intrinsics.k5 * distrotion_r4 + intrinsics.k6 * distortion_r6);
		float distorted_y = point.y * radial_factor;
		float distorted_x = point.x * radial_factor;

		//impose tangential distortion
		distorted_y += intrinsics.p1 * (distortion_r2 + 2 * image_yy) + 2 * intrinsics.p2 * image_xy;
		distorted_x += 2 * intrinsics.p1 * image_xy + intrinsics.p2 * (distortion_r2 + 2 * image_xx);

		//return the distorted coordinates
		Point2D distorted_coordinate(distorted_x, distorted_y);
		return distorted_coordinate;
	}

	void Calibration::prepare(int height, int width) {
		//initialize the map of distorted coordinates in image coordinate system
		map_x = Eigen::MatrixXf::Zero(height, width);
		map_y = Eigen::MatrixXf::Zero(height, width);

		//initialize the correction map
#pragma omp parallel for
		for (int r = 0; r < height; r++) {
			for (int c = 0; c < width; c++) {
				Point2D sensor_coordinate(c, r);
				Point2D image_coordinate(sensor_to_image(sensor_coordinate));
				map_x(r, c) = image_coordinate.x;
				map_y(r, c) = image_coordinate.y;
			}
		}

#pragma omp parallel for
		for (int r = 0; r < height; r++) {
			for (int c = 0; c < width; c++) {
				float deviation_y;
				float deviation_x;
				bool stop_iteration = false;
				int i = 0;
				Point2D image_coordinate(map_x(r, c), map_y(r, c));
				while (i < iteration && stop_iteration == false) {
					i++;
					Point2D distorted_coordinate(distort(image_coordinate));
					Point2D sensor_coordinate(image_to_sensor(distorted_coordinate));
					deviation_y = r - sensor_coordinate.y;
					deviation_x = c - sensor_coordinate.x;
					if (std::isinf(deviation_x) || std::isinf(deviation_y)) {
						stop_iteration = true;
						image_coordinate.y = map_y(r, c);
						image_coordinate.x = map_x(r, c);
					}
					if (fabs(deviation_x) > convergence || fabs(deviation_y) > convergence) {
						deviation_y /= intrinsics.fy;
						image_coordinate.y += deviation_y;
						image_coordinate.x += (deviation_x - deviation_y * intrinsics.fs) / intrinsics.fx;
					}
					else {
						stop_iteration = true;
					}
				}
				map_y(r, c) = image_coordinate.y;
				map_x(r, c) = image_coordinate.x;
			}
		}

	}

	Point2D Calibration::undistort_nt(Point2D& point) {
		//convert to the image coordinates
		Point2D image_coordinate(sensor_to_image(point));

		//initialize the coordinates for Newton algorithm
		int i = 0;
		Point2D coordinate_previous;
		Point2D coordinate_current(image_coordinate.x, image_coordinate.y);
		Point2D coordinate_increment(1, 1);

		//iterative procedure for undistortion
		while (i < iteration && (fabs(coordinate_increment.x) > convergence || fabs(coordinate_increment.y) > convergence)) {
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
			float radial_factor = (1 + intrinsics.k4 * distortion_r2 + intrinsics.k5 * distrotion_r4 + intrinsics.k5 * distortion_r6)
				/ (1 + intrinsics.k1 * distortion_r2 + intrinsics.k2 * distrotion_r4 + intrinsics.k3 * distortion_r6);
			float tangential_y = intrinsics.p1 * (distortion_r2 + 2 * physical_yy) + 2 * intrinsics.p2 * physical_xy;
			float tangential_x = 2 * intrinsics.p1 * physical_xy + intrinsics.p2 * (distortion_r2 + 2 * physical_xx);
			coordinate_current.y = (coordinate_previous.y - tangential_y) * radial_factor;
			coordinate_current.x = (coordinate_previous.x - tangential_x) * radial_factor;

			//calculate the increment
			coordinate_increment = coordinate_current - coordinate_previous;
		}

		//convert back to the sensor coordinates
		Point2D undistorted_coordinate(image_to_sensor(image_coordinate));
		return undistorted_coordinate;
	}

	Point2D Calibration::undistort(Point2D& point) {
		//deal with the points adjacent to boundary
		if (point.x < 0)
			point.x = 0;
		if (point.y < 0)
			point.y = 0;
		if (point.x > map_x.cols() - 2)
			point.x = map_x.cols() - 2;
		if (point.y > map_y.rows() - 2)
			point.y = map_y.rows() - 2;

		//get integral part and decimal part of the coordinate
		int y_integral = (int)floor(point.y);
		int x_integral = (int)floor(point.x);

		float y_decimal = point.y - y_integral;
		float x_decimal = point.x - x_integral;

		//locate the position of the point in the map of distorted image coordinates
		float corrected_y = map_y(y_integral, x_integral) * (1 - y_decimal) * (1 - x_decimal)
			+ map_y(y_integral + 1, x_integral) * y_decimal * (1 - x_decimal)
			+ map_y(y_integral, x_integral + 1) * (1 - y_decimal) * x_decimal
			+ map_y(y_integral + 1, x_integral + 1) * y_decimal * x_decimal;

		float corrected_x = map_x(y_integral, x_integral) * (1 - y_decimal) * (1 - x_decimal)
			+ map_x(y_integral + 1, x_integral) * y_decimal * (1 - x_decimal)
			+ map_x(y_integral, x_integral + 1) * (1 - y_decimal) * x_decimal
			+ map_x(y_integral + 1, x_integral + 1) * y_decimal * x_decimal;

		Point2D image_coordinate(corrected_x, corrected_y);

		//convert to the sensor coordinates
		Point2D undistorted_coordinate(image_to_sensor(image_coordinate));
		return undistorted_coordinate;
	}

}//namespace opencorr
