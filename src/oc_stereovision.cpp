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
	Stereovision::Stereovision(Calibration& left_camera, Calibration& right_camera) {
		this->left_camera = &left_camera;
		this->right_camera = &right_camera;
		this->icgn1 = new ICGN2D1();
		this->icgn2 = new ICGN2D2();
	}

	Stereovision::~Stereovision() {
		delete this->icgn1;
		delete this->icgn2;
	}

	int Stereovision::getSearchRadius() const {
		return this->search_radius;
	}

	int Stereovision::getSearchStep() const {
		return this->search_step;
	}

	void Stereovision::setImages(Image2D& left_view_image, Image2D& right_view_image) {
		this->left_view_image = &left_view_image;
		this->right_view_image = &right_view_image;
	}

	void Stereovision::prepare() {
		this->left_camera->setIntrinsicMatrix();
		this->left_camera->setRotationMatrix();
		this->left_camera->setTranslationVector();
		this->left_camera->setProjectionMatrix();
		this->right_camera->setIntrinsicMatrix();
		this->right_camera->setRotationMatrix();
		this->right_camera->setTranslationVector();
		this->right_camera->setProjectionMatrix();
	}

	void Stereovision::setSearch(int search_radius, int search_step) {
		if (search_step >= search_radius) {
			std::cerr << "search radius is less than search step";
			exit(1);
		}
		this->search_radius = search_radius;
		this->search_step = search_step;
	}

	void Stereovision::setThreadNumber(int CPU_thread_number) {
		this->CPU_thread_number = CPU_thread_number;
		omp_set_num_threads(this->CPU_thread_number);
	}

	void Stereovision::fundementalMatrix() {
		//creat transposed inverse intrinsic matrix of right camera
		Eigen::Matrix3f right_invK_t = this->right_camera->intrinsic_matrix.inverse().transpose();

		//create an anti-symmetric matrix of translation vector of right camera
		Eigen::Matrix3f right_t_antisymmetric;
		right_t_antisymmetric << 0, -this->right_camera->translation_vector(2), this->right_camera->translation_vector(1),
			this->right_camera->translation_vector(2), 0, -this->right_camera->translation_vector(0),
			-this->right_camera->translation_vector(1), this->right_camera->translation_vector(0), 0;

		//create essential matrix of right camera
		Eigen::Matrix3f right_E = right_t_antisymmetric * this->right_camera->rotation_matrix;

		//creat inversed intrinsic matrix of left camera
		Eigen::Matrix3f left_K = this->left_camera->intrinsic_matrix.inverse();

		this->fundamental_matrix = right_invK_t * right_E * left_K;
	}

	Point2D Stereovision::epoipolarMatch(POI2D& POI) {
		//convert locatoin of left POI to a vector
		Eigen::Vector3f left_vector;
		left_vector << (POI.x + POI.deformation.u), (POI.y + POI.deformation.v), 1;

		//get the projection of POI in left view on the epipolar line in right view
		Eigen::Vector3f right_epipolar = this->fundamental_matrix * left_vector;
		float line_slope = -right_epipolar(0) / right_epipolar(1);
		float line_intercept = -right_epipolar(2) / right_epipolar(1);
		int x_right = (int)((line_slope * (POI.y - line_intercept) + POI.x) / (line_slope * line_slope + 1));
		int y_right = (int)(line_slope * x_right + line_intercept);

		std::vector<POI2D> POI_candidates;
		POI2D current_POI(x_right, y_right);
		current_POI.deformation.u = x_right - POI.x;
		current_POI.deformation.v = y_right - POI.y;
		POI_candidates.push_back(current_POI);

		int x_trial, y_trial;
		for (int i = search_step; i < this->search_radius; i += this->search_step) {
			x_trial = x_right + i;
			y_trial = (int)(line_slope * x_trial + line_intercept);
			current_POI.deformation.u = x_trial - POI.x;
			current_POI.deformation.v = y_trial - POI.y;
			POI_candidates.push_back(current_POI);

			x_trial = x_right - i;
			y_trial = (int)(line_slope * x_trial + line_intercept);
			current_POI.deformation.u = x_trial - POI.x;
			current_POI.deformation.v = y_trial - POI.y;
			POI_candidates.push_back(current_POI);
		}

		this->icgn1->setImages(*this->left_view_image, *this->right_view_image);
		this->icgn1->prepare();

		for (int i = 0; i < (int)POI_candidates.size(); ++i) {
			this->icgn1->compute(&POI_candidates[i]);
		}

		std::sort(POI_candidates.begin(), POI_candidates.end(), sortByZNCC);

		this->icgn2->setImages(*this->left_view_image, *this->right_view_image);
		this->icgn2->prepare();
		this->icgn2->compute(&POI_candidates[0]);

		Point2D POI_right(POI.x + POI_candidates[0].deformation.u, POI.y + POI_candidates[0].deformation.v);
		return POI_right;
	}

	Point3D Stereovision::reconstruct(Point2D& left_point, Point2D& right_point) {
		Point2D left_coordinate = this->left_camera->correct(left_point);
		Point2D right_coordinate = this->right_camera->correct(right_point);

		float x_left = left_coordinate.x;
		float y_left = left_coordinate.y;
		float x_right = right_coordinate.x;
		float y_right = right_coordinate.y;

		//left side of equations
		Eigen::MatrixXf left_matrix(4, 3);
		//column 1
		left_matrix(0, 0) = x_left * this->left_camera->projection_matrix(2, 0) - this->left_camera->projection_matrix(0, 0);
		left_matrix(1, 0) = y_left * this->left_camera->projection_matrix(2, 0) - this->left_camera->projection_matrix(1, 0);
		left_matrix(2, 0) = x_right * this->right_camera->projection_matrix(2, 0) - this->right_camera->projection_matrix(0, 0);
		left_matrix(3, 0) = y_right * this->right_camera->projection_matrix(2, 0) - this->right_camera->projection_matrix(1, 0);

		//column 2
		left_matrix(0, 1) = x_left * this->left_camera->projection_matrix(2, 1) - this->left_camera->projection_matrix(0, 1);
		left_matrix(1, 1) = y_left * this->left_camera->projection_matrix(2, 1) - this->left_camera->projection_matrix(1, 1);
		left_matrix(2, 1) = x_right * this->right_camera->projection_matrix(2, 1) - this->right_camera->projection_matrix(0, 1);
		left_matrix(3, 1) = y_right * this->right_camera->projection_matrix(2, 1) - this->right_camera->projection_matrix(1, 1);

		//column 3
		left_matrix(0, 2) = x_left * this->left_camera->projection_matrix(2, 2) - this->left_camera->projection_matrix(0, 2);
		left_matrix(1, 2) = y_left * this->left_camera->projection_matrix(2, 2) - this->left_camera->projection_matrix(1, 2);
		left_matrix(2, 2) = x_right * this->right_camera->projection_matrix(2, 2) - this->right_camera->projection_matrix(0, 2);
		left_matrix(3, 2) = y_right * this->right_camera->projection_matrix(2, 2) - this->right_camera->projection_matrix(1, 2);

		//right side of equations
		Eigen::Vector4f right_matrix;
		right_matrix(0) = this->left_camera->projection_matrix(0, 3) - x_left * this->left_camera->projection_matrix(2, 3);
		right_matrix(1) = this->left_camera->projection_matrix(0, 3) - y_left * this->left_camera->projection_matrix(2, 3);
		right_matrix(2) = this->right_camera->projection_matrix(0, 3) - x_right * this->right_camera->projection_matrix(2, 3);
		right_matrix(3) = this->right_camera->projection_matrix(0, 3) - y_right * this->right_camera->projection_matrix(2, 3);

		Eigen::Vector3f world_coordinate = left_matrix.colPivHouseholderQr().solve(right_matrix);

		Point3D reconstructed_point(world_coordinate(0), world_coordinate(1), world_coordinate(2));
		return reconstructed_point;
	}

	bool sortByZNCC(const POI2D& p1, const POI2D& p2) {
		return p1.result.ZNCC > p2.result.ZNCC;
	}
}//namespace opencorr