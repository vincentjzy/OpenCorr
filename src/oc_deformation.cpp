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

#include "oc_deformation.h"

namespace opencorr
{
	//deformation2d 1st order
	Deformation2D1::Deformation2D1() {
		this->u = 0.f;
		this->ux = 0.f;
		this->uy = 0.f;
		this->v = 0.f;
		this->vx = 0.f;
		this->vy = 0.f;

		this->setWarp();
	}

	Deformation2D1::Deformation2D1(float u, float ux, float uy, float v, float vx, float vy) {
		this->u = u;
		this->ux = ux;
		this->uy = uy;
		this->v = v;
		this->vx = vx;
		this->vy = vy;

		this->setWarp();
	}

	Deformation2D1::Deformation2D1(float p[6]) {
		this->u = p[0];
		this->ux = p[1];
		this->uy = p[2];
		this->v = p[3];
		this->vx = p[4];
		this->vy = p[5];

		this->setWarp();
	}

	Deformation2D1::~Deformation2D1() {
	}

	void Deformation2D1::setDeformation(float u, float ux, float uy, float v, float vx, float vy) {
		this->u = u;
		this->ux = ux;
		this->uy = uy;
		this->v = v;
		this->vx = vx;
		this->vy = vy;

		this->setWarp();
	}

	void Deformation2D1::setDeformation(float p[6]) {
		this->u = p[0];
		this->ux = p[1];
		this->uy = p[2];
		this->v = p[3];
		this->vx = p[4];
		this->vy = p[5];

		this->setWarp();
	}

	void Deformation2D1::setDeformation(Deformation2D1& another_deformation) {
		this->u = another_deformation.u;
		this->ux = another_deformation.ux;
		this->uy = another_deformation.uy;
		this->v = another_deformation.v;
		this->vx = another_deformation.vx;
		this->vy = another_deformation.vy;

		this->setWarp();
	}

	Point2D Deformation2D1::warp(Point2D& point) {
		Eigen::Vector3f point_vector;
		point_vector(0) = point.x;
		point_vector(1) = point.y;
		point_vector(2) = 1.f;

		Eigen::Vector3f warped_vector = this->warp_matrix * point_vector;

		Point2D new_point(warped_vector(0), warped_vector(1));
		return new_point;
	}

	void Deformation2D1::setDeformation() {
		this->u = this->warp_matrix(0, 2);
		this->ux = this->warp_matrix(0, 0) - 1;
		this->uy = this->warp_matrix(0, 1);
		this->v = this->warp_matrix(1, 2);
		this->vx = this->warp_matrix(1, 0);
		this->vy = this->warp_matrix(1, 1) - 1;
	}

	void Deformation2D1::setWarp() {
		this->warp_matrix(0, 0) = 1 + this->ux;
		this->warp_matrix(0, 1) = this->uy;
		this->warp_matrix(0, 2) = this->u;
		this->warp_matrix(1, 0) = this->vx;
		this->warp_matrix(1, 1) = 1 + this->vy;
		this->warp_matrix(1, 2) = this->v;
		this->warp_matrix(2, 0) = 0;
		this->warp_matrix(2, 1) = 0;
		this->warp_matrix(2, 2) = 1.f;
	}


	//deformation2d 2nd order////////////////////////////////////////////////////////////////////////////
	Deformation2D2::Deformation2D2() {
		this->u = 0.f;
		this->ux = 0.f;
		this->uy = 0.f;
		this->uxx = 0.f;
		this->uxy = 0.f;
		this->uyy = 0.f;

		this->v = 0.f;
		this->vx = 0.f;
		this->vy = 0.f;
		this->vxx = 0.f;
		this->vxy = 0.f;
		this->vyy = 0.f;

		this->setWarp();
	}

	Deformation2D2::Deformation2D2(float u, float ux, float uy, float uxx, float uxy, float uyy,
		float v, float vx, float vy, float vxx, float vxy, float vyy) {
		this->u = u;
		this->ux = ux;
		this->uy = uy;
		this->uxx = uxx;
		this->uxy = uxy;
		this->uyy = uyy;

		this->v = v;
		this->vx = vx;
		this->vy = vy;
		this->vxx = vxx;
		this->vxy = vxy;
		this->vyy = vyy;

		this->setWarp();
	}

	Deformation2D2::Deformation2D2(float p[12]) {
		this->u = p[0];
		this->ux = p[1];
		this->uy = p[2];
		this->uxx = p[3];
		this->uxy = p[4];
		this->uyy = p[5];

		this->v = p[6];
		this->vx = p[7];
		this->vy = p[8];
		this->vxx = p[9];
		this->vxy = p[10];
		this->vyy = p[11];

		this->setWarp();
	}

	Deformation2D2::~Deformation2D2() {
	}

	void Deformation2D2::setDeformation(float u, float ux, float uy, float uxx, float uxy, float uyy,
		float v, float vx, float vy, float vxx, float vxy, float vyy) {
		this->u = u;
		this->ux = ux;
		this->uy = uy;
		this->uxx = uxx;
		this->uxy = uxy;
		this->uyy = uyy;

		this->v = v;
		this->vx = vx;
		this->vy = vy;
		this->vxx = vxx;
		this->vxy = vxy;
		this->vyy = vyy;

		this->setWarp();
	}

	void Deformation2D2::setDeformation(float p[12]) {
		this->u = p[0];
		this->ux = p[1];
		this->uy = p[2];
		this->uxx = p[3];
		this->uxy = p[4];
		this->uyy = p[5];

		this->v = p[6];
		this->vx = p[7];
		this->vy = p[8];
		this->vxx = p[9];
		this->vxy = p[10];
		this->vyy = p[11];

		this->setWarp();
	}

	void Deformation2D2::setDeformation(Deformation2D2& another_deformation) {
		this->u = another_deformation.u;
		this->ux = another_deformation.ux;
		this->uy = another_deformation.uy;
		this->uxx = another_deformation.uxx;
		this->uxy = another_deformation.uxy;
		this->uyy = another_deformation.uyy;

		this->v = another_deformation.v;
		this->vx = another_deformation.vx;
		this->vy = another_deformation.vy;
		this->vxx = another_deformation.vxx;
		this->vxy = another_deformation.vxy;
		this->vyy = another_deformation.vyy;

		this->setWarp();
	}

	void Deformation2D2::setDeformation(Deformation2D1& another_deformation) {
		this->u = another_deformation.u;
		this->ux = another_deformation.ux;
		this->uy = another_deformation.uy;
		this->uxx = 0.f;
		this->uxy = 0.f;
		this->uyy = 0.f;

		this->v = another_deformation.v;
		this->vx = another_deformation.vx;
		this->vy = another_deformation.vy;
		this->vxx = 0.f;
		this->vxy = 0.f;
		this->vyy = 0.f;

		this->setWarp();
	}

	Point2D Deformation2D2::warp(Point2D point) {
		Vector6f point_vector;
		point_vector(0) = point.x * point.x;
		point_vector(1) = point.x * point.y;
		point_vector(2) = point.y * point.y;
		point_vector(3) = point.x;
		point_vector(4) = point.y;
		point_vector(5) = 1.f;

		Vector6f warped_vector = this->warp_matrix * point_vector;

		Point2D new_point(warped_vector(3), warped_vector(4));
		return new_point;
	}

	void Deformation2D2::setDeformation() {
		this->u = this->warp_matrix(3, 5);
		this->ux = this->warp_matrix(3, 3) - 1.f;
		this->uy = this->warp_matrix(3, 4);
		this->uxx = this->warp_matrix(3, 0) * 2.f;
		this->uxy = this->warp_matrix(3, 1);
		this->uyy = this->warp_matrix(3, 2) * 2.f;

		this->v = this->warp_matrix(4, 5);
		this->vx = this->warp_matrix(4, 3);
		this->vy = this->warp_matrix(4, 4) - 1.f;
		this->vxx = this->warp_matrix(4, 0) * 2.f;
		this->vxy = this->warp_matrix(4, 1);
		this->vyy = this->warp_matrix(4, 2) * 2.f;
	}

	void Deformation2D2::setWarp() {
		// row 0, S1 - S6
		this->warp_matrix(0, 0) = 1.f + 2.f * this->ux + this->ux * this->ux + this->u * this->uxx;
		this->warp_matrix(0, 1) = 2.f * this->u * this->uxy + 2.f * (1.f + this->ux) * this->uy;
		this->warp_matrix(0, 2) = this->uy * this->uy + this->u * this->uyy;
		this->warp_matrix(0, 3) = 2.f * this->u * (1 + this->ux);
		this->warp_matrix(0, 4) = 2.f * this->u * this->uy;
		this->warp_matrix(0, 5) = this->u * this->u;

		// row 1, S7 - S12
		this->warp_matrix(1, 0) = 0.5f * (this->v * this->uxx + 2.f * (1.f + this->ux) * this->vx + this->u * this->vxx);
		this->warp_matrix(1, 1) = 1.f + this->uy * this->vx + this->ux * this->vy + this->v * this->uxy + this->u * this->vxy + this->vy + this->ux;
		this->warp_matrix(1, 2) = 0.5f * (this->v * this->uyy + +2.f * this->uy * (1.f + this->vy) + this->u * this->vyy);
		this->warp_matrix(1, 3) = this->v + this->v * this->ux + this->u * this->vx;
		this->warp_matrix(1, 4) = this->u + this->v * this->uy + this->u * this->vy;
		this->warp_matrix(1, 5) = this->u * this->v;

		// row 2, S12 - S18
		this->warp_matrix(2, 0) = this->vx * this->vx + this->v * this->vxx;
		this->warp_matrix(2, 1) = 2.f * this->v * this->vxy + 2.f * this->vx * (1.f + this->vy);
		this->warp_matrix(2, 2) = 1.f + 2.f * this->vy + this->vy * this->vy + this->v * this->vyy;
		this->warp_matrix(2, 3) = 2.f * this->v * this->vx;
		this->warp_matrix(2, 4) = 2.f * this->v * (1.f + this->vy);
		this->warp_matrix(2, 5) = this->v * this->v;

		// row 3
		this->warp_matrix(3, 0) = 0.5f * this->uxx;
		this->warp_matrix(3, 1) = this->uxy;
		this->warp_matrix(3, 2) = 0.5f * this->uyy;
		this->warp_matrix(3, 3) = 1.f + this->ux;
		this->warp_matrix(3, 4) = this->uy;
		this->warp_matrix(3, 5) = this->u;

		// row 4
		this->warp_matrix(4, 0) = 0.5f * this->vxx;
		this->warp_matrix(4, 1) = this->vxy;
		this->warp_matrix(4, 2) = 0.5f * this->vyy;
		this->warp_matrix(4, 3) = this->vx;
		this->warp_matrix(4, 4) = 1.f + this->vy;
		this->warp_matrix(4, 5) = this->v;

		// row 5
		this->warp_matrix(5, 0) = 0.f;
		this->warp_matrix(5, 1) = 0.f;
		this->warp_matrix(5, 2) = 0.f;
		this->warp_matrix(5, 3) = 0.f;
		this->warp_matrix(5, 4) = 0.f;
		this->warp_matrix(5, 5) = 1.f;
	}

}//namespace opencorr