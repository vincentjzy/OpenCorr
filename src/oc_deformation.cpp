/*
 * This file is part of OpenCorr, an open source C++ library for
 * study and development of 2D, 3D/stereo and volumetric
 * digital image correlation.
 *
 * Copyright (C) 2021, Zhenyu Jiang <zhenyujiang@scut.edu.cn>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one from http://mozilla.org/MPL/2.0/.
 *
 * More information about OpenCorr can be found at https://www.opencorr.org/
 */

#include "oc_deformation.h"

namespace opencorr
{
	//2D deformation with the 1st order shape function
	Deformation2D1::Deformation2D1()
	{
		u = 0.f;
		ux = 0.f;
		uy = 0.f;
		v = 0.f;
		vx = 0.f;
		vy = 0.f;

		setWarp();
	}

	Deformation2D1::Deformation2D1(float u, float ux, float uy, float v, float vx, float vy)
	{
		this->u = u;
		this->ux = ux;
		this->uy = uy;
		this->v = v;
		this->vx = vx;
		this->vy = vy;

		setWarp();
	}

	Deformation2D1::Deformation2D1(float p[6])
	{
		u = p[0];
		ux = p[1];
		uy = p[2];
		v = p[3];
		vx = p[4];
		vy = p[5];

		setWarp();
	}

	Deformation2D1::~Deformation2D1() {}

	void Deformation2D1::setDeformation(float u, float ux, float uy, float v, float vx, float vy)
	{
		this->u = u;
		this->ux = ux;
		this->uy = uy;
		this->v = v;
		this->vx = vx;
		this->vy = vy;

		setWarp();
	}

	void Deformation2D1::setDeformation(float p[6])
	{
		u = p[0];
		ux = p[1];
		uy = p[2];
		v = p[3];
		vx = p[4];
		vy = p[5];

		setWarp();
	}

	void Deformation2D1::setDeformation(Deformation2D1& another_deformation)
	{
		u = another_deformation.u;
		ux = another_deformation.ux;
		uy = another_deformation.uy;
		v = another_deformation.v;
		vx = another_deformation.vx;
		vy = another_deformation.vy;

		setWarp();
	}

	Point2D Deformation2D1::warp(Point2D& location)
	{
		Eigen::Vector3f point_vector;
		point_vector(0) = location.x;
		point_vector(1) = location.y;
		point_vector(2) = 1.f;

		Eigen::Vector3f warped_vector = warp_matrix * point_vector;

		Point2D new_location(warped_vector(0), warped_vector(1));
		return new_location;
	}

	void Deformation2D1::setDeformation()
	{
		u = warp_matrix(0, 2);
		ux = warp_matrix(0, 0) - 1;
		uy = warp_matrix(0, 1);
		v = warp_matrix(1, 2);
		vx = warp_matrix(1, 0);
		vy = warp_matrix(1, 1) - 1;
	}

	void Deformation2D1::setWarp()
	{
		warp_matrix(0, 0) = 1 + ux;
		warp_matrix(0, 1) = uy;
		warp_matrix(0, 2) = u;
		warp_matrix(1, 0) = vx;
		warp_matrix(1, 1) = 1 + vy;
		warp_matrix(1, 2) = v;
		warp_matrix(2, 0) = 0.f;
		warp_matrix(2, 1) = 0.f;
		warp_matrix(2, 2) = 1.f;
	}

	//2D deformation with the 2nd order shape function
	Deformation2D2::Deformation2D2()
	{
		u = 0.f;
		ux = 0.f;
		uy = 0.f;
		uxx = 0.f;
		uxy = 0.f;
		uyy = 0.f;

		v = 0.f;
		vx = 0.f;
		vy = 0.f;
		vxx = 0.f;
		vxy = 0.f;
		vyy = 0.f;

		setWarp();
	}

	Deformation2D2::Deformation2D2(float u, float ux, float uy, float uxx, float uxy, float uyy,
		float v, float vx, float vy, float vxx, float vxy, float vyy)
	{
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

		setWarp();
	}

	Deformation2D2::Deformation2D2(float p[12])
	{
		u = p[0];
		ux = p[1];
		uy = p[2];
		uxx = p[3];
		uxy = p[4];
		uyy = p[5];

		v = p[6];
		vx = p[7];
		vy = p[8];
		vxx = p[9];
		vxy = p[10];
		vyy = p[11];

		setWarp();
	}

	Deformation2D2::~Deformation2D2() {}

	void Deformation2D2::setDeformation(float u, float ux, float uy, float uxx, float uxy, float uyy,
		float v, float vx, float vy, float vxx, float vxy, float vyy)
	{
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

		setWarp();
	}

	void Deformation2D2::setDeformation(float p[12])
	{
		u = p[0];
		ux = p[1];
		uy = p[2];
		uxx = p[3];
		uxy = p[4];
		uyy = p[5];

		v = p[6];
		vx = p[7];
		vy = p[8];
		vxx = p[9];
		vxy = p[10];
		vyy = p[11];

		setWarp();
	}

	void Deformation2D2::setDeformation(Deformation2D2& another_deformation)
	{
		u = another_deformation.u;
		ux = another_deformation.ux;
		uy = another_deformation.uy;
		uxx = another_deformation.uxx;
		uxy = another_deformation.uxy;
		uyy = another_deformation.uyy;

		v = another_deformation.v;
		vx = another_deformation.vx;
		vy = another_deformation.vy;
		vxx = another_deformation.vxx;
		vxy = another_deformation.vxy;
		vyy = another_deformation.vyy;

		setWarp();
	}

	void Deformation2D2::setDeformation(Deformation2D1& another_deformation)
	{
		u = another_deformation.u;
		ux = another_deformation.ux;
		uy = another_deformation.uy;
		uxx = 0.f;
		uxy = 0.f;
		uyy = 0.f;

		v = another_deformation.v;
		vx = another_deformation.vx;
		vy = another_deformation.vy;
		vxx = 0.f;
		vxy = 0.f;
		vyy = 0.f;

		setWarp();
	}

	Point2D Deformation2D2::warp(Point2D location)
	{
		Vector6f point_vector;
		point_vector(0) = location.x * location.x;
		point_vector(1) = location.x * location.y;
		point_vector(2) = location.y * location.y;
		point_vector(3) = location.x;
		point_vector(4) = location.y;
		point_vector(5) = 1.f;

		Vector6f warped_vector = warp_matrix * point_vector;

		Point2D new_location(warped_vector(3), warped_vector(4));
		return new_location;
	}

	void Deformation2D2::setDeformation()
	{
		u = warp_matrix(3, 5);
		ux = warp_matrix(3, 3) - 1.f;
		uy = warp_matrix(3, 4);
		uxx = warp_matrix(3, 0) * 2.f;
		uxy = warp_matrix(3, 1);
		uyy = warp_matrix(3, 2) * 2.f;

		v = warp_matrix(4, 5);
		vx = warp_matrix(4, 3);
		vy = warp_matrix(4, 4) - 1.f;
		vxx = warp_matrix(4, 0) * 2.f;
		vxy = warp_matrix(4, 1);
		vyy = warp_matrix(4, 2) * 2.f;
	}

	void Deformation2D2::setWarp()
	{
		// row 0, S1 - S6
		warp_matrix(0, 0) = 1.f + 2.f * ux + ux * ux + u * uxx;
		warp_matrix(0, 1) = 2.f * u * uxy + 2.f * (1.f + ux) * uy;
		warp_matrix(0, 2) = uy * uy + u * uyy;
		warp_matrix(0, 3) = 2.f * u * (1 + ux);
		warp_matrix(0, 4) = 2.f * u * uy;
		warp_matrix(0, 5) = u * u;

		// row 1, S7 - S12
		warp_matrix(1, 0) = 0.5f * (v * uxx + 2.f * (1.f + ux) * vx + u * vxx);
		warp_matrix(1, 1) = 1.f + uy * vx + ux * vy + v * uxy + u * vxy + vy + ux;
		warp_matrix(1, 2) = 0.5f * (v * uyy + +2.f * uy * (1.f + vy) + u * vyy);
		warp_matrix(1, 3) = v + v * ux + u * vx;
		warp_matrix(1, 4) = u + v * uy + u * vy;
		warp_matrix(1, 5) = u * v;

		// row 2, S12 - S18
		warp_matrix(2, 0) = vx * vx + v * vxx;
		warp_matrix(2, 1) = 2.f * v * vxy + 2.f * vx * (1.f + vy);
		warp_matrix(2, 2) = 1.f + 2.f * vy + vy * vy + v * vyy;
		warp_matrix(2, 3) = 2.f * v * vx;
		warp_matrix(2, 4) = 2.f * v * (1.f + vy);
		warp_matrix(2, 5) = v * v;

		// row 3
		warp_matrix(3, 0) = 0.5f * uxx;
		warp_matrix(3, 1) = uxy;
		warp_matrix(3, 2) = 0.5f * uyy;
		warp_matrix(3, 3) = 1.f + ux;
		warp_matrix(3, 4) = uy;
		warp_matrix(3, 5) = u;

		// row 4
		warp_matrix(4, 0) = 0.5f * vxx;
		warp_matrix(4, 1) = vxy;
		warp_matrix(4, 2) = 0.5f * vyy;
		warp_matrix(4, 3) = vx;
		warp_matrix(4, 4) = 1.f + vy;
		warp_matrix(4, 5) = v;

		// row 5
		warp_matrix(5, 0) = 0.f;
		warp_matrix(5, 1) = 0.f;
		warp_matrix(5, 2) = 0.f;
		warp_matrix(5, 3) = 0.f;
		warp_matrix(5, 4) = 0.f;
		warp_matrix(5, 5) = 1.f;
	}

	//3D deformation with the 1st order shape function
	Deformation3D1::Deformation3D1()
	{
		u = 0.f;
		ux = 0.f;
		uy = 0.f;
		uz = 0.f;

		v = 0.f;
		vx = 0.f;
		vy = 0.f;
		vz = 0.f;

		w = 0.f;
		wx = 0.f;
		wy = 0.f;
		wz = 0.f;

		setWarp();
	}

	Deformation3D1::Deformation3D1(float u, float ux, float uy, float uz,
		float v, float vx, float vy, float vz, float w, float wx, float wy, float wz)
	{
		this->u = u;
		this->ux = ux;
		this->uy = uy;
		this->uz = uz;

		this->v = v;
		this->vx = vx;
		this->vy = vy;
		this->vz = vz;

		this->w = w;
		this->wx = wx;
		this->wy = wy;
		this->wz = wz;

		setWarp();
	}

	Deformation3D1::Deformation3D1(float p[12])
	{
		this->u = p[0];
		this->ux = p[1];
		this->uy = p[2];
		this->uz = p[3];

		this->v = p[4];
		this->vx = p[5];
		this->vy = p[6];
		this->vz = p[7];

		this->w = p[8];
		this->wx = p[9];
		this->wy = p[10];
		this->wz = p[11];

		setWarp();
	}

	Deformation3D1::~Deformation3D1() {}

	void Deformation3D1::setDeformation()
	{
		u = warp_matrix(0, 3);
		ux = warp_matrix(0, 0) - 1;
		uy = warp_matrix(0, 1);
		uz = warp_matrix(0, 2);

		v = warp_matrix(1, 3);
		vx = warp_matrix(1, 0);
		vy = warp_matrix(1, 1) - 1;
		vz = warp_matrix(1, 2);

		w = warp_matrix(2, 3);
		wx = warp_matrix(2, 0);
		wy = warp_matrix(2, 1);
		wz = warp_matrix(2, 2) - 1;
	}

	void Deformation3D1::setDeformation(float u, float ux, float uy, float uz,
		float v, float vx, float vy, float vz, float w, float wx, float wy, float wz)
	{
		this->u = u;
		this->ux = ux;
		this->uy = uy;
		this->uz = uz;

		this->v = v;
		this->vx = vx;
		this->vy = vy;
		this->vz = vz;

		this->w = w;
		this->wx = wx;
		this->wy = wy;
		this->wz = wz;

		setWarp();
	}

	void Deformation3D1::setDeformation(float p[12])
	{
		u = p[0];
		ux = p[1];
		uy = p[2];
		uz = p[3];

		v = p[4];
		vx = p[5];
		vy = p[6];
		vz = p[7];

		w = p[8];
		wx = p[9];
		wy = p[10];
		wz = p[11];

		setWarp();
	}

	void Deformation3D1::setDeformation(Deformation3D1& another_deformation)
	{
		u = another_deformation.u;
		ux = another_deformation.ux;
		uy = another_deformation.uy;
		uz = another_deformation.uz;

		v = another_deformation.v;
		vx = another_deformation.vx;
		vy = another_deformation.vy;
		vz = another_deformation.vz;

		w = another_deformation.w;
		wx = another_deformation.wx;
		wy = another_deformation.wy;
		wz = another_deformation.wz;

		setWarp();
	}

	void Deformation3D1::setWarp()
	{
		warp_matrix(0, 0) = 1 + ux;
		warp_matrix(0, 1) = uy;
		warp_matrix(0, 2) = uz;
		warp_matrix(0, 3) = u;

		warp_matrix(1, 0) = vx;
		warp_matrix(1, 1) = 1 + vy;
		warp_matrix(1, 2) = vz;
		warp_matrix(1, 3) = v;

		warp_matrix(2, 0) = wx;
		warp_matrix(2, 1) = wy;
		warp_matrix(2, 2) = 1 + wz;
		warp_matrix(2, 3) = w;

		warp_matrix(3, 0) = 0.f;
		warp_matrix(3, 1) = 0.f;
		warp_matrix(3, 2) = 0.f;
		warp_matrix(3, 3) = 1.f;
	}

	Point3D Deformation3D1::warp(Point3D& location)
	{
		Eigen::Vector4f point_vector;
		point_vector(0) = location.x;
		point_vector(1) = location.y;
		point_vector(2) = location.z;
		point_vector(3) = 1.f;

		Eigen::Vector4f warped_vector = warp_matrix * point_vector;

		Point3D new_location(warped_vector(0), warped_vector(1), warped_vector(2));
		return new_location;
	}

}//namespace opencorr