/*
 * This file is part of OpenCorr, an open source C++ library for
 * study and development of 2D, 3D/stereo and volumetric
 * digital image correlation.
 *
 * Copyright (C) 2021-2025, Zhenyu Jiang <zhenyujiang@scut.edu.cn>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one from http://mozilla.org/MPL/2.0/.
 *
 * More information about OpenCorr can be found at https://www.opencorr.org/
 */

#pragma once

#ifndef _DEFORMATION_H_
#define _DEFORMATION_H_

#include "oc_array.h"
#include "oc_point.h"

namespace opencorr
{
	//2D deformation with the 1st order shape function
	class Deformation2D1
	{
	public:
		float u, ux, uy;
		float v, vx, vy;
		Eigen::Matrix3f warp_matrix;

		Deformation2D1();
		Deformation2D1(float u, float ux, float uy, float v, float vx, float vy);
		Deformation2D1(float p[6]);
		~Deformation2D1();

		void setDeformation(); // set deformation according to warp_matrix
		void setDeformation(float u, float ux, float uy, float v, float vx, float vy);
		void setDeformation(float p[6]);
		void setDeformation(Deformation2D1& another_deformation);

		void setWarp(); //update warp_matrix according to deformation
		Point2D warp(Point2D& location);
	};

	//2D deformation with the 2nd order shape function
	class Deformation2D2
	{
	public:
		float u, ux, uy, uxx, uxy, uyy;
		float v, vx, vy, vxx, vxy, vyy;
		Matrix6f warp_matrix;

		Deformation2D2();
		Deformation2D2(float u, float ux, float uy, float uxx, float uxy, float uyy,
			float v, float vx, float vy, float vxx, float vxy, float vyy);
		Deformation2D2(float p[12]);
		~Deformation2D2();

		void setDeformation(); //set deformation according to warp_matrix
		void setDeformation(float u, float ux, float uy, float uxx, float uxy, float uyy,
			float v, float vx, float vy, float vxx, float vxy, float vyy);
		void setDeformation(float p[12]);
		void setDeformation(Deformation2D2& another_deformation);
		void setDeformation(Deformation2D1& another_deformation);

		void setWarp(); //update warp_matrix according to deformation
		Point2D warp(Point2D location);
	};

	//3D deformation with the 1st order shape function
	class Deformation3D1
	{
	public:
		float u, ux, uy, uz;
		float v, vx, vy, vz;
		float w, wx, wy, wz;
		Eigen::Matrix4f warp_matrix;

		Deformation3D1();
		Deformation3D1(float u, float ux, float uy, float uz,
			float v, float vx, float vy, float vz,
			float w, float wx, float wy, float wz);
		Deformation3D1(float p[12]);
		~Deformation3D1();

		void setDeformation(); //set deformation according to warp_matrix
		void setDeformation(float u, float ux, float uy, float uz,
			float v, float vx, float vy, float vz,
			float w, float wx, float wy, float wz);
		void setDeformation(float p[12]);
		void setDeformation(Deformation3D1& another_deformation);

		void setWarp(); //update warp_matrix according to deformation
		Point3D warp(Point3D& point);
	};

}//namespace opencorr

#endif // _DEFORMATION_H_