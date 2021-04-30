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

#ifndef _POI_H_
#define _POI_H_

#include <algorithm>
#include "oc_point.h"
#include "oc_deformation.h"

namespace opencorr {

	union DeformationVector2D {
		struct {
			float u, ux, uy, uxx, uxy, uyy;
			float v, vx, vy, vxx, vxy, vyy;
		};
		float p[12]; //content: u ux uy uxx uyy uxy v vx vy vxx vyy vxy
	};

	union StrainVector2D {
		struct {
			float exx, eyy, exy;
		};
		float e[3]; //conetent: exx, eyy, exy
	};

	union Result2D {
		struct {
			float u0, v0, u, v, exx, eyy, exy, ZNCC, iteration, convergence, feature;
		};
		float r[11];
	};

	class POI2D : public Point2D
	{
	public:
		DeformationVector2D deformation;
		Result2D result;

		POI2D(int x, int y);
		POI2D(float x, float y);
		POI2D(Point2D location);
		~POI2D();

		void clean(); //reset data except the location
		//set convergence criterion and stop condition for iterative procedure at a specific POI
		void setIterationCriteria(float conv_criterion, float stop_condition, float neighbor_essential);
	};

	union DeformationVector3D {
		struct {
			float u, ux, uy, uz;
			float v, vx, vy, vz;
			float w, wx, wy, wz;
		};
		float p[12]; //content: u ux uy uxx uyy uxy v vx vy vxx vyy vxy
	};

	union StrainVector3D {
		struct {
			float exx, eyy, ezz;
			float exy, eyz, ezx;
		};
		float e[6]; //conetent: exx, eyy, ezz, exy, eyz, ezx
	};

	union Result3D {
		struct {
			float u0, v0, w0, u, v, w, exx, eyy, ezz, exy, eyz, ezx, ZNCC, iteration, convergence, feature;
		};
		float r[16];
	};

	class POI3D : public Point3D
	{
	public:
		DeformationVector3D deformation;
		Result3D result;

		POI3D(int x, int y, int z);
		POI3D(float x, float y, float z);
		POI3D(Point3D location);
		~POI3D();

		void clean(); //reset data except the location
	};

}//namespace opencorr

#endif //_POI_H_
