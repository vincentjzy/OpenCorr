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

#pragma once

#ifndef _POI_H_
#define _POI_H_

#include <algorithm>
#include "oc_point.h"
#include "oc_deformation.h"

namespace opencorr {

	//structures for POI
	union DeformationVector2D {
		struct {
			float u, ux, uy, uxx, uxy, uyy;
			float v, vx, vy, vxx, vxy, vyy;
		};
		//content: u ux uy uxx uxy uyy v vx vy vxx vxy vyy
		float p[12];
	};

	union StrainVector2D {
		struct {
			float exx, eyy, exy;
		};
		//conetent: exx, eyy, exy
		float e[3];
	};

	union Result2D {
		struct {
			float u0, v0, zncc, iteration, convergence, feature;
		};
		float r[6];
	};

	union DeformationVector3D {
		struct {
			float u, ux, uy, uz;
			float v, vx, vy, vz;
			float w, wx, wy, wz;
		};
		//content: u ux uy uz v vx vy vz w wx wy wz
		float p[12];
	};

	union DisplacementVector3D {
		struct {
			float u, v, w;
		};
		float p[3];
	};

	union StrainVector3D {
		struct {
			float exx, eyy, ezz;
			float exy, eyz, ezx;
		};
		//conetent: exx, eyy, ezz, exy, eyz, ezx
		float e[6];
	};

	union Result3D {
		struct {
			float u0, v0, w0, zncc, iteration, convergence, feature;
		};
		float r[7];
	};

	union Result2DS {
		struct {
			float r1r2_zncc, r1t1_zncc, r1t2_zncc, r2_x, r2_y, t1_x, t1_y, t2_x, t2_y;
		};
		float r[9];
	};

	//class for 2D DIC
	class POI2D : public Point2D
	{
	public:
		DeformationVector2D deformation;
		Result2D result;
		StrainVector2D strain;

		POI2D(int x, int y);
		POI2D(float x, float y);
		POI2D(Point2D location);
		~POI2D();

		//reset data except the location
		void clean();
		//set convergence criterion and stop condition for iterative procedure at a specific POI
		void setIterationCriteria(float conv_criterion, float stop_condition, float neighbor_kp_min);
	};


	//class for stereo DIC
	class POI2DS : public Point2D
	{
	public:
		DisplacementVector3D deformation;
		Result2DS result;
		Point3D ref_coor, tar_coor;
		StrainVector3D strain;

		POI2DS(int x, int y);
		POI2DS(float x, float y);
		POI2DS(Point2D location);
		~POI2DS();

		//reset data except the location
		void clean();
	};


	//class for DVC
	class POI3D : public Point3D
	{
	public:
		DeformationVector3D deformation;
		Result3D result;
		StrainVector3D strain;

		POI3D(int x, int y, int z);
		POI3D(float x, float y, float z);
		POI3D(Point3D location);
		~POI3D();

		//reset data except the location
		void clean();
	};

}//namespace opencorr

#endif //_POI_H_
