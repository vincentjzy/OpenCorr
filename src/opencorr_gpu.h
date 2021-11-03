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
#ifndef _ICGN_GPU_H_
#define _ICGN_GPU_H_

#ifdef LIB_SELF
#define OCAPI __declspec(dllexport)
#else 
#define OCAPI 
#endif

#include <vector>
using std::vector;


namespace opencorr_gpu {

	struct ICGNConfiguration {
		int subset_rx;				//subset radius in x direction
		int subset_ry;				//subset radius in y direction
		int stop_condtion;			//[IC-GN] max iteration steps
		float convergence_criterion;//[IC-GN] max norm of delta_p

		ICGNConfiguration() {
			subset_rx = subset_ry = 15;
			stop_condtion = 10;
			convergence_criterion = 0.001f;
		}
	};


	struct ICGNDeformationVector {
		float u, ux, uy, uxx, uxy, uyy;
		float v, vx, vy, vxx, vxy, vyy;

		ICGNDeformationVector() {
			u = ux = uy = uxx = uxy = uyy = 0.f;
			v = vx = vy = vxx = vxy = vyy = 0.f;
		}
	};


	struct ICGNPOI {
		int x;								//[ input]
		int y;								//[ input]
		int iteration;						//[output][IC-GN] actual iteration steps
		float dpNorm;						//[output][IC-GN] norm of delta_p
		bool convergence;					//[output][IC-GN] is the convergence criterion satisfied
		float ZNCC;							//[output] final value of zero mean normalized cross correlation
		float xw, yw, zw;					//[output][3D RECONSTRUCT]
		ICGNDeformationVector initial;		//[ input] initial deformation vector
		ICGNDeformationVector final;		//[output] final deformation vector

		ICGNPOI(int x, int y) :x(x), y(y) {
			iteration = -1;
			dpNorm = -1.f;
			convergence = false;
			ZNCC = -2.f;
			xw = yw = zw = 0.f;
		}
	};

	struct ICGNImage {
		int w, h;		//width and height of the images to be processed
		float* data;	//row major grayscale data, e.g. if w=10,h=10,x=3,y=1, then grayscale of image at postion (x,y) is : gray(x,y) == data[x + y * w -1] == data[12]

		ICGNImage() :w(0), h(0), data(nullptr) {}
	};

	OCAPI bool ICGN2D1GPU(ICGNImage ref, ICGNImage tar, vector<ICGNPOI>& pois, ICGNConfiguration = ICGNConfiguration());

	OCAPI bool ICGN2D2GPU(ICGNImage ref, ICGNImage tar, vector<ICGNPOI>& pois, ICGNConfiguration = ICGNConfiguration());

}//namespace opencorr_gpu

#endif //_ICGN_GPU_H_