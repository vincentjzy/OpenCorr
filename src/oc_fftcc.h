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

#ifndef _FFTCC_H_
#define _FFTCC_H_

#include <vector>
#include "fftw3.h"

#include "oc_array.h"
#include "oc_dic.h"
#include "oc_image.h"
#include "oc_poi.h"
#include "oc_point.h"
#include "oc_subset.h"

namespace opencorr {

	class FFTW {
	public:
		float* ref_subset;
		float* tar_subset;
		float* zncc;
		fftwf_complex* ref_freq;
		fftwf_complex* tar_freq;
		fftwf_complex* zncc_freq;
		fftwf_plan ref_plan;
		fftwf_plan tar_plan;
		fftwf_plan zncc_plan;

		static FFTW* allocate(int subset_radius_x, int subset_radius_y);
		static void release(FFTW* instance);

	};

	class FFTCC2D : public DIC {
	private:
		//pool of FFTW instances for multi-thread processing
		std::vector<FFTW*> instance_pool;
		FFTW* getInstance(int tid);

	public:
		FFTCC2D(int subset_radius_x, int subset_radius_y, int thread_number);
		~FFTCC2D();

		void compute(POI2D* poi);
		void compute(std::vector<POI2D>& poi_queue);
	};

}//namespace opencorr

#endif //_FFTCC_H_
