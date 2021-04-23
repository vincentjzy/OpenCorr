/*
 * This file is part of OpenCorr, an open-source C++ library for
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

#ifndef _DIC_H_
#define _DIC_H_

#include "oc_array.h"
#include "oc_image.h"
#include "oc_poi.h"
#include "oc_subset.h"

namespace opencorr
{
	class DIC
	{
	protected:
		Image2D* ref_img = nullptr;
		Image2D* tar_img = nullptr;

	public:
		int subset_radius_x, subset_radius_y;
		int CPU_thread_number; //number of CPU threads

		DIC();
		void setImages(Image2D& ref_img, Image2D& tar_img);
		virtual void prepare();
		virtual void compute(POI2D* POI) = 0;
		virtual ~DIC() = default;

		void setThreadNumber(int CPU_thread_number);

	};


	class DVC
	{
	protected:
		Image3D* ref_img;
		Image3D* tar_img;

	public:
		int subvolume_radius_x, subvolume_radius_y, subvolume_radius_z;
		int CPU_thread_number; //number of CPU threads
		//		virtual void setImages(image3d& ref_img, image3d& tar_img) = 0;
		//		virtual void prepare() = 0;
		//		virtual void compute(POI3d* POI) = 0;
		//		virtual ~DVC() = default;
	};

}//namespace opencorr

#endif //_DIC_H_
