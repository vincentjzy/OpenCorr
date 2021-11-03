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

#ifndef _FEATURE_H_
#define _FEATURE_H_
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "oc_dic.h"
#include "oc_image.h"
#include "oc_poi.h"
#include "oc_point.h"
#include "oc_subset.h"

namespace opencorr
{
	class Feature2D
	{
	protected:
		Image2D* ref_img = nullptr;
		Image2D* tar_img = nullptr;

	public:
		virtual ~Feature2D() = default;

		void setImages(Image2D& ref_img, Image2D& tar_img);
		virtual void prepare() = 0;
		virtual void compute() = 0;

	};

}//namespace opencorr

#endif //_FEATURE_H_

