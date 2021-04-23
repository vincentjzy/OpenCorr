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

#include "oc_dic.h"

namespace opencorr
{
	DIC::DIC() {
		this->CPU_thread_number = 1;
	}

	void DIC::setImages(Image2D& ref_img, Image2D& tar_img) {
		this->ref_img = &ref_img;
		this->tar_img = &tar_img;
	}

	void DIC::setThreadNumber(int CPU_thread_number) {
		this->CPU_thread_number = CPU_thread_number;
	}

	void DIC::prepare() {
	}
}
