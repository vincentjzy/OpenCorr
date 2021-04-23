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

#ifndef  _OUTPUT_H_
#define  _OUTPUT_H_

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <Eigen>

#include "oc_poi.h"

using std::vector;
using std::string;

namespace opencorr
{
	class IO2D {
	private:
		string file_path, delimiter;
		int width;
		int height;

	public:
		IO2D();
		~IO2D();

		string getPath() const;
		string getDilimiter() const;
		int getWidth() const;
		int getHeight() const;
		void setPath(string file_path);
		void setDelimiter(string delimiter);
		void setWidth(int width);
		void setHeight(int height);
		vector<POI2D> loadTable2D();
		void saveTable2D(vector<POI2D> POI_queue);
		void saveDeformationTable2D(vector<POI2D> POI_queue);
		void saveMap2D(vector<POI2D> POI_queue, int variable); //variable is the index of r[11] in Result2D
	};
}//namespace opencorr

#endif //_OUTPUT_H_