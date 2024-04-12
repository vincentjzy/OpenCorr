/*
 * This file is part of OpenCorr, an open source C++ library for
 * study and development of 2D, 3D/stereo and volumetric
 * digital image correlation.
 *
 * Copyright (C) 2021-2024, Zhenyu Jiang <zhenyujiang@scut.edu.cn>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one from http://mozilla.org/MPL/2.0/.
 *
 * More information about OpenCorr can be found at https://www.opencorr.org/
 */

#include <fstream>
#include <iomanip>

#include "oc_io.h"

namespace opencorr
{
	IO2D::IO2D() {}

	IO2D::~IO2D() {}

	string IO2D::getPath() const
	{
		return file_path;
	}

	string IO2D::getDelimiter() const
	{
		return delimiter;
	}

	int IO2D::getWidth() const
	{
		return width;
	}

	int IO2D::getHeight()const
	{
		return height;
	}

	void IO2D::setPath(string file_path)
	{
		this->file_path = file_path;
	}

	void IO2D::setDelimiter(string delimiter)
	{
		this->delimiter = delimiter;
	}

	void IO2D::setWidth(int width)
	{
		this->width = width;
	}

	void IO2D::setHeight(int height)
	{
		this->height = height;
	}

	vector<POI2D> IO2D::loadTable2D()
	{
		std::ifstream file_in(file_path);
		if (!file_in)
		{
			std::cerr << "failed to read file " << file_path << std::endl;
		}

		string data_line;
		getline(file_in, data_line);
		vector<POI2D> poi_queue;
		size_t position1, position2;
		string variable;
		vector<float> key_buffer;

		while (getline(file_in, data_line))
		{
			position1 = 0;
			position2 = 0;
			std::vector<float>().swap(key_buffer);
			do
			{
				position2 = data_line.find(delimiter, position1);
				if (position2 == string::npos)
				{
					position2 = data_line.length();
				}

				variable = data_line.substr(position1, position2 - position1);
				if (!variable.empty())
				{
					key_buffer.push_back(std::stof(variable));
				}

				position1 = position2 + delimiter.length();
			} while (position2 < data_line.length() && position1 < data_line.length());

			float x = key_buffer[0];
			float y = key_buffer[1];
			POI2D current_POI(x, y);

			current_POI.deformation.u = key_buffer[2];
			current_POI.deformation.v = key_buffer[3];

			int current_index = 4;
			int array_size = (int)(sizeof(current_POI.result.r) / sizeof(current_POI.result.r[0]));
			for (int i = 0; i < array_size; i++)
			{
				current_POI.result.r[i] = key_buffer[current_index + i];
			}

			current_index += array_size;
			array_size = (int)(sizeof(current_POI.strain.e) / sizeof(current_POI.strain.e[0]));
			for (int i = 0; i < array_size; i++)
			{
				current_POI.strain.e[i] = key_buffer[current_index + i];
			}

			poi_queue.push_back(current_POI);
		}
		file_in.close();

		return poi_queue;
	}

	vector<Point2D> IO2D::loadPoint2D(string file_path)
	{
		std::ifstream file_in(file_path);
		if (!file_in)
		{
			std::cerr << "failed to read file " << file_path << std::endl;
		}

		string data_line;
		getline(file_in, data_line);
		vector<Point2D> point_queue;
		size_t position1, position2;
		string variable;
		vector<float> key_buffer;
		int point_number = 0;

		while (getline(file_in, data_line))
		{
			point_number++;
			position1 = 0;
			position2 = 0;
			std::vector<float>().swap(key_buffer);

			position2 = data_line.find(delimiter, position1);
			if (position2 == string::npos)
			{
				std::cerr << "failed to read POI at line: " << point_number << std::endl;
			}

			variable = data_line.substr(position1, position2 - position1);
			if (!variable.empty())
			{
				key_buffer.push_back(std::stof(variable));
			}

			position1 = position2 + delimiter.length();
			position2 = data_line.length();
			variable = data_line.substr(position1, position2 - position1);
			if (!variable.empty())
			{
				key_buffer.push_back(std::stof(variable));
			}

			float x = key_buffer[0];
			float y = key_buffer[1];
			Point2D current_point(x, y);

			point_queue.push_back(current_point);
		}
		file_in.close();

		return point_queue;
	}

	void IO2D::saveTable2D(vector<POI2D>& poi_queue)
	{
		std::ofstream file_out(file_path);
		file_out.setf(std::ios::fixed);
		file_out << std::setprecision(8);

		if (file_out.is_open())
		{
			file_out << "x" << delimiter;
			file_out << "y" << delimiter;

			file_out << "u" << delimiter;
			file_out << "v" << delimiter;

			file_out << "u0" << delimiter;
			file_out << "v0" << delimiter;
			file_out << "ZNCC" << delimiter;
			file_out << "iteration" << delimiter;
			file_out << "convergence" << delimiter;
			file_out << "feature" << delimiter;

			file_out << "exx" << delimiter;
			file_out << "eyy" << delimiter;
			file_out << "exy" << delimiter;

			file_out << "subset_rx" << delimiter;
			file_out << "subset_ry" << delimiter;
			file_out << std::endl;

			for (vector<POI2D>::iterator iter = poi_queue.begin(); iter != poi_queue.end(); iter++)
			{
				file_out << iter->x << delimiter;
				file_out << iter->y << delimiter;

				file_out << iter->deformation.u << delimiter;
				file_out << iter->deformation.v << delimiter;

				int array_size = (int)(sizeof(iter->result.r) / sizeof(iter->result.r[0]));
				for (int i = 0; i < array_size; i++)
				{
					file_out << iter->result.r[i] << delimiter;
				}

				array_size = (int)(sizeof(iter->strain.e) / sizeof(iter->strain.e[0]));
				for (int i = 0; i < array_size; i++)
				{
					file_out << iter->strain.e[i] << delimiter;
				}

				file_out << iter->subset_radius.x << delimiter;
				file_out << iter->subset_radius.y << delimiter;
				file_out << std::endl;
			}
		}
		file_out.close();
	}

	void IO2D::saveDeformationTable2D(vector<POI2D>& poi_queue)
	{
		std::ofstream file_out(file_path);
		file_out.setf(std::ios::fixed);
		file_out << std::setprecision(8);

		if (file_out.is_open())
		{
			file_out << "x" << delimiter;
			file_out << "y" << delimiter;

			file_out << "u" << delimiter;
			file_out << "ux" << delimiter;
			file_out << "uy" << delimiter;
			file_out << "uxx" << delimiter;
			file_out << "uxy" << delimiter;
			file_out << "uyy" << delimiter;

			file_out << "v" << delimiter;
			file_out << "vx" << delimiter;
			file_out << "vy" << delimiter;
			file_out << "vxx" << delimiter;
			file_out << "vxy" << delimiter;
			file_out << "vyy" << delimiter;

			file_out << "subset_rx" << delimiter;
			file_out << "subset_ry" << delimiter;
			file_out << std::endl;

			for (vector<POI2D>::iterator iter = poi_queue.begin(); iter != poi_queue.end(); iter++)
			{
				file_out << iter->x << delimiter;
				file_out << iter->y << delimiter;

				int array_size = (int)(sizeof(iter->deformation.p) / sizeof(iter->deformation.p[0]));
				for (int i = 0; i < array_size; i++)
				{
					file_out << iter->deformation.p[i] << delimiter;
				}
				file_out << iter->subset_radius.x << delimiter;
				file_out << iter->subset_radius.y << delimiter;
				file_out << std::endl;
			}
		}
		file_out.close();
	}

	void IO2D::saveMap2D(vector<POI2D>& poi_queue, char variable)
	{
		int height = getHeight();
		int width = getWidth();
		Eigen::MatrixXf output_map = Eigen::MatrixXf::Zero(height, width);

		switch (variable)
		{
		case 'u':
			for (int i = 0; i < poi_queue.size(); i++)
			{
				output_map((int)poi_queue[i].y, (int)poi_queue[i].x) = poi_queue[i].deformation.u;
			}
			break;
		case 'v':
			for (int i = 0; i < poi_queue.size(); i++)
			{
				output_map((int)poi_queue[i].y, (int)poi_queue[i].x) = poi_queue[i].deformation.v;
			}
			break;
		case 'c': //ZNCC value
			for (int i = 0; i < poi_queue.size(); i++)
			{
				output_map((int)poi_queue[i].y, (int)poi_queue[i].x) = poi_queue[i].result.zncc;
			}
			break;
		case 'd': //final ||delta_p||
			for (int i = 0; i < poi_queue.size(); i++)
			{
				output_map((int)poi_queue[i].y, (int)poi_queue[i].x) = poi_queue[i].result.convergence;
			}
			break;
		case 'i': //iteration steps
			for (int i = 0; i < poi_queue.size(); i++)
			{
				output_map((int)poi_queue[i].y, (int)poi_queue[i].x) = poi_queue[i].result.iteration;
			}
			break;
		case 'f': //number of neighbor features
			for (int i = 0; i < poi_queue.size(); i++)
			{
				output_map((int)poi_queue[i].y, (int)poi_queue[i].x) = poi_queue[i].result.feature;
			}
			break;
		case 'x': //strain exx
			for (int i = 0; i < poi_queue.size(); i++)
			{
				output_map((int)poi_queue[i].y, (int)poi_queue[i].x) = poi_queue[i].strain.exx;
			}
			break;
		case 'y': //strain eyy
			for (int i = 0; i < poi_queue.size(); i++)
			{
				output_map((int)poi_queue[i].y, (int)poi_queue[i].x) = poi_queue[i].strain.eyy;
			}
			break;
		case 'r': //strain exy
			for (int i = 0; i < poi_queue.size(); i++)
			{
				output_map((int)poi_queue[i].y, (int)poi_queue[i].x) = poi_queue[i].strain.exy;
			}
			break;
		default:
			return;
		}

		std::ofstream file_out(file_path);
		file_out.setf(std::ios::fixed);
		file_out << std::setprecision(8);
		if (file_out.is_open())
		{
			for (int r = 0; r < height; r++)
			{
				for (int c = 0; c < width; c++)
				{
					file_out << output_map(r, c) << delimiter;
				}
				file_out << std::endl;
			}
		}
		file_out.close();
	}

	vector<POI2DS> IO2D::loadTable2DS()
	{
		std::ifstream file_in(file_path);
		if (!file_in)
		{
			std::cerr << "failed to read file " << file_path << std::endl;
		}

		string data_line;
		getline(file_in, data_line);
		vector<POI2DS> poi_queue;
		size_t position1, position2;
		string variable;
		vector<float> key_buffer;

		while (getline(file_in, data_line))
		{
			position1 = 0;
			position2 = 0;
			vector<float>().swap(key_buffer);
			do
			{
				position2 = data_line.find(delimiter, position1);
				if (position2 == string::npos)
				{
					position2 = data_line.length();
				}

				variable = data_line.substr(position1, position2 - position1);
				if (!variable.empty())
				{
					key_buffer.push_back(std::stof(variable));
				}

				position1 = position2 + delimiter.length();
			} while (position2 < data_line.length() && position1 < data_line.length());

			float x = key_buffer[0];
			float y = key_buffer[1];
			POI2DS current_POI(x, y);

			int current_index = 2;
			int array_size = (int)(sizeof(current_POI.deformation.p) / sizeof(current_POI.deformation.p[0]));
			for (int i = 0; i < array_size; i++)
			{
				current_POI.deformation.p[i] = key_buffer[current_index + i];
			}

			current_index += array_size;
			array_size = (int)(sizeof(current_POI.result.r) / sizeof(current_POI.result.r[0]));
			for (int i = 0; i < array_size; i++)
			{
				current_POI.result.r[i] = key_buffer[current_index + i];
			}

			current_index += array_size;
			current_POI.ref_coor.x = key_buffer[current_index];
			current_POI.ref_coor.y = key_buffer[current_index + 1];
			current_POI.ref_coor.z = key_buffer[current_index + 2];

			current_POI.tar_coor.x = key_buffer[current_index + 3];
			current_POI.tar_coor.y = key_buffer[current_index + 4];
			current_POI.tar_coor.z = key_buffer[current_index + 5];

			current_index += 6;
			array_size = (int)(sizeof(current_POI.strain.e) / sizeof(current_POI.strain.e[0]));
			for (int i = 0; i < array_size; i++)
			{
				current_POI.strain.e[i] = key_buffer[current_index + i];
			}

			poi_queue.push_back(current_POI);
		}
		file_in.close();

		return poi_queue;
	}

	void IO2D::saveTable2DS(vector<POI2DS>& poi_queue)
	{
		std::ofstream file_out(file_path);
		file_out.setf(std::ios::fixed);
		file_out << std::setprecision(8);

		if (file_out.is_open())
		{
			file_out << "x" << delimiter;
			file_out << "y" << delimiter;

			file_out << "u" << delimiter;
			file_out << "v" << delimiter;
			file_out << "w" << delimiter;

			file_out << "r1r2 ZNCC" << delimiter;
			file_out << "r1t1 ZNCC" << delimiter;
			file_out << "r1t2 ZNCC" << delimiter;

			file_out << "r2_x" << delimiter;
			file_out << "r2_y" << delimiter;
			file_out << "t1_x" << delimiter;
			file_out << "t1_y" << delimiter;
			file_out << "t2_x" << delimiter;
			file_out << "t2_y" << delimiter;

			file_out << "ref_x" << delimiter;
			file_out << "ref_y" << delimiter;
			file_out << "ref_z" << delimiter;
			file_out << "tar_x" << delimiter;
			file_out << "tar_y" << delimiter;
			file_out << "tar_z" << delimiter;

			file_out << "exx" << delimiter;
			file_out << "eyy" << delimiter;
			file_out << "ezz" << delimiter;
			file_out << "exy" << delimiter;
			file_out << "eyz" << delimiter;
			file_out << "ezx" << delimiter;

			file_out << "subset_rx" << delimiter;
			file_out << "subset_ry" << delimiter;
			file_out << std::endl;

			for (vector<POI2DS>::iterator iter = poi_queue.begin(); iter != poi_queue.end(); iter++)
			{
				file_out << iter->x << delimiter;
				file_out << iter->y << delimiter;

				int array_size = (int)(sizeof(iter->deformation.p) / sizeof(iter->deformation.p[0]));
				for (int i = 0; i < array_size; i++)
				{
					file_out << iter->deformation.p[i] << delimiter;
				}

				array_size = (int)(sizeof(iter->result.r) / sizeof(iter->result.r[0]));
				for (int i = 0; i < array_size; i++)
				{
					file_out << iter->result.r[i] << delimiter;
				}

				file_out << iter->ref_coor.x << delimiter;
				file_out << iter->ref_coor.y << delimiter;
				file_out << iter->ref_coor.z << delimiter;

				file_out << iter->tar_coor.x << delimiter;
				file_out << iter->tar_coor.y << delimiter;
				file_out << iter->tar_coor.z << delimiter;

				array_size = (int)(sizeof(iter->strain.e) / sizeof(iter->strain.e[0]));
				for (int i = 0; i < array_size; i++)
				{
					file_out << iter->strain.e[i] << delimiter;
				}
				file_out << iter->subset_radius.x << delimiter;
				file_out << iter->subset_radius.y << delimiter;
				file_out << std::endl;
			}
		}
		file_out.close();
	}

	void IO2D::saveMap2DS(vector<POI2DS>& poi_queue, char variable)
	{
		int height = getHeight();
		int width = getWidth();
		Eigen::MatrixXf output_map = Eigen::MatrixXf::Zero(height, width);

		switch (variable)
		{
		case 'u':
			for (int i = 0; i < poi_queue.size(); i++)
			{
				output_map((int)poi_queue[i].y, (int)poi_queue[i].x) = poi_queue[i].deformation.u;
			}
			break;
		case 'v':
			for (int i = 0; i < poi_queue.size(); i++)
			{
				output_map((int)poi_queue[i].y, (int)poi_queue[i].x) = poi_queue[i].deformation.v;
			}
			break;
		case 'w':
			for (int i = 0; i < poi_queue.size(); i++)
			{
				output_map((int)poi_queue[i].y, (int)poi_queue[i].x) = poi_queue[i].deformation.w;
			}
			break;
		case 'c': //ZNCC value in matching between the two reference images
			for (int i = 0; i < poi_queue.size(); i++)
			{
				output_map((int)poi_queue[i].y, (int)poi_queue[i].x) = poi_queue[i].result.r1r2_zncc;
			}
			break;
		case 'd': //ZNCC value in matching between the reference image and the target image from same view
			for (int i = 0; i < poi_queue.size(); i++)
			{
				output_map((int)poi_queue[i].y, (int)poi_queue[i].x) = poi_queue[i].result.r1t1_zncc;
			}
			break;
		case 'e': //ZNCC value in matching between the reference image and the target image from different views
			for (int i = 0; i < poi_queue.size(); i++)
			{
				output_map((int)poi_queue[i].y, (int)poi_queue[i].x) = poi_queue[i].result.r1t2_zncc;
			}
			break;
		case 'x': //strain exx
			for (int i = 0; i < poi_queue.size(); i++)
			{
				output_map((int)poi_queue[i].y, (int)poi_queue[i].x) = poi_queue[i].strain.exx;
			}
			break;
		case 'y': //strain eyy
			for (int i = 0; i < poi_queue.size(); i++)
			{
				output_map((int)poi_queue[i].y, (int)poi_queue[i].x) = poi_queue[i].strain.eyy;
			}
			break;
		case 'z': //strain ezz
			for (int i = 0; i < poi_queue.size(); i++)
			{
				output_map((int)poi_queue[i].y, (int)poi_queue[i].x) = poi_queue[i].strain.ezz;
			}
			break;
		case 'r': //strain exy
			for (int i = 0; i < poi_queue.size(); i++)
			{
				output_map((int)poi_queue[i].y, (int)poi_queue[i].x) = poi_queue[i].strain.exy;
			}
			break;
		case 's': //strain eyz
			for (int i = 0; i < poi_queue.size(); i++)
			{
				output_map((int)poi_queue[i].y, (int)poi_queue[i].x) = poi_queue[i].strain.eyz;
			}
			break;
		case 't': //strain ezx
			for (int i = 0; i < poi_queue.size(); i++)
			{
				output_map((int)poi_queue[i].y, (int)poi_queue[i].x) = poi_queue[i].strain.ezx;
			}
			break;
		default:
			return;
		}

		std::ofstream file_out(file_path);
		file_out.setf(std::ios::fixed);
		file_out << std::setprecision(8);

		if (file_out.is_open())
		{
			for (int r = 0; r < height; r++)
			{
				for (int c = 0; c < width; c++)
				{
					file_out << output_map(r, c) << delimiter;
				}
				file_out << std::endl;
			}
		}
		file_out.close();
	}


	IO3D::IO3D() {}

	IO3D::~IO3D() {}

	string IO3D::getPath() const
	{
		return file_path;
	}

	string IO3D::getDelimiter() const
	{
		return delimiter;
	}

	void IO3D::setPath(string file_path)
	{
		this->file_path = file_path;
	}

	void IO3D::setDelimiter(string delimiter)
	{
		this->delimiter = delimiter;
	}

	int IO3D::getDimX()
	{
		return dim_x;
	}

	int IO3D::getDimY()
	{
		return dim_y;
	}

	int IO3D::getDimZ()
	{
		return dim_z;
	}

	void IO3D::setDimX(int dim_x)
	{
		this->dim_x = dim_x;
	}

	void IO3D::setDimY(int dim_y)
	{
		this->dim_y = dim_y;
	}

	void IO3D::setDimZ(int dim_z)
	{
		this->dim_z = dim_z;
	}

	vector<POI3D> IO3D::loadTable3D()
	{
		std::ifstream file_in(file_path);
		if (!file_in)
		{
			std::cerr << "failed to read file " << file_path << std::endl;
		}

		string data_line;
		getline(file_in, data_line);
		vector<POI3D> poi_queue;
		size_t position1, position2;
		string variable;
		vector<float> key_buffer;

		while (getline(file_in, data_line))
		{
			position1 = 0;
			position2 = 0;
			vector<float>().swap(key_buffer);
			do
			{
				position2 = data_line.find(delimiter, position1);
				if (position2 == string::npos)
				{
					position2 = data_line.length();
				}

				variable = data_line.substr(position1, position2 - position1);
				if (!variable.empty())
				{
					key_buffer.push_back(std::stof(variable));
				}

				position1 = position2 + delimiter.length();
			} while (position2 < data_line.length() && position1 < data_line.length());

			float x = key_buffer[0];
			float y = key_buffer[1];
			float z = key_buffer[2];
			POI3D current_POI(x, y, z);

			current_POI.deformation.u = key_buffer[3];
			current_POI.deformation.v = key_buffer[4];
			current_POI.deformation.w = key_buffer[5];

			int current_index = 6;
			int array_size = (int)(sizeof(current_POI.result.r) / sizeof(current_POI.result.r[0]));
			for (int i = 0; i < array_size; i++)
			{
				current_POI.result.r[i] = key_buffer[current_index + i];
			}

			current_index += array_size;
			current_POI.deformation.ux = key_buffer[current_index];
			current_POI.deformation.uy = key_buffer[current_index + 1];
			current_POI.deformation.uz = key_buffer[current_index + 2];
			current_POI.deformation.vx = key_buffer[current_index + 3];
			current_POI.deformation.vy = key_buffer[current_index + 4];
			current_POI.deformation.vz = key_buffer[current_index + 5];
			current_POI.deformation.wx = key_buffer[current_index + 6];
			current_POI.deformation.wy = key_buffer[current_index + 7];
			current_POI.deformation.wz = key_buffer[current_index + 8];
			array_size = 9;

			current_index += array_size;
			array_size = (int)(sizeof(current_POI.strain.e) / sizeof(current_POI.strain.e[0]));
			for (int i = 0; i < array_size; i++)
			{
				current_POI.strain.e[i] = key_buffer[current_index + i];
			}

			current_index += array_size;
			current_POI.subset_radius.x = key_buffer[current_index];
			current_POI.subset_radius.y = key_buffer[current_index + 1];
			current_POI.subset_radius.z = key_buffer[current_index + 2];

			poi_queue.push_back(current_POI);
		}
		file_in.close();

		return poi_queue;
	}

	vector<Point3D> IO3D::loadPoint3D(string file_path)
	{
		std::ifstream file_in(file_path);
		if (!file_in)
		{
			std::cerr << "failed to read file " << file_path << std::endl;
		}

		string data_line;
		getline(file_in, data_line);
		vector<Point3D> point_queue;
		size_t position1, position2;
		string variable;
		vector<float> key_buffer;
		int point_number = 0;

		while (getline(file_in, data_line))
		{
			point_number++;
			position1 = 0;
			position2 = 0;
			vector<float>().swap(key_buffer);

			//get x
			position2 = data_line.find(delimiter, position1);
			if (position2 == string::npos)
			{
				std::cerr << "failed to read POI at line: " << point_number << std::endl;
			}

			variable = data_line.substr(position1, position2 - position1);
			if (!variable.empty())
			{
				key_buffer.push_back(std::stof(variable));
			}

			//get y
			position1 = position2 + delimiter.length();
			position2 = data_line.find(delimiter, position1);
			variable = data_line.substr(position1, position2 - position1);
			if (!variable.empty())
			{
				key_buffer.push_back(std::stof(variable));
			}

			//get z
			position1 = position2 + delimiter.length();
			position2 = data_line.length();
			variable = data_line.substr(position1, position2 - position1);
			if (!variable.empty())
			{
				key_buffer.push_back(std::stof(variable));
			}

			float x = key_buffer[0];
			float y = key_buffer[1];
			float z = key_buffer[2];
			Point3D current_point(x, y, z);

			point_queue.push_back(current_point);
		}
		file_in.close();

		return point_queue;
	}

	void IO3D::saveTable3D(vector<POI3D>& poi_queue)
	{
		std::ofstream file_out(file_path);
		file_out.setf(std::ios::fixed);
		file_out << std::setprecision(8);

		if (file_out.is_open())
		{
			file_out << "x" << delimiter;
			file_out << "y" << delimiter;
			file_out << "z" << delimiter;

			file_out << "u" << delimiter;
			file_out << "v" << delimiter;
			file_out << "w" << delimiter;

			file_out << "u0" << delimiter;
			file_out << "v0" << delimiter;
			file_out << "w0" << delimiter;
			file_out << "ZNCC" << delimiter;
			file_out << "iteration" << delimiter;
			file_out << "convergence" << delimiter;
			file_out << "feature" << delimiter;

			file_out << "ux" << delimiter;
			file_out << "uy" << delimiter;
			file_out << "uz" << delimiter;
			file_out << "vx" << delimiter;
			file_out << "vy" << delimiter;
			file_out << "vz" << delimiter;
			file_out << "wx" << delimiter;
			file_out << "wy" << delimiter;
			file_out << "wz" << delimiter;

			file_out << "exx" << delimiter;
			file_out << "eyy" << delimiter;
			file_out << "ezz" << delimiter;
			file_out << "exy" << delimiter;
			file_out << "eyz" << delimiter;
			file_out << "ezx" << delimiter;

			file_out << "subset_rx" << delimiter;
			file_out << "subset_ry" << delimiter;
			file_out << "subset_rz" << delimiter;
			file_out << std::endl;

			for (vector<POI3D>::iterator iter = poi_queue.begin(); iter != poi_queue.end(); iter++)
			{
				file_out << iter->x << delimiter;
				file_out << iter->y << delimiter;
				file_out << iter->z << delimiter;

				file_out << iter->deformation.u << delimiter;
				file_out << iter->deformation.v << delimiter;
				file_out << iter->deformation.w << delimiter;

				int array_size = (int)(sizeof(iter->result.r) / sizeof(iter->result.r[0]));
				for (int i = 0; i < array_size; i++)
				{
					file_out << iter->result.r[i] << delimiter;
				}

				file_out << iter->deformation.ux << delimiter;
				file_out << iter->deformation.uy << delimiter;
				file_out << iter->deformation.uz << delimiter;
				file_out << iter->deformation.vx << delimiter;
				file_out << iter->deformation.vy << delimiter;
				file_out << iter->deformation.vz << delimiter;
				file_out << iter->deformation.wx << delimiter;
				file_out << iter->deformation.wy << delimiter;
				file_out << iter->deformation.wz << delimiter;

				array_size = (int)(sizeof(iter->strain.e) / sizeof(iter->strain.e[0]));
				for (int i = 0; i < array_size; i++)
				{
					file_out << iter->strain.e[i] << delimiter;
				}

				file_out << iter->subset_radius.x << delimiter;
				file_out << iter->subset_radius.y << delimiter;
				file_out << iter->subset_radius.z << delimiter;
				file_out << std::endl;
			}
		}
		file_out.close();
	}

	void IO3D::saveMap3D(vector<POI3D>& poi_queue, char variable)
	{
		int queue_length = (int)poi_queue.size();
		float*** output_map = new3D(getDimZ(), getDimY(), getDimX());

		switch (variable)
		{
		case 'u':
			for (int i = 0; i < queue_length; i++)
			{
				output_map[(int)poi_queue[i].z][(int)poi_queue[i].y][(int)poi_queue[i].x] = poi_queue[i].deformation.u;
			}
			break;
		case 'v':
			for (int i = 0; i < queue_length; i++)
			{
				output_map[(int)poi_queue[i].z][(int)poi_queue[i].y][(int)poi_queue[i].x] = poi_queue[i].deformation.v;
			}
			break;
		case 'w':
			for (int i = 0; i < queue_length; i++)
			{
				output_map[(int)poi_queue[i].z][(int)poi_queue[i].y][(int)poi_queue[i].x] = poi_queue[i].deformation.w;
			}
			break;
		case 'c': //ZNCC value
			for (int i = 0; i < queue_length; i++)
			{
				output_map[(int)poi_queue[i].z][(int)poi_queue[i].y][(int)poi_queue[i].x] = poi_queue[i].result.zncc;
			}
			break;
		case 'x': //strain exx
			for (int i = 0; i < queue_length; i++)
			{
				output_map[(int)poi_queue[i].z][(int)poi_queue[i].y][(int)poi_queue[i].x] = poi_queue[i].strain.exx;
			}
			break;
		case 'y': //strain eyy
			for (int i = 0; i < queue_length; i++)
			{
				output_map[(int)poi_queue[i].z][(int)poi_queue[i].y][(int)poi_queue[i].x] = poi_queue[i].strain.eyy;
			}
			break;
		case 'z': //strain ezz
			for (int i = 0; i < queue_length; i++)
			{
				output_map[(int)poi_queue[i].z][(int)poi_queue[i].y][(int)poi_queue[i].x] = poi_queue[i].strain.ezz;
			}
			break;
		case 'r': //strain exy
			for (int i = 0; i < queue_length; i++)
			{
				output_map[(int)poi_queue[i].z][(int)poi_queue[i].y][(int)poi_queue[i].x] = poi_queue[i].strain.exy;
			}
			break;
		case 's': //strain eyz
			for (int i = 0; i < queue_length; i++)
			{
				output_map[(int)poi_queue[i].z][(int)poi_queue[i].y][(int)poi_queue[i].x] = poi_queue[i].strain.eyz;
			}
			break;
		case 't': //strain ezx
			for (int i = 0; i < queue_length; i++)
			{
				output_map[(int)poi_queue[i].z][(int)poi_queue[i].y][(int)poi_queue[i].x] = poi_queue[i].strain.ezx;
			}
			break;
		default:
			return;
		}

		std::ofstream file_out(file_path);
		file_out.setf(std::ios::fixed);
		file_out << std::setprecision(8);

		if (file_out.is_open())
		{
			for (int i = 0; i < getDimZ(); i++)
			{
				for (int j = 0; j < getDimY(); j++)
				{
					for (int k = 0; k < getDimX(); k++)
					{
						file_out << output_map[i][j][k] << delimiter;
					}
					file_out << std::endl;
				}
				file_out << std::endl;
			}
		}
		file_out.close();
	}

	void IO3D::saveMatrixBin(vector<POI3D>& poi_queue)
	{
		std::ofstream file_out;
		file_out.open(file_path, std::ios::out | std::ios::binary);

		if (!file_out.is_open())
		{
			std::cerr << "failed to open file " << file_path << std::endl;
		}

		//head information, including the number of POIs and the three dimensions of image
		int queue_length = (int)poi_queue.size();
		int head_info[4];
		head_info[0] = queue_length;
		head_info[1] = dim_x;
		head_info[2] = dim_y;
		head_info[3] = dim_z;

		//create a 1D array and fill it with output data of POIs
		int result_length = 7;
		int data_length = result_length * queue_length;
		float* data_array = new float[data_length];

		for (int i = 0; i < queue_length; i++)
		{
			data_array[i * result_length] = poi_queue[i].x;
			data_array[i * result_length + 1] = poi_queue[i].y;
			data_array[i * result_length + 2] = poi_queue[i].z;
			data_array[i * result_length + 3] = poi_queue[i].deformation.u;
			data_array[i * result_length + 4] = poi_queue[i].deformation.v;
			data_array[i * result_length + 5] = poi_queue[i].deformation.w;
			data_array[i * result_length + 6] = poi_queue[i].result.zncc;
		}

		//write head information
		file_out.write((char*)head_info, sizeof(head_info[0]) * 4);

		//write data
		file_out.write((char*)data_array, sizeof(data_array[0]) * data_length);

		file_out.close();

		delete[] data_array;
	}

	vector<POI3D> IO3D::loadMatrixBin()
	{
		std::ifstream file_in(file_path);
		if (!file_in)
		{
			std::cerr << "failed to open file " << file_path << std::endl;
		}

		//read head information
		int head_info[4];
		file_in.read((char*)head_info, sizeof(head_info[0]) * 4);
		int queue_length = head_info[0];
		setDimX(head_info[1]);
		setDimY(head_info[2]);
		setDimZ(head_info[3]);

		int result_length = 7;
		int data_length = result_length * queue_length;
		float* data_array = new float[data_length];
		file_in.read((char*)data_array, sizeof(float) * data_length);
		file_in.close();

		vector<POI3D> poi_queue;
		POI3D empty_poi(0, 0, 0);
		poi_queue.resize(queue_length, empty_poi);

		for (int i = 0; i < queue_length; i++)
		{
			poi_queue[i].x = data_array[i * result_length];
			poi_queue[i].y = data_array[i * result_length + 1];
			poi_queue[i].z = data_array[i * result_length + 2];
			poi_queue[i].deformation.u = data_array[i * result_length + 3];
			poi_queue[i].deformation.v = data_array[i * result_length + 4];
			poi_queue[i].deformation.w = data_array[i * result_length + 5];
			poi_queue[i].result.zncc = data_array[i * result_length + 6];
		}

		delete[] data_array;

		return poi_queue;
	}

}//namespace opencorr