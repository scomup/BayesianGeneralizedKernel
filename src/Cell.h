
#ifndef CELL_H_
#define CELL_H_

#include <vector>
#include <set>
#include <cstdlib>
#include <memory>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace BGK
{
constexpr double inf = std::numeric_limits<double>::infinity();

struct Cell
{
	Cell(float height)
	{
		height_ = height;
		//position = pos;
		//neighbours[0] = nullptr;//upper left 
		//neighbours[1] = nullptr;//left
		//neighbours[2] = nullptr;//bottom left
		//neighbours[3] = nullptr;//upper
		//neighbours[4] = nullptr;//bottom
		//neighbours[5] = nullptr;//upper right 
		//neighbours[6] = nullptr;//right
		//neighbours[7] = nullptr;//bottom right
		//untraversable[0] = false;
		//untraversable[1] = false;
		//dist[0] = inf;
		//dist[1] = inf;
		//cont=0;
	}
	float height_;
	//int cont;
	//Eigen::Vector3f position;
	//std::array<bool, 2> untraversable;
	//std::array<double, 2> dist;
	//std::array<Cell* , 8> neighbours;
};



}

#endif