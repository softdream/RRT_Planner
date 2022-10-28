#ifndef __OBSTACLES_H
#define __OBSTACLES_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "utils.h"

namespace rrt
{

template<typename T>
class Obstacles
{
public:
	using PoseType = typename Eigen::Matrix<T, 2, 1>;

	Obstacles()
	{

	}

	~Obstacles()
	{

	}

	void addObstacle( const PoseType& p1, const PoseType& p2 )
	{
		if( p1[0] < 0 || p1[0] > WORLD_WIDTH || p1[1] < 0 || p1[1] > WORLD_HEIGHT ){
			std::cout<<"the point 1 is out of range !"<<std::endl;
			return;
		}

		if( p2[0] < 0 || p2[0] > WORLD_WIDTH || p2[1] < 0 || p2[1] > WORLD_HEIGHT ){
                        std::cout<<"the point 2 is out of range !"<<std::endl;
                        return;
                }

		obstacles.push_back( std::make_pair( p1, p2 ) );
	}

	const bool isPointInObstacleArea( const PoseType& p ) const
	{
		for( auto& obs : obstacles ){
			if( p[0] >= obs.first[0] && p[0] <= obs.second[0] 
			 && p[1] >= obs.first[1] && p[1] <= obs.second[1] ){
				return true;
			}
		}	
		
		return false;
	}

	const std::vector<std::pair<PoseType, PoseType>>& getObstacles() const
	{
		return obstacles;
	}

private:
	std::vector<std::pair<PoseType, PoseType>> obstacles;
};

}

#endif
