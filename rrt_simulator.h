#ifndef __RRT_SIMULATOR_H
#define __RRT_SIMULATOR_H

#include "rrt.h"
#include "obstacles.h"
#include <opencv2/opencv.hpp>


namespace rrt
{

template<typename T>
class Simulator
{
public:
	using PoseType = typename Eigen::Matrix<T, 2, 1>;
	using ObstacleType = typename std::pair<PoseType, PoseType>;
	using NodePointer = typename RRT<T>::NodePointer;
	using NodeType = typename RRT<T>::NodeType;

	Simulator()
	{
		rrt = new RRT<T>( PoseType( 0, 0 ) );
		// start pose
		cv::circle( map, cv::Point( 0, 0 ), 8, cv::Scalar( 0, 0, 255 ), -1 );
		// end pose
		cv::circle( map, cv::Point( end_pose[0], end_pose[1] ), 8, cv::Scalar( 0, 0, 255 ), -1 );
	}

	~Simulator()
	{
		delete rrt;
	}

	void initMap()
	{
		// init obstacles
		obstacles.addObstacle( PoseType( 200, 0 ), PoseType( 250, 400 ) );
		obstacles.addObstacle( PoseType( 550, 400 ), PoseType( 600, 800 ) );		

		std::vector<ObstacleType> obstacles_vec = obstacles.getObstacles();
		drawObstacles( obstacles_vec );
		cv::imshow( "map", map );
		cv::waitKey(0);
	}

	const bool runRrtOnce()
	{
		// 1. get the random point in the map
		NodePointer random_node = rrt->getRandomNode();
		std::cout<<"random point x = "<<random_node->pose[0]<<", y = "<<random_node->pose[1]<<std::endl;
		cv::circle( map, cv::Point( random_node->pose[0], random_node->pose[1] ), 8, cv::Scalar( 0, 255, 0 ), -1 );

		// 2. get the nearest point of the random point
		NodePointer nearest_node = rrt->getNearestNode( random_node->pose );
		std::cout<<"nearest point x = "<<nearest_node->pose[0]<<", y = "<<nearest_node->pose[1]<<std::endl;

		// 3. get the new pose of the next point
		PoseType new_pose = rrt->getNewPoseOfNode( random_node, nearest_node );

		// 4. judge 
		if( obstacles.isPointInObstacleArea( new_pose ) ){
			std::cout<<"in obstacle area !"<<std::endl;
			return false;
		}

		// 5. 
		NodePointer new_node = new NodeType( new_pose );
		rrt->add( nearest_node, new_node );

		cv::circle( map, cv::Point( new_pose[0], new_pose[1] ), 8, cv::Scalar( 0, 0, 255 ), -1 );

		cv::line( map, cv::Point( new_pose[0], new_pose[1] ), cv::Point( nearest_node->pose[0], nearest_node->pose[1] ), cv::Scalar( 255, 0, 0 ), 2 );

		if( rrt->reached( end_pose ) ){
			std::cout<<"reached !"<<std::endl;
			cv::line( map, cv::Point( new_pose[0], new_pose[1] ), cv::Point( end_pose[0], end_pose[1] ), cv::Scalar( 255, 0, 0 ), 2 );
			cv::imshow( "map", map );
			cv::waitKey(0);
			return true;
		}	

		cv::imshow( "map", map );
		return false;
	}

	void rrtProcess( const int max_iteration = 3000 )
	{	
		for( int i = 0; i < max_iteration; i ++ ){
			if( runRrtOnce() ){
				NodePointer q = rrt->getLastNode();
				while (q != NULL) {
					rrt->addToPath( q );
        				q = q->parent;
    				}

				break;
                	}
                	cv::waitKey(100);
		}

		std::vector<NodePointer> path = rrt->getPath();
		for( int i = 0; i < path.size() - 1; i ++ ){
			cv::line( map, cv::Point( path[i]->pose[0], path[i]->pose[1] ), cv::Point( path[i + 1]->pose[0], path[i + 1]->pose[1] ), cv::Scalar( 0, 0, 255 ), 3 );
		}
		cv::imshow( "map", map );

		cv::waitKey(0);
	}

private:	
	void drawObstacles( const std::vector<ObstacleType>& obstacles_vec )
	{
		for( auto& obs : obstacles_vec ){
			cv::rectangle( map, cv::Point( obs.first[0], obs.first[1] ), cv::Point( obs.second[0], obs.second[1] ), cv::Scalar( 0, 0, 0 ), -1 );
		}
	}	

private:
	cv::Mat map = cv::Mat( WORLD_WIDTH, WORLD_HEIGHT, CV_8UC3, cv::Scalar(255, 255, 255 ) );

	Obstacles<T> obstacles;
	RRT<T>* rrt;

	PoseType end_pose = PoseType( 800, 800 );

};


}


#endif
