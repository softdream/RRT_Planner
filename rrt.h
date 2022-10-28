#ifndef __RRT_H
#define __RRT_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>

//#include <stdlib.h>
#include <ctime>
#include <limits>

//#include "obstacles.h"
#include "utils.h"

namespace rrt{


template<typename T>
struct Node
{
	using value_type = T;
	using DataType = typename Eigen::Matrix<value_type, 2, 1>;

	Node()
	{

	}

	Node( const DataType& pose_ ) : pose( pose_ )
	{

	}

	~Node()
	{

	}

	std::vector<struct Node*> children;
	struct Node* parent = nullptr;

	DataType pose = DataType::Zero();

};


template<typename T>
class RRT
{
public:
	using NodeType = Node<T>;
	using NodePointer = Node<T>*;
	using PoseType = typename Eigen::Matrix<T, 2, 1>;

	RRT()
	{
		root = new NodeType( start_pose  );
                nodes.push_back( root );
                last_node = root;

	}

	RRT( const PoseType& start_pose_ ) : start_pose( start_pose_ )
	{
		root = new NodeType( start_pose_ );
		nodes.push_back( root );
		last_node = root;
	}

	~RRT()
	{

	}

	const NodePointer getRandomNode() const
	{
		NodePointer ret = nullptr;
	
		srand(time(NULL));
		T a = rand() % (1000) / (T)(1000);
	
                T b = rand() % (1000) / (T)(1000);

		PoseType point(  a * WORLD_WIDTH, b * WORLD_HEIGHT );

		if( point[0] >= 0 && point[0] <= WORLD_WIDTH && point[1] >= 0 && point[1] <= WORLD_HEIGHT ){
			ret = new NodeType( point );
				
			return ret;
		}

		return nullptr;
	}

	const NodePointer getNearestNode( const PoseType& point ) const
	{
		T min_dist = std::numeric_limits<T>::max();
		NodePointer closest = nullptr;
	
		for( auto& it : nodes ){
			T dist = distance( point, it->pose );
	
			if( dist < min_dist ){
				min_dist = dist;
				closest = it;
			}
		}

		return closest;
	}

	const PoseType getNewPoseOfNode( const NodePointer& random_node, const NodePointer& nearest_node )
	{
		PoseType to = random_node->pose;
		PoseType from = nearest_node->pose;

		PoseType inter_mediate = to - from;
		inter_mediate = inter_mediate / inter_mediate.norm();
		
		PoseType ret = from + step_size * inter_mediate;

		return ret;
	}

	void add( const NodePointer& nearest_node, const NodePointer& new_node )
	{
		new_node->parent = nearest_node;
		nearest_node->children.push_back( new_node );
		nodes.push_back( new_node );
		
		last_node = new_node;
	}

	void deleteNodes( const NodePointer& root )
	{
		for( auto& it : root->children ){
			deleteNodes( it );
		}

		delete root;
	}
	
	const bool reached( const PoseType& end_pose ) const
	{	
		if( distance( last_node->pose, end_pose ) < END_DIST_THRESHOLD ){
			return true;
		}
		return false;
	}
	
	void setStartPose( const PoseType& start_pose_ )
	{
		start_pose = start_pose_;
	}

	const PoseType& getStarPose() const
	{
		return start_pose;
	}

	const NodePointer getLastNode() const
	{
		return last_node;
	}

	void addToPath( const NodePointer& node )
	{
		return path.push_back( node );
	}

	const std::vector<NodePointer>& getPath() const
	{
		return path;
	}
	
private:
	const T distance( const PoseType& v1, const PoseType& v2  ) const
	{
		return ( v1 - v2 ).norm();
	}

private:
	//std::default_random_engine e;
	//std::uniform_real_distribution<T> value = std::uniform_real_distribution<T>(0, 1);
	std::vector<NodePointer> path;

	std::vector<NodePointer> nodes;
	NodePointer root = nullptr;
	NodePointer last_node = nullptr;

	PoseType start_pose = PoseType::Zero();

	T step_size = 40; 
};

}

#endif
