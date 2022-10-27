#ifdef __RRT_H
#define __RRT_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>

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

	std::vector<struct Node*> children = nullptr;
	struct Node* parent = nullptr;

	Datatype pose = DataType::Zero();

};


}

#endif
