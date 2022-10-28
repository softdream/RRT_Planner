#include <iostream>
#include "rrt_simulator.h"

int main()
{
	std::cout<<"--------------------- RRT TEST -------------------"<<std::endl;
	//rrt::RRT<float> rrt;	
	rrt::Simulator<float> rrt_sim;
	rrt_sim.initMap();
	

	rrt_sim.rrtProcess();
	cv::waitKey(0);

	return 0;
}
