#ifndef CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_SUBSCRIBER_H
#define CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_SUBSCRIBER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cpt_reconstruction/coordinates.h"

namespace cad_percept{
namespace cpt_reconstruction{
	class ReconstructionPointsSubscriber {
	public:
		ReconstructionPointsSubscriber(ros::NodeHandle nodeHandle);
		void startReceiving();

	private:
		void messageCallback(const ::cpt_reconstruction::coordinates& msg);
		ros::NodeHandle nodeHandle_;
		ros::Subscriber subscriber_;
	};
}
}

#endif //CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_SUBSCRIBER_H