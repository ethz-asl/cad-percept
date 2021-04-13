#ifndef CPT_RECONSTRUCTION_MESHGENERATION_H
#define CPT_RECONSTRUCTION_MESHGENERATION_H

#include "ros/ros.h"

namespace cad_percept {
    namespace cpt_reconstruction {
        class MeshGeneration {
        public:
            MeshGeneration();
        private:
            ros::NodeHandle nodeHandle_;
        };
    }  // namespace cpt_reconstruction
}  // namespace cad_percept
#endif //CPT_RECONSTRUCTION_MESHGENERATION_H
