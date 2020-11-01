
#pragma once

#include <string>
#include <memory>

#include <rbdl/Model.h>

#include "../Math/Math.hpp"
#include "../Kinematic/Kinematic.hpp"

namespace IKL {
    namespace Model {
        class RBDL {
            public :
                using ScalerType = double;
                using BodyID = unsigned int;
                using ModelStructure = RigidBodyDynamics::Model;

                RBDL();
                RBDL(const std::string &urdf_filename);

                void load_urdf(const std::string &urdf_filename);

                unsigned int dof() const;

                void update_kinematics(const Kinematic::JointState<ScalerType> &joint_state);

                BodyID body_name_to_id(const std::string &body_name);
                std::string body_id_to_name(const BodyID &body_id);

                Math::Vector3<ScalerType> to_body_point(const std::string &body_name, const Kinematic::JointState<ScalerType> &);
                Math::Vector3<ScalerType> to_body_point(const BodyID &body_id, const Kinematic::JointState<ScalerType> &);

                Math::Matrix3x3<ScalerType> to_body_rotation(const std::string &body_name, const Kinematic::JointState<ScalerType> &);
                Math::Matrix3x3<ScalerType> to_body_rotation(const BodyID &body_id, const Kinematic::JointState<ScalerType> &);

                const Math::MatrixNxN<ScalerType> &twist_basic_jacobian(const std::string &body_name, const Kinematic::JointState<ScalerType> &);
                const Math::MatrixNxN<ScalerType> &twist_basic_jacobian(const BodyID &body_id, const Kinematic::JointState<ScalerType> &);

                ModelStructure *access();

                ModelStructure &operator ()();

            private :
                using ModelStructurePtr = std::unique_ptr<ModelStructure>;

                const Math::Vector3<ScalerType> zero_vec3;

                ModelStructurePtr model;

        };
    }
}

