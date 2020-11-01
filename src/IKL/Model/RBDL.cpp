
#include <IKL/Model/RBDL.hpp>

#include <stdexcept>

#include <rbdl/Kinematics.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

namespace IKL {
    namespace Model {
        RBDL::RBDL() : zero_vec3(Math::Vector3<ScalerType>::Zero()) {
            model = std::make_unique<ModelStructure>();

            if(model == nullptr) {
                throw std::runtime_error("Failed allocate model pointer");
            }
        }

        RBDL::RBDL(const std::string &urdf_filename) : RBDL() {
            load_urdf(urdf_filename);
        }

        void RBDL::load_urdf(const std::string &urdf_filename) {
            if(
                !RigidBodyDynamics::Addons::URDFReadFromFile(
                    urdf_filename.c_str(), model.get(), false, false
                )) {
                throw std::runtime_error("Failed load urdf model from file");
            }
        }

        unsigned int RBDL::dof() const {
            return model->dof_count;
        }

        void RBDL::update_kinematics(const Kinematic::JointState<ScalerType> &joint_state) {
            RigidBodyDynamics::UpdateKinematicsCustom(*model, &joint_state.q, nullptr, nullptr);
        }

        RBDL::BodyID RBDL::body_name_to_id(const std::string &body_name) {
            return model->GetBodyId(body_name.c_str());
        }

        std::string RBDL::body_id_to_name(const BodyID &body_id) {
            return model->GetBodyName(body_id);
        }

        Math::Vector3<RBDL::ScalerType> RBDL::to_body_point(const std::string &body_name, const Kinematic::JointState<ScalerType> &joint_state) {
            return to_body_point(body_name_to_id(body_name), joint_state);
        }

        Math::Vector3<RBDL::ScalerType> RBDL::to_body_point(const BodyID &body_id, const Kinematic::JointState<ScalerType> &joint_state) {
            return RigidBodyDynamics::CalcBodyToBaseCoordinates(
                *model,
                joint_state.q,
                body_id,
                zero_vec3,
                false
            );
        }

        Math::Matrix3x3<RBDL::ScalerType> RBDL::to_body_rotation(const std::string &body_name, const Kinematic::JointState<ScalerType> &joint_state) {
            return to_body_rotation(body_name_to_id(body_name), joint_state);
        }

        Math::Matrix3x3<RBDL::ScalerType> RBDL::to_body_rotation(const BodyID &body_id, const Kinematic::JointState<ScalerType> &joint_state) {
            return RigidBodyDynamics::CalcBodyWorldOrientation(
                *model,
                joint_state.q,
                body_id,
                false
            );
        }

        const Math::MatrixNxN<RBDL::ScalerType> &RBDL::twist_basic_jacobian(const std::string &body_name, const Kinematic::JointState<ScalerType> &joint_state) {
            return twist_basic_jacobian(body_name_to_id(body_name), joint_state);
        }

        const Math::MatrixNxN<RBDL::ScalerType> &RBDL::twist_basic_jacobian(const BodyID &body_id, const Kinematic::JointState<ScalerType> &joint_state) {
            static Math::MatrixNxN<ScalerType> J;
            J.setZero(6, joint_state.size());
            RigidBodyDynamics::CalcPointJacobian6D(*model, joint_state.q, body_id, zero_vec3, J, false);
            return J;
        }

        RBDL::ModelStructure *RBDL::access() {
            return model.get();
        }

        RBDL::ModelStructure &RBDL::operator ()() {
            return *model;
        }
    }
}

