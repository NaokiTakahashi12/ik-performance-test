
#include <memory>
#include <exception>
#include <iostream>

#include <IKL/IKL.hpp>

#include <IKBenchmark/IKBenchmark.hpp>

auto main() -> int {
    using namespace IKL;
    using namespace IKL::Math;

    try {
        constexpr bool enable_benchmark = true;
        constexpr bool enable_darwin_op_benchmark = true;
        constexpr bool enable_atlas_v5_benchmark = true;

        using Scaler = Model::RBDL::ScalerType;
        Parameters<Scaler> ik_param;
        //! @todo From csv file
        ik_param.max_itr_reached(1024);
        ik_param.min_step_tol(1e-13);
        ik_param.min_pose_tol(1e-8);
        ik_param.weight_e_scaler(1 - 1e-4);
        ik_param.weight_n_scaler(2e-5);
        ik_param.manipulability_threshold(1e-1);
        ik_param.minimum_bias(1e-3);

        if constexpr (enable_darwin_op_benchmark && enable_benchmark) {
            // Squat benchmakrs
            Model::RBDL darwin_op_model("../data/model/urdf/robotis_op2.urdf");

            Kinematic::JointState<Scaler> darwin_op_joint_state{darwin_op_model.dof()};
            std::vector<std::unique_ptr<NumericIK<Scaler>>> darwin_op_iks;

            // Initial joint state
            //! @todo From csv file
            darwin_op_joint_state.q << -2.22045e-16, -2.22045e-16, -2.2204e-16, -2.2204e-16, -2.22045e-16, -2.2204e-16, -2.2204e-16, -2.2204e-16, -2.22045e-16, -2.22045e-16, -2.22045e-16, -0.000100624, -0.00628986, -1.47892, 2.97041, -1.49159, 0.00618986, -2.22045e-16, -2.22045e-16, -2.22045e-16;
            VectorN<double> limit_q_min = -M_PI * VectorN<double>::Ones(darwin_op_joint_state.size());
            VectorN<double> limit_q_max = M_PI * VectorN<double>::Ones(darwin_op_joint_state.size());
            limit_q_min(14) = -0.0349;
            limit_q_min(5) = -0.0349;
            ik_param.limit_q_min(limit_q_min);
            ik_param.limit_q_max(limit_q_max);

            // Push back inverse kinematics solvers
            darwin_op_iks.push_back(std::make_unique<Solver::JacobianTranspose<Scaler>>(ik_param));
            darwin_op_iks.push_back(std::make_unique<Solver::LeastSquares<Scaler>>(ik_param));
            darwin_op_iks.push_back(std::make_unique<Solver::LeastSquaresQR<Scaler>>(ik_param));
            darwin_op_iks.push_back(std::make_unique<Solver::GaussNewton<Scaler>>(ik_param));
            darwin_op_iks.push_back(std::make_unique<Solver::GaussNewtonQR<Scaler>>(ik_param));
            darwin_op_iks.push_back(std::make_unique<Solver::GaussNewtonSVD<Scaler>>(ik_param));
            darwin_op_iks.push_back(std::make_unique<Solver::LevenbergMarquardtSRC<Scaler>>(ik_param));
            darwin_op_iks.push_back(std::make_unique<Solver::LevenbergMarquardtSRM<Scaler>>(ik_param));
            darwin_op_iks.push_back(std::make_unique<Solver::LevenbergMarquardtED<Scaler>>(ik_param));
            darwin_op_iks.push_back(std::make_unique<Solver::LevenbergMarquardtSu<Scaler>>(ik_param));
            darwin_op_iks.push_back(std::make_unique<Solver::QProblemGNByqpOASES<Scaler>>(ik_param));
            darwin_op_iks.push_back(std::make_unique<Solver::QProblemLMByqpOASES<Scaler>>(ik_param));
            darwin_op_iks.push_back(std::make_unique<Solver::QProblemGNByOsqp<Scaler>>(ik_param));
            darwin_op_iks.push_back(std::make_unique<Solver::QProblemLMByOsqp<Scaler>>(ik_param));

            IKBenchmark::Squat6D::Squat6DBenchmark<Scaler> squat_benchmark;

            squat_benchmark.output_directory("../data/generated/darwin_op");
            squat_benchmark.init_joint_state(darwin_op_joint_state);
            squat_benchmark.move_foot_name("r_ank_roll_link");
            squat_benchmark.fixed_body_name("l_ank_roll_link");
            squat_benchmark.fixed_body_name("r_el_link");
            squat_benchmark.fixed_body_name("l_el_link");
            squat_benchmark.fixed_body_name("head_tilt_link");
            squat_benchmark.move_ik_solvers(darwin_op_iks);
            squat_benchmark(darwin_op_model);
        }
        if constexpr (enable_atlas_v5_benchmark && enable_benchmark) {
            // Squat benchmakrs
            Model::RBDL atlas_v5_model("../data/model/urdf/atlas_v5_raw.urdf");

            Kinematic::JointState<Scaler> atlas_v5_joint_state{atlas_v5_model.dof()};
            std::vector<std::unique_ptr<NumericIK<Scaler>>> atlas_v5_iks;

            // Initial joint state
            //! @todo From csv file
            atlas_v5_joint_state.q << 2.22045e-16, 2.22045e-16, 2.22045e-16, 2.22045e-16, 2.22045e-16, 2.22045e-16, 2.22045e-16, 2.22045e-16, 2.22045e-16, 2.22045e-16, 2.22045e-16, 2.22045e-16, 2.22045e-16, 2.22045e-16, 2.22045e-16, 2.22045e-16, 2.22045e-16, 2.22045e-16, -2.25954e-06,  3.89461e-05, -2.25954e-16, -2.25954e-16, -2.25954e-16, -3.89585e-05, 2.25954e-06, -3.89461e-05, -1.56534, 2.78826, -1.22292, 3.89585e-05;
            VectorN<double> limit_q_min = -M_PI * VectorN<double>::Ones(atlas_v5_joint_state.size());
            VectorN<double> limit_q_max = M_PI * VectorN<double>::Ones(atlas_v5_joint_state.size());
            limit_q_min(27) = -0.0872;
            limit_q_min(21) = -0.0872;
            ik_param.limit_q_min(limit_q_min);
            ik_param.limit_q_max(limit_q_max);

            // Push back inverse kinematics solvers
            atlas_v5_iks.push_back(std::make_unique<Solver::JacobianTranspose<Scaler>>(ik_param));
            atlas_v5_iks.push_back(std::make_unique<Solver::LeastSquares<Scaler>>(ik_param));
            atlas_v5_iks.push_back(std::make_unique<Solver::LeastSquaresQR<Scaler>>(ik_param));
            atlas_v5_iks.push_back(std::make_unique<Solver::GaussNewton<Scaler>>(ik_param));
            atlas_v5_iks.push_back(std::make_unique<Solver::GaussNewtonQR<Scaler>>(ik_param));
            atlas_v5_iks.push_back(std::make_unique<Solver::GaussNewtonSVD<Scaler>>(ik_param));
            atlas_v5_iks.push_back(std::make_unique<Solver::LevenbergMarquardtSRC<Scaler>>(ik_param));
            atlas_v5_iks.push_back(std::make_unique<Solver::LevenbergMarquardtSRM<Scaler>>(ik_param));
            atlas_v5_iks.push_back(std::make_unique<Solver::LevenbergMarquardtED<Scaler>>(ik_param));
            atlas_v5_iks.push_back(std::make_unique<Solver::LevenbergMarquardtSu<Scaler>>(ik_param));
            atlas_v5_iks.push_back(std::make_unique<Solver::QProblemGNByqpOASES<Scaler>>(ik_param));
            atlas_v5_iks.push_back(std::make_unique<Solver::QProblemLMByqpOASES<Scaler>>(ik_param));
            atlas_v5_iks.push_back(std::make_unique<Solver::QProblemGNByOsqp<Scaler>>(ik_param));
            atlas_v5_iks.push_back(std::make_unique<Solver::QProblemLMByOsqp<Scaler>>(ik_param));

            IKBenchmark::Squat6D::Squat6DBenchmark<Scaler> squat_benchmark;

            squat_benchmark.output_directory("../data/generated/atlas_v5");
            squat_benchmark.init_joint_state(atlas_v5_joint_state);
            squat_benchmark.move_foot_name("r_foot");
            squat_benchmark.fixed_body_name("l_foot");
            squat_benchmark.fixed_body_name("r_hand");
            squat_benchmark.fixed_body_name("l_hand");
            squat_benchmark.fixed_body_name("head");
            squat_benchmark.move_ik_solvers(atlas_v5_iks);
            squat_benchmark(atlas_v5_model);
        }

    }
    catch(const std::exception &error) {
        std::cerr << error.what() << std::endl;
    }
    return 0;
}

