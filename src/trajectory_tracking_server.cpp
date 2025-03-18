/**
 * @file   velocity_control_server.cpp
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  This is testing & demonstration.
 */

#include <ModelUpdater.h>                                                                           // Joint state subscriber
#include <RobotLibrary/Control/SerialKinematicControl.h>                                            // For serial link robots
#include <TrackCartesianTrajectory.h>
#include <TrackJointTrajectory.h>
#include <Utilities.h>


Eigen::Matrix<double,6,6>
vector_to_matrix(const std::vector<double> vector)
{
    Eigen::Matrix<double,6,6> matrix;  // Value to be returned
    
    if(vector.size() != 36)
    {
        throw std::invalid_argument("Expected a vector with 36 elements, but it had " + std::to_string(vector.size()) + ".");
    }

    // Fill matrix in row-major order
    for (std::size_t i = 0; i < 6; ++i)
    {
        for (std::size_t j = 0; j < 6; ++j)
        {
            matrix(i, j) = vector[i * 6 + j];
        }
    }
    
    return matrix;
}
RobotLibrary::Control::Parameters
load_control_parameters(const std::shared_ptr<rclcpp::Node> &node)
{
    RobotLibrary::Control::Parameters parameters;

    // Declare parameters for controller
    node->declare_parameter("cartesian_damping", std::vector<double>());
    node->declare_parameter("cartesian_stiffness", std::vector<double>());
    node->declare_parameter("frequency", parameters.controlFrequency);
    node->declare_parameter("joint_position_gain", parameters.jointPositionGain);
    node->declare_parameter("joint_velocity_gain", parameters.jointVelocityGain);
    node->declare_parameter("manipulability_threshold", parameters.minManipulability);
    node->declare_parameter("max_joint_acceleration", parameters.maxJointAcceleration);
    
    // Declare parameters for QP solver
    node->declare_parameter("barrier_reduction_rate", parameters.qpsolver.barrierReductionRate);
    node->declare_parameter("initial_barrier_scalar", parameters.qpsolver.initialBarrierScalar);
    node->declare_parameter("max_steps", static_cast<int>(parameters.qpsolver.maxSteps));           // Need to cast from unsigned int to int
    node->declare_parameter("step_size_tolerance", parameters.qpsolver.stepSizeTolerance);
    
    // Get control parameters
    std::vector<double> damping_vec;
    node->get_parameter("cartesian_damping", damping_vec);
    parameters.cartesianDamping = vector_to_matrix(damping_vec);

    std::vector<double> stiffness_vec;
    node->get_parameter("cartesian_stiffness", stiffness_vec);
    parameters.cartesianStiffness = vector_to_matrix(stiffness_vec);

    node->get_parameter("frequency", parameters.controlFrequency);
    node->get_parameter("joint_position_gain", parameters.jointPositionGain);
    node->get_parameter("joint_velocity_gain", parameters.jointVelocityGain);
    node->get_parameter("manipulability_threshold", parameters.minManipulability);
    node->get_parameter("max_joint_acceleration", parameters.maxJointAcceleration);

    // Get QP solver parameters
    node->get_parameter("barrier_reduction_rate", parameters.qpsolver.barrierReductionRate);
    node->get_parameter("initial_barrier_scalar", parameters.qpsolver.initialBarrierScalar);
    node->get_parameter("step_size_tolerance", parameters.qpsolver.stepSizeTolerance);
    node->get_parameter("max_steps", parameters.qpsolver.maxSteps);
    
    return parameters;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            MAIN                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                                                                       // Launches ROS2

    unsigned int returnCode = 0;

    if (argc < 5)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"),
                     "Invalid number of arguments. Usage: <executable> <urdf_path> <endpoint_name> <control_topic_name> <joint_state_topic_name>");
        returnCode = 1;
    }
    else
    {  
        // For clarity:
        std::string urdfPath        = argv[1];
        std::string endpointName    = argv[2];
        std::string controlTopic    = argv[3];
        std::string jointStateTopic = argv[4];
            
        try 
        {
            auto model = std::make_shared<RobotLibrary::Model::KinematicTree>(urdfPath);            // Generate dynamic model
            
            auto modelUpdaterNode = std::make_shared<ModelUpdater>(model, jointStateTopic);         // Create node for updating joint state
            
            auto serverNode = std::make_shared<rclcpp::Node>(model->name()+"_action_server");       // Create action server nodes
            
            auto controller = std::make_shared<RobotLibrary::Control::SerialKinematicControl>(model, endpointName, load_control_parameters(serverNode));
              
            // Declare action servers
            auto mutex = std::shared_ptr<std::mutex>(); 
       
            TrackJointTrajectory jointTrajectoryServer(
                serverNode,
                controller,
                mutex,
                "track_joint_trajectory",
                "joint_command_relay");
                                                   
            TrackCartesianTrajectory cartesianTrajectoryServer(
                serverNode,
                controller,
                mutex,
                "track_cartesian_trajectory",
                "joint_command_relay");
                     
            // Add nodes to executor and spin
            rclcpp::executors::MultiThreadedExecutor executor;
            executor.add_node(modelUpdaterNode);
            executor.add_node(serverNode);
            executor.spin();

        }
        catch(const std::exception &exception)
        {
            RCLCPP_ERROR(rclcpp::get_logger("main"), exception.what());
            
            returnCode = 1;
        }  
    }
    
    rclcpp::shutdown();

    return returnCode;
}
