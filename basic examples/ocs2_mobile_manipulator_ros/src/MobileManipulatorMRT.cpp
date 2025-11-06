/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <interbotix_xs_msgs/msg/joint_group_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <thread>
#include <chrono>

using namespace ocs2;
using namespace mobile_manipulator;

// Global variables for robot state and control
std::mutex stateMutex_;
SystemObservation currentRobotObservation_;
bool robotStateReceived_ = false;
std::vector<std::string> jointNames_;
rclcpp::Node::SharedPtr node_ = nullptr;
rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr commandPublisher_ = nullptr;
MRT_ROS_Interface* mrtPtr_ = nullptr;

/**
 * Converts robot joint state message to OCS2 SystemObservation
 */
SystemObservation convertToObservation(
    const control_msgs::msg::JointTrajectoryControllerState::SharedPtr& msg) {
    SystemObservation observation;
    
    // Convert ROS time to OCS2 time
    // Adjust time offset if needed (e.g., if you want to start from 0)
    static double timeOffset = 0.0;
    static bool firstTime = true;
    if (firstTime) {
        timeOffset = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        firstTime = false;
    }
    observation.time = (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9) - timeOffset;
    
    // Extract joint positions as state
    // For controller_state topic, use feedback.positions (for state topic, use actual.positions)
    size_t numJoints = 0;
    if (!msg->feedback.positions.empty()) {
        // Use feedback (controller_state topic)
        numJoints = msg->feedback.positions.size();
    } else if (!msg->actual.positions.empty()) {
        // Fallback to actual (state topic)
        numJoints = msg->actual.positions.size();
    } else {
        RCLCPP_WARN(node_->get_logger(), "No joint positions found in state message");
        return observation;
    }
    
    observation.state.resize(numJoints);
    observation.input.resize(numJoints);
    
    for (size_t i = 0; i < numJoints; i++) {
        // Prefer feedback over actual for controller_state compatibility
        if (!msg->feedback.positions.empty() && i < msg->feedback.positions.size()) {
            observation.state[i] = msg->feedback.positions[i];
            if (i < msg->feedback.velocities.size()) {
                observation.input[i] = msg->feedback.velocities[i];
            } else {
                observation.input[i] = 0.0;
            }
        } else if (!msg->actual.positions.empty() && i < msg->actual.positions.size()) {
            observation.state[i] = msg->actual.positions[i];
            if (i < msg->actual.velocities.size()) {
                observation.input[i] = msg->actual.velocities[i];
            } else {
                observation.input[i] = 0.0;
            }
        } else {
            observation.state[i] = 0.0;
            observation.input[i] = 0.0;
        }
    }
    
    // Set mode (adjust based on your system)
    observation.mode = 0;
    
    return observation;
}

/**
 * Converts MPC state (positions) to robot command message
 * @param mpcState: The MPC optimized state trajectory position (already computed by MPC)
 */
interbotix_xs_msgs::msg::JointGroupCommand createCommandMessage(const vector_t& mpcState) {
    interbotix_xs_msgs::msg::JointGroupCommand command;
    
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (jointNames_.empty()) {
        RCLCPP_WARN(node_->get_logger(), "No joint names available yet");
        return command;
    }
    
    // Set joint group name for Interbotix robots
    command.name = "arm";
    
    size_t numJoints = jointNames_.size();
    command.cmd.resize(numJoints);
    
    // Use MPC state trajectory position directly (no integration needed)
    // The MPC already computed the optimal position trajectory
    // Note: cmd is float32[], so we cast to float to match the message type
    for (size_t i = 0; i < numJoints && i < mpcState.size(); i++) {
        command.cmd[i] = static_cast<float>(mpcState[i]);
    }
    
    // Fill remaining joints with zero if MPC state is shorter
    for (size_t i = mpcState.size(); i < numJoints; i++) {
        command.cmd[i] = 0.0f;
    }
    
    return command;
}

// interbotix_xs_msgs::msg::JointGroupCommand createCommandMessage(const vector_t& mpcInput) {
//     interbotix_xs_msgs::msg::JointGroupCommand command;

//     std::lock_guard<std::mutex> lock(stateMutex_);
//     if (jointNames_.empty()) {
//         RCLCPP_WARN(node_->get_logger(), "No joint names available yet");
//         return command;
//     }

//     command.name = "arm";
//     size_t numJoints = jointNames_.size();
//     command.cmd.resize(numJoints);

//     // Use the first numJoints elements of mpcInput as the command for each joint
//     for (size_t i = 0; i < numJoints; ++i) {
//         if (i < mpcInput.size()) {
//             command.cmd[i] = static_cast<float>(mpcInput[i]);
//         } else {
//             command.cmd[i] = 0.0f; // or keep as default
//         }
//     }

//     return command;
// }

/**
 * Callback for robot state messages
 */
void robotStateCallback(
    const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg) {
    // Store joint names on first message
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (jointNames_.empty() && !msg->joint_names.empty()) {
            jointNames_ = msg->joint_names;
            RCLCPP_INFO_STREAM(node_->get_logger(),
                              "Detected " << jointNames_.size() << " joints");
        }
    }
    
    // Convert to OCS2 observation
    SystemObservation observation = convertToObservation(msg);
    
    // Update current observation
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        currentRobotObservation_ = observation;
        robotStateReceived_ = true;
    }
    
    // Send to MPC (if MRT is initialized)
    if (mrtPtr_ != nullptr) {
        mrtPtr_->setCurrentObservation(observation);
    }
}

/**
 * Main control loop function
 */
void controlLoop(MRT_ROS_Interface& mrt, MobileManipulatorInterface& interface) {
    // Wait for initial robot state
    RCLCPP_INFO(node_->get_logger(), "Waiting for initial robot state...");
    while (!robotStateReceived_ && rclcpp::ok()) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Get initial state
    SystemObservation initObservation;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        initObservation = currentRobotObservation_;
    }
    
    // Create initial target trajectory (current state as target)
    vector_t initTarget;
    if (interface.dual_arm_) {
        initTarget.resize(14);
        // First arm target: current position
        initTarget.head(3) = initObservation.state.head(3);
        initTarget.segment(3, 4) << Eigen::Quaternion<scalar_t>(1, 0, 0, 0).coeffs();
        // Second arm target
        initTarget.segment(7, 3) = initObservation.state.segment(3, 3);
        initTarget.tail(4) << Eigen::Quaternion<scalar_t>(1, 0, 0, 0).coeffs();
    } else {
        initTarget.resize(7);
        initTarget.head(3) << 0.5, 0, 0.5;
        initTarget.tail(4) << Eigen::Quaternion<scalar_t>(1, 0, 0, 0).coeffs();
    }
    const vector_t zeroInput =
        vector_t::Zero(interface.getManipulatorModelInfo().inputDim);
    const TargetTrajectories initTargetTrajectories({initObservation.time},
                                                    {initTarget}, {zeroInput});
    
    // Reset MPC node
    RCLCPP_INFO(node_->get_logger(), "Resetting MPC node...");
    mrt.resetMpcNode(initTargetTrajectories);
    
    // Wait for initial policy
    RCLCPP_INFO(node_->get_logger(), "Waiting for initial MPC policy...");
    while (!mrt.initialPolicyReceived() && rclcpp::ok()) {
        mrt.spinMRT();
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    RCLCPP_INFO(node_->get_logger(), "Initial MPC policy received!");
    
    // Main control loop
    const scalar_t mrtFrequency = interface.mpcSettings().mrtDesiredFrequency_;
    const auto dt = 1.0 / mrtFrequency;
    rclcpp::Rate controlRate(mrtFrequency);
    
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "Starting control loop at " << mrtFrequency << " Hz");
    
    while (rclcpp::ok()) {
        // Process ROS callbacks
        mrt.spinMRT();
        rclcpp::spin_some(node_);
        
        // Get current robot state
        SystemObservation currentObservation;
        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            if (!robotStateReceived_) {
                controlRate.sleep();
                continue;
            }
            currentObservation = currentRobotObservation_;
        }
        
        // Update policy if new one is available
        bool policyUpdated = mrt.updatePolicy();
        if (policyUpdated) {
            RCLCPP_INFO_STREAM(node_->get_logger(),
                              "<<< New MPC policy received at time "
                                  << currentObservation.time);
        }
        
        // Evaluate the MPC policy to get control input
        vector_t mpcState, mpcInput;
        size_t mode;
        try {
            mrt.evaluatePolicy(currentObservation.time, currentObservation.state,
                              mpcState, mpcInput, mode);
            
            // Use MPC state trajectory position directly
            // mpcState is the interpolated position from the MPC's optimized state trajectory
            // No integration needed - the MPC already computed the optimal positions
            auto commandMsg = createCommandMessage(mpcState);
            
            // Publish command to robot
            if (!commandMsg.cmd.empty()) {
                commandPublisher_->publish(commandMsg);
            }
            
            // Logging for monitoring (throttled to avoid spam)
            vector_t stateError = currentObservation.state - mpcState;
            double maxError = stateError.cwiseAbs().maxCoeff();
            RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                  "Time: %.3f, Max tracking error: %.4f rad",
                                  currentObservation.time, maxError);
        } catch (const std::exception& e) {
            RCLCPP_WARN_STREAM(node_->get_logger(),
                              "Error evaluating policy: " << e.what());
        }
        
        // Send current observation to MPC
        mrt.setCurrentObservation(currentObservation);
        
        controlRate.sleep();
    }
}

int main(int argc, char** argv) {
    const std::string robotName = "mobile_manipulator";
    
    // Initialize ROS node
    rclcpp::init(argc, argv);
    node_ = rclcpp::Node::make_shared(
        robotName + "_mrt",
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true));
    
    // Get node parameters
    std::string taskFile = node_->get_parameter("taskFile").as_string();
    std::string libFolder = node_->get_parameter("libFolder").as_string();
    std::string urdfFile = node_->get_parameter("urdfFile").as_string();
    
    // Get robot state topic (optional parameter, with default)
    // Use absolute path with leading slash to match actual topic namespacing
    std::string robotStateTopic = "/vx300s/arm_controller/controller_state";
    if (node_->has_parameter("robotStateTopic")) {
        robotStateTopic = node_->get_parameter("robotStateTopic").as_string();
        // Ensure topic has leading slash for absolute path
        if (!robotStateTopic.empty() && robotStateTopic[0] != '/') {
            robotStateTopic = "/" + robotStateTopic;
        }
    }
    
    // Get robot command topic (optional parameter, with default)
    // Use absolute path with leading slash to match actual topic namespacing
    std::string robotCommandTopic = "/vx300s/commands/joint_group";
    if (node_->has_parameter("robotCommandTopic")) {
        robotCommandTopic = node_->get_parameter("robotCommandTopic").as_string();
        // Ensure topic has leading slash for absolute path
        if (!robotCommandTopic.empty() && robotCommandTopic[0] != '/') {
            robotCommandTopic = "/" + robotCommandTopic;
        }
    }
    
    RCLCPP_INFO_STREAM(node_->get_logger(), "Loading task file: " << taskFile);
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "Loading library folder: " << libFolder);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Loading urdf file: " << urdfFile);
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "Robot state topic: " << robotStateTopic);
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "Robot command topic: " << robotCommandTopic);
    
    // Robot Interface
    MobileManipulatorInterface interface(taskFile, libFolder, urdfFile);
    
    // MRT Interface
    MRT_ROS_Interface mrt(robotName);
    mrt.initRollout(&interface.getRollout());
    mrt.launchNodes(node_);
    mrtPtr_ = &mrt; // Set global pointer for state callback
    
    // Subscribe to robot state
    auto stateSubscription = node_->create_subscription<
        control_msgs::msg::JointTrajectoryControllerState>(
        robotStateTopic, 10, robotStateCallback);
    
    // Publisher for robot commands
    commandPublisher_ = node_->create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>(
        robotCommandTopic, 10);
    
    // Run control loop (handles all ROS spinning internally)
    controlLoop(mrt, interface);
    
    rclcpp::shutdown();
    return 0;
}

