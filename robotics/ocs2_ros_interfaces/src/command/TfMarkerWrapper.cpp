#include "ocs2_ros_interfaces/command/TfMarkerWrapper.h"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace ocs2
{
    TfMarkerWrapper::TfMarkerWrapper(
        rclcpp::Node::SharedPtr node,
        IMarkerControl* markerControl,
        const std::string& targetFrame,
        const std::string& sourceFrame,
        const double updateRate)
        : node_(std::move(node)),
          tfBuffer_(node_->get_clock()),
          tfListener_(tfBuffer_),
          markerControl_(markerControl),
          targetFrame_(targetFrame),
          sourceFrame_(sourceFrame),
          updateRate_(updateRate),
          enabled_(false),
          lastUpdateTime_(node_->now())
    {
        // Create timer for periodic TF checking
        auto timerCallback = [this]() {
            this->timerCallback();
        };
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / updateRate_)),
            timerCallback);

        // Initialize transform caching
        hasPrevTf_ = false;
        prevPosition_ = Eigen::Vector3d::Zero();
        prevOrientation_ = Eigen::Quaterniond::Identity();

        // Get threshold parameters from ROS parameters (with defaults)
        if (!node_->has_parameter("tf_marker.position_threshold")) {
            node_->declare_parameter<double>("tf_marker.position_threshold", 0.03);  // 5cm default
        }
        positionThreshold_ = node_->get_parameter("tf_marker.position_threshold").as_double();

        if (!node_->has_parameter("tf_marker.orientation_threshold")) {
            node_->declare_parameter<double>("tf_marker.orientation_threshold", 0.5236);  // 30 degree in radians default
        }
        orientationThreshold_ = node_->get_parameter("tf_marker.orientation_threshold").as_double();
        
        RCLCPP_INFO(node_->get_logger(), "🔗 TfMarkerWrapper created");
        RCLCPP_INFO(node_->get_logger(), "🔗 Position threshold: %.4f m, Orientation threshold: %.4f rad (%.2f deg)",
                    positionThreshold_, orientationThreshold_, orientationThreshold_ * 180.0 / M_PI);
        RCLCPP_INFO(node_->get_logger(), "🔗 Listening to TF: %s -> %s", sourceFrame_.c_str(), targetFrame_.c_str());
        RCLCPP_INFO(node_->get_logger(), "🔗 TF control is DISABLED by default. Call enable() to activate.");
    }

    void TfMarkerWrapper::enable()
    {
        enabled_.store(true);
        
        // Enable continuous mode if not already enabled
        if (!markerControl_->isContinuousMode())
        {
            markerControl_->togglePublishMode();
            RCLCPP_INFO(node_->get_logger(), "🔗 Continuous mode enabled for TF control");
        }
        
        RCLCPP_INFO(node_->get_logger(), "🔗 TF control ENABLED! Updating marker from %s -> %s", 
                    sourceFrame_.c_str(), targetFrame_.c_str());
    }

    void TfMarkerWrapper::disable()
    {
        enabled_.store(false);
        RCLCPP_INFO(node_->get_logger(), "🔗 TF control DISABLED!");
    }

    void TfMarkerWrapper::timerCallback()
    {
        if (!enabled_.load())
        {
            return;
        }

        // Check update frequency
        auto currentTime = node_->now();
        double timeSinceLastUpdate = (currentTime - lastUpdateTime_).seconds();
        double updateInterval = 1.0 / updateRate_;

        if (timeSinceLastUpdate < updateInterval)
        {
            return;
        }
        lastUpdateTime_ = currentTime;

        try
        {
            // Lookup transform from source frame to target frame
            // Use Time(0) to get the latest available transform
            rclcpp::Time timeStamp = rclcpp::Time(0);
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped = tfBuffer_.lookupTransform(
                sourceFrame_, targetFrame_, timeStamp);

            // Extract position and orientation from transform
            Eigen::Vector3d position;
            position.x() = transformStamped.transform.translation.x;
            position.y() = transformStamped.transform.translation.y;
            position.z() = transformStamped.transform.translation.z;

            Eigen::Quaterniond orientation;
            orientation.x() = transformStamped.transform.rotation.x;
            orientation.y() = transformStamped.transform.rotation.y;
            orientation.z() = transformStamped.transform.rotation.z;
            orientation.w() = transformStamped.transform.rotation.w;
            orientation.normalize();

            // Check if change passes threshold
            bool shouldUpdate = false;
            if (!hasPrevTf_)
            {
                // First update - always update
                shouldUpdate = true;
                hasPrevTf_ = true;
            }
            else
            {
                // Check position change
                double positionChange = (position - prevPosition_).norm();
                bool positionChanged = positionChange >= positionThreshold_;

                // Check orientation change (angular difference in radians)
                Eigen::Quaterniond orientationDiff = prevOrientation_.inverse() * orientation;
                double orientationChange = 2.0 * std::acos(std::abs(orientationDiff.w()));
                bool orientationChanged = orientationChange >= orientationThreshold_;

                shouldUpdate = positionChanged || orientationChanged;

                if (shouldUpdate)
                {
                    RCLCPP_DEBUG(node_->get_logger(), 
                                "🔗 Transform change detected - Position: %.4f m (threshold: %.4f), Orientation: %.4f rad (threshold: %.4f)",
                                positionChange, positionThreshold_, orientationChange, orientationThreshold_);
                }
            }

            // Update marker pose only if threshold is passed
            if (shouldUpdate)
            {
                updateMarkerPose(position, orientation);
                
                // Cache the current transform
                prevPosition_ = position;
                prevOrientation_ = orientation;
            }
        }
        catch (tf2::TransformException& ex)
        {
            // Only log warnings occasionally to avoid spam
            static rclcpp::Time lastWarningTime(0, 0, RCL_ROS_TIME);
            auto currentTime = node_->now();
            if ((currentTime - lastWarningTime).seconds() > 1.0)
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                    "🔗 Could not transform %s to %s: %s", 
                    targetFrame_.c_str(), sourceFrame_.c_str(), ex.what());
                lastWarningTime = currentTime;
            }
        }
    }

    void TfMarkerWrapper::updateMarkerPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
    {
        if (markerControl_->getMode() == IMarkerControl::Mode::SINGLE_ARM)
        {
            markerControl_->setSingleArmPose(position, orientation);
            markerControl_->updateMarkerDisplay("Goal", position, orientation);
        }
        else
        {
            // Dual arm mode: update current active arm
            const auto activeArm = markerControl_->getActiveArm();
            markerControl_->setDualArmPose(activeArm, position, orientation);

            const std::string markerName =
                (activeArm == IMarkerControl::ArmType::LEFT) ? "LeftArmGoal" : "RightArmGoal";
            markerControl_->updateMarkerDisplay(markerName, position, orientation);
        }

        // Publish target trajectories if continuous mode is enabled
        if (markerControl_->isContinuousMode())
        {
            if (markerControl_->getMode() == IMarkerControl::Mode::SINGLE_ARM)
            {
                markerControl_->sendSingleArmTrajectories();
            }
            else
            {
                markerControl_->sendDualArmTrajectories();
            }
        }

        // Output debug information
        RCLCPP_DEBUG(node_->get_logger(), "🔗 Updated %s marker position from TF: [%.3f, %.3f, %.3f]",
                     markerControl_->getMode() == IMarkerControl::Mode::SINGLE_ARM ? "single arm" :
                     markerControl_->getActiveArm() == IMarkerControl::ArmType::LEFT ? "left arm" : "right arm",
                     position.x(), position.y(), position.z());
    }
} // namespace ocs2

