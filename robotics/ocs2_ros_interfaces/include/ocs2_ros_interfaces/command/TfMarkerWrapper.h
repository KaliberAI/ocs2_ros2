#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <atomic>
#include <string>
#include "ocs2_ros_interfaces/command/IMarkerControl.h"

namespace ocs2 {

    /**
     * TF marker wrapper that listens to TF transforms and updates marker pose.
     * This class acts as a wrapper/adapter between TF transforms and marker control.
     * It is decoupled from specific marker implementations through the IMarkerControl interface.
     */
    class TfMarkerWrapper {
    public:
        /**
         * Constructor
         * @param node ROS node handle
         * @param markerControl Pointer to marker control interface
         * @param targetFrame Frame name to listen to for TF transforms
         * @param sourceFrame Source frame for TF lookup (default: "world")
         * @param updateRate Update rate for TF processing (Hz)
         */
        TfMarkerWrapper(
            rclcpp::Node::SharedPtr node,
            IMarkerControl* markerControl,
            const std::string& targetFrame,
            const std::string& sourceFrame = "world",
            double updateRate = 30.0);

        /**
         * Destructor
         */
        ~TfMarkerWrapper() = default;

        /**
         * Enable TF control
         */
        void enable();

        /**
         * Disable TF control
         */
        void disable();

        /**
         * Check if TF control is enabled
         * @return true if enabled, false otherwise
         */
        bool isEnabled() const { return enabled_.load(); }

        /**
         * Set target frame to listen to
         * @param frame Frame name
         */
        void setTargetFrame(const std::string& frame) { targetFrame_ = frame; }

        /**
         * Set source frame for TF lookup
         * @param frame Frame name
         */
        void setSourceFrame(const std::string& frame) { sourceFrame_ = frame; }

        /**
         * Get current target frame
         * @return Target frame name
         */
        const std::string& getTargetFrame() const { return targetFrame_; }

        /**
         * Get current source frame
         * @return Source frame name
         */
        const std::string& getSourceFrame() const { return sourceFrame_; }

        /**
         * Set update rate
         * @param rate Update rate in Hz
         */
        void setUpdateRate(double rate) { updateRate_ = rate; }

    private:
        /**
         * Timer callback function to check for TF updates
         */
        void timerCallback();

        /**
         * Update marker pose based on TF transform
         * @param position New position
         * @param orientation New orientation
         */
        void updateMarkerPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);

        // ROS components
        rclcpp::Node::SharedPtr node_;
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Marker control interface
        IMarkerControl* markerControl_;

        // Frame names
        std::string targetFrame_;
        std::string sourceFrame_;

        // Control parameters
        double updateRate_;

        // State management
        std::atomic<bool> enabled_;
        std::mutex stateMutex_;

        // Timing control
        rclcpp::Time lastUpdateTime_;
    };

} // namespace ocs2

