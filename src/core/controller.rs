use super::types::{Odometry, Pose, Twist};
use bevy::math::EulerRot;
use bevy::prelude::Vec3;

pub trait LocalController: Send + Sync {
    /// Calculate current velocity command based on current odometry and local path
    fn compute_velocity_commands(&mut self, current_odom: &Odometry, path: &[Pose]) -> Twist;
}

pub struct DwaController {
    max_linear_vel: f32,
    max_angular_vel: f32,
    linear_acc: f32,
    angular_acc: f32,
}

impl DwaController {
    pub fn new() -> Self {
        Self {
            max_linear_vel: 1.0,
            max_angular_vel: 1.0,
            linear_acc: 0.5,
            angular_acc: 0.5,
        }
    }
}

impl LocalController for DwaController {
    fn compute_velocity_commands(&mut self, current_odom: &Odometry, path: &[Pose]) -> Twist {
        if path.is_empty() {
            return Twist::default();
        }

        let next_point = &path[0]; // For simplicity, follow the first point
        let current_pose = &current_odom.pose;

        // Compute direction to next point
        let dx = next_point.position.x - current_pose.position.x;
        let dy = next_point.position.y - current_pose.position.y;
        let distance = (dx * dx + dy * dy).sqrt();

        if distance < 0.1 {
            // Close enough, stop or go to next
            return Twist::default();
        }

        // Desired direction
        let desired_yaw = dy.atan2(dx);

        // Current yaw from orientation
        let current_yaw = current_pose.orientation.to_euler(EulerRot::ZYX).2; // Assuming Z is yaw

        // Angular error
        let mut yaw_error = desired_yaw - current_yaw;
        // Normalize to -pi to pi
        while yaw_error > std::f32::consts::PI {
            yaw_error -= 2.0 * std::f32::consts::PI;
        }
        while yaw_error < -std::f32::consts::PI {
            yaw_error += 2.0 * std::f32::consts::PI;
        }

        // Set angular velocity proportional to error
        let angular_vel = yaw_error.clamp(-self.max_angular_vel, self.max_angular_vel);

        // Set linear velocity, reduce if turning a lot
        let linear_vel = if yaw_error.abs() < 0.1 {
            self.max_linear_vel.min(distance)
        } else {
            (self.max_linear_vel * (1.0 - yaw_error.abs() / std::f32::consts::PI)).max(0.1)
        };

        Twist {
            linear: Vec3::new(linear_vel, 0.0, 0.0),
            angular: Vec3::new(0.0, 0.0, angular_vel),
        }
    }
}
