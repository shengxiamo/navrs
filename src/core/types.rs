use std::sync::Arc;
use parking_lot::RwLock;
use bevy::prelude::{Vec3, Quat};

/// A 3D pose consists of position and orientation
#[derive(Clone, Debug)]
pub struct Pose {
    pub position: Vec3,
    pub orientation: Quat,
}

impl Default for Pose {
    fn default() -> Self {
        Self {
            position: Vec3::ZERO,
            orientation: Quat::IDENTITY,
        }
    }
}

/// Twist represents linear and angular velocity
#[derive(Clone, Debug)]
pub struct Twist {
    pub linear: Vec3,
    pub angular: Vec3,
}

impl Default for Twist {
    fn default() -> Self {
        Self {
            linear: Vec3::ZERO,
            angular: Vec3::ZERO,
        }
    }
}

/// PointCloud representing a collection of 3D points
#[derive(Clone, Debug, Default)]
pub struct PointCloud {
    pub points: Vec<Vec3>,
}

/// LaserScan representing 2D laser scan data
#[derive(Clone, Debug, Default)]
pub struct LaserScan {
    pub ranges: Vec<f32>,
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub range_min: f32,
    pub range_max: f32,
}

/// 2D Costmap representing obstacles
#[derive(Clone, Debug)]
pub struct Costmap2D {
    pub width: u32,
    pub height: u32,
    pub resolution: f32,
    pub origin: Pose,
    pub data: Vec<u8>,
}

impl Default for Costmap2D {
    fn default() -> Self {
        Self {
            width: 0,
            height: 0,
            resolution: 0.05,
            origin: Pose::default(),
            data: Vec::new(),
        }
    }
}

/// Shared Zero-Copy Data Types
pub type SharedPointCloud = Arc<RwLock<PointCloud>>;
pub type SharedCostmap2D = Arc<RwLock<Costmap2D>>;
pub type SharedLaserScan = Arc<RwLock<LaserScan>>;

/// Core Navigation State
#[derive(Clone, Debug)]
pub struct Odometry {
    pub pose: Pose,
    pub twist: Twist,
}

impl Default for Odometry {
    fn default() -> Self {
        Self {
            pose: Pose::default(),
            twist: Twist::default(),
        }
    }
}
