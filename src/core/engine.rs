// src/core/engine.rs
use crossbeam_channel::{Receiver, Sender};
use std::sync::Arc;
use parking_lot::RwLock;
use std::thread;
use std::time::Duration;

use super::types::{Odometry, Pose, SharedCostmap2D, Twist};
use super::planner::{GlobalPlanner, AStarPlanner};
use super::controller::{LocalController, DwaController};

/// Events that drive the navigation engine
pub enum NavEvent {
    SetGoal(Pose),
    UpdateOdometry(Odometry),
    Stop,
}

pub struct NavEngine {
    costmap: SharedCostmap2D,
    current_odom: Odometry, // We'll hold local odom here
    planner: Box<dyn GlobalPlanner>,
    controller: Box<dyn LocalController>,
}

impl NavEngine {
    pub fn new(costmap: SharedCostmap2D) -> Self {
        Self {
            costmap,
            current_odom: Odometry::default(),
            planner: Box::new(AStarPlanner),
            controller: Box::new(DwaController::new()),
        }
    }

    /// Spawns the navigation engine loop on a separate thread.
    /// Returns the Sender to send commands (like goal positions or odometry updates)
    /// and a Receiver to get velocity commands (Twist).
    pub fn spawn_loop(mut self) -> (Sender<NavEvent>, Receiver<Twist>) {
        let (event_tx, event_rx) = crossbeam_channel::unbounded();
        let (cmd_vel_tx, cmd_vel_rx) = crossbeam_channel::unbounded();

        thread::spawn(move || {
            let mut active_goal: Option<Pose> = None;
            let mut current_path: Option<Vec<Pose>> = None;
            let mut current_odom = Odometry::default();

            loop {
                // Process incoming events without blocking long
                while let Ok(event) = event_rx.try_recv() {
                    match event {
                        NavEvent::SetGoal(goal) => {
                            println!("[NavEngine] Received new goal!");
                            active_goal = Some(goal);
                            // In a real system, we re-plan immediately or on the next loop
                            current_path = None;
                        }
                        NavEvent::UpdateOdometry(odom) => {
                            current_odom = odom;
                        }
                        NavEvent::Stop => {
                            println!("[NavEngine] Stopping engine thread.");
                            return;
                        }
                    }
                }

                if let Some(goal) = &active_goal {
                    // Plan if we don't have a path
                    if current_path.is_none() {
                        current_path = self.planner.plan(&current_odom.pose, goal, &self.costmap);
                    }

                    // Control
                    if let Some(path) = &current_path {
                        let cmd_vel = self.controller.compute_velocity_commands(&current_odom, path);
                        let _ = cmd_vel_tx.send(cmd_vel);
                    }
                } else {
                    // Stop robot if no goal
                    let _ = cmd_vel_tx.send(Twist::default());
                }

                // Control the tick rate (e.g., 20Hz)
                thread::sleep(Duration::from_millis(50));
            }
        });

        (event_tx, cmd_vel_rx)
    }
}