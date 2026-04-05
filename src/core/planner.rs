use super::types::{Pose, SharedCostmap2D};
use bevy::prelude::Vec3;
use std::collections::{BinaryHeap, HashMap};
use parking_lot::RwLockReadGuard;

#[derive(Clone, Eq, PartialEq)]
struct Node {
    position: (i32, i32),
    cost: u32,
    heuristic: u32,
    parent: Option<(i32, i32)>,
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        (other.cost + other.heuristic).cmp(&(self.cost + self.heuristic))
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

pub trait GlobalPlanner: Send + Sync {
    /// Plan a path from start to goal
    fn plan(&mut self, start: &Pose, goal: &Pose, costmap: &SharedCostmap2D) -> Option<Vec<Pose>>;
}

pub struct AStarPlanner;

impl AStarPlanner {
    fn pose_to_grid(&self, pose: &Pose, costmap: &RwLockReadGuard<super::types::Costmap2D>) -> (i32, i32) {
        let x = ((pose.position.x - costmap.origin.position.x) / costmap.resolution).round() as i32;
        let y = ((pose.position.y - costmap.origin.position.y) / costmap.resolution).round() as i32;
        (x, y)
    }

    fn grid_to_pose(&self, grid: (i32, i32), costmap: &RwLockReadGuard<super::types::Costmap2D>) -> Pose {
        let x = grid.0 as f32 * costmap.resolution + costmap.origin.position.x;
        let y = grid.1 as f32 * costmap.resolution + costmap.origin.position.y;
        Pose {
            position: Vec3::new(x, y, 0.0),
            orientation: costmap.origin.orientation,
        }
    }

    fn heuristic(&self, a: (i32, i32), b: (i32, i32)) -> u32 {
        ((a.0 - b.0).abs() + (a.1 - b.1).abs()) as u32
    }

    fn neighbors(&self, pos: (i32, i32), costmap: &RwLockReadGuard<super::types::Costmap2D>) -> Vec<(i32, i32)> {
        let mut neighbors = Vec::new();
        for dx in -1..=1 {
            for dy in -1..=1 {
                if dx == 0 && dy == 0 { continue; }
                let nx = pos.0 + dx;
                let ny = pos.1 + dy;
                if nx >= 0 && nx < costmap.width as i32 && ny >= 0 && ny < costmap.height as i32 {
                    let idx = (ny * costmap.width as i32 + nx) as usize;
                    if idx < costmap.data.len() && costmap.data[idx] < 100 { // Assume <100 is free
                        neighbors.push((nx, ny));
                    }
                }
            }
        }
        neighbors
    }
}

impl GlobalPlanner for AStarPlanner {
    fn plan(&mut self, start: &Pose, goal: &Pose, costmap: &SharedCostmap2D) -> Option<Vec<Pose>> {
        let costmap_guard = costmap.read();
        if costmap_guard.data.is_empty() {
            return Some(vec![start.clone(), goal.clone()]); // Mock if no costmap
        }
        let start_grid = self.pose_to_grid(start, &costmap_guard);
        let goal_grid = self.pose_to_grid(goal, &costmap_guard);

        let mut open_set = BinaryHeap::new();
        let mut came_from = HashMap::new();
        let mut g_score = HashMap::new();
        let mut f_score = HashMap::new();

        open_set.push(Node {
            position: start_grid,
            cost: 0,
            heuristic: self.heuristic(start_grid, goal_grid),
            parent: None,
        });
        g_score.insert(start_grid, 0);
        f_score.insert(start_grid, self.heuristic(start_grid, goal_grid));

        while let Some(current) = open_set.pop() {
            if current.position == goal_grid {
                // Reconstruct path
                let mut path = Vec::new();
                let mut current_pos = current.position;
                while let Some(parent) = came_from.get(&current_pos) {
                    path.push(self.grid_to_pose(current_pos, &costmap_guard));
                    current_pos = *parent;
                }
                path.push(self.grid_to_pose(start_grid, &costmap_guard));
                path.reverse();
                return Some(path);
            }

            for neighbor in self.neighbors(current.position, &costmap_guard) {
                let tentative_g = g_score[&current.position] + 1; // Assume cost 1 for each step
                if tentative_g < *g_score.get(&neighbor).unwrap_or(&u32::MAX) {
                    came_from.insert(neighbor, current.position);
                    g_score.insert(neighbor, tentative_g);
                    f_score.insert(neighbor, tentative_g + self.heuristic(neighbor, goal_grid));
                    open_set.push(Node {
                        position: neighbor,
                        cost: tentative_g,
                        heuristic: self.heuristic(neighbor, goal_grid),
                        parent: Some(current.position),
                    });
                }
            }
        }
        None
    }
}
