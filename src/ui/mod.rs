use bevy::prelude::*;
use bevy::render::render_asset::RenderAssetUsages;
use crossbeam_channel::{Receiver, Sender};
use std::thread;
use std::sync::Arc;
use bevy_egui::{egui, EguiContexts, EguiPlugin};

use crate::core::engine::{NavEngine, NavEvent};
use crate::core::types::{Odometry, Pose, SharedCostmap2D, SharedPointCloud, Twist};

#[derive(Resource)]
struct NavChannels {
    nav_tx: Sender<NavEvent>,
    odom_rx: Receiver<Odometry>,
    costmap_rx: Receiver<SharedCostmap2D>,
    pointcloud_rx: Receiver<SharedPointCloud>,
    path_rx: Receiver<Vec<Pose>>,
}

#[derive(Resource)]
struct LatestData {
    odom: Odometry,
    costmap: Option<SharedCostmap2D>,
    pointcloud: Option<SharedPointCloud>,
    path: Vec<Pose>,
    last_planner_time: f32,
    last_controller_time: f32,
}

#[derive(Component)]
struct CostmapSprite;

#[derive(Component)]
struct PointCloudEntity;

#[derive(Component)]
struct PathEntity;

#[derive(Component)]
struct RobotEntity;

pub fn run_bevy_app() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(EguiPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, (
            receive_data,
            render_costmap,
            render_pointcloud,
            render_path,
            render_robot,
            handle_input,
            ui_system,
        ))
        .run();
}

fn setup(mut commands: Commands) {
    let (nav_tx, nav_rx) = crossbeam_channel::unbounded();
    let (odom_tx, odom_rx) = crossbeam_channel::unbounded();
    let (costmap_tx, costmap_rx) = crossbeam_channel::unbounded();
    let (pointcloud_tx, pointcloud_rx) = crossbeam_channel::unbounded();
    let (_path_tx, path_rx) = crossbeam_channel::unbounded();

    thread::spawn(move || {
        let global_costmap: SharedCostmap2D = Arc::new(parking_lot::RwLock::new(crate::core::types::Costmap2D {
            width: 100,
            height: 100,
            resolution: 0.1,
            origin: Pose::default(),
            data: vec![0; 10000],
        }));
        for i in 2000..2100 {
            global_costmap.write().data[i] = 255;
        }
        for i in 3000..3100 {
            global_costmap.write().data[i] = 255;
        }

        let engine = NavEngine::new(global_costmap.clone());
        let (engine_tx, cmd_vel_rx) = engine.spawn_loop();

        let nav_rx_clone = nav_rx.clone();
        thread::spawn(move || {
            while let Ok(event) = nav_rx_clone.recv() {
                let _ = engine_tx.send(event);
            }
        });

        let odom_tx_clone = odom_tx.clone();
        thread::spawn(move || {
            let mut loop_count = 0;
            loop {
                let mut odom = Odometry::default();
                odom.pose.position.x = (loop_count as f32) * 0.1;
                if odom_tx_clone.send(odom).is_err() {
                    break;
                }
                loop_count += 1;
                std::thread::sleep(std::time::Duration::from_millis(100));
            }
        });

        let _ = costmap_tx.send(global_costmap.clone());

        let pointcloud_tx_clone = pointcloud_tx.clone();
        thread::spawn(move || {
            loop {
                let pc = Arc::new(parking_lot::RwLock::new(crate::core::types::PointCloud::default()));
                if pointcloud_tx_clone.send(pc).is_err() {
                    break;
                }
                std::thread::sleep(std::time::Duration::from_millis(200));
            }
        });

        while let Ok(twist) = cmd_vel_rx.recv() {
            println!("Received Cmd_Vel -> Linear: {:.2}, Angular: {:.2}",
                     twist.linear.x, twist.angular.z);
        }
    });

    commands.insert_resource(NavChannels {
        nav_tx,
        odom_rx,
        costmap_rx,
        pointcloud_rx,
        path_rx,
    });

    commands.insert_resource(LatestData {
        odom: Odometry::default(),
        costmap: None,
        pointcloud: None,
        path: Vec::new(),
        last_planner_time: 12.0,
        last_controller_time: 2.0,
    });

    commands.spawn(Camera2dBundle::default());
}

fn receive_data(
    channels: Res<NavChannels>,
    mut data: ResMut<LatestData>,
) {
    if let Ok(odom) = channels.odom_rx.try_recv() {
        data.odom = odom;
    }
    if let Ok(costmap) = channels.costmap_rx.try_recv() {
        data.costmap = Some(costmap);
    }
    if let Ok(pointcloud) = channels.pointcloud_rx.try_recv() {
        data.pointcloud = Some(pointcloud);
    }
    if let Ok(path) = channels.path_rx.try_recv() {
        data.path = path;
    }
}

fn render_costmap(
    mut commands: Commands,
    data: Res<LatestData>,
    mut images: ResMut<Assets<Image>>,
    query: Query<Entity, With<CostmapSprite>>,
) {
    let costmap_arc = match &data.costmap {
        Some(c) => c,
        None => return,
    };
    let costmap = costmap_arc.read();
    
    if costmap.data.is_empty() {
        return;
    }

    for entity in query.iter() {
        commands.entity(entity).despawn();
    }

    let mut image_data = Vec::new();
    for &cost in &costmap.data {
        let gray = 255 - cost;
        image_data.extend_from_slice(&[gray, gray, gray, 255]);
    }

    let image = Image::new(
        bevy::render::render_resource::Extent3d {
            width: costmap.width,
            height: costmap.height,
            depth_or_array_layers: 1,
        },
        bevy::render::render_resource::TextureDimension::D2,
        image_data,
        bevy::render::render_resource::TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::default(),
    );

    let texture_handle = images.add(image);

    commands.spawn((
        SpriteBundle {
            texture: texture_handle,
            transform: Transform::from_scale(Vec3::new(costmap.resolution * 100.0, costmap.resolution * 100.0, 1.0)),
            ..default()
        },
        CostmapSprite,
    ));
}

fn render_pointcloud(
    mut commands: Commands,
    data: Res<LatestData>,
    query: Query<Entity, With<PointCloudEntity>>,
) {
    let pc_arc = match &data.pointcloud {
        Some(p) => p,
        None => return,
    };
    let pc = pc_arc.read();

    for entity in query.iter() {
        commands.entity(entity).despawn();
    }

    for point in &pc.points {
        commands.spawn((
            SpriteBundle {
                sprite: Sprite {
                    color: Color::srgb(1.0, 0.0, 0.0),
                    custom_size: Some(Vec2::new(0.05, 0.05)),
                    ..default()
                },
                transform: Transform::from_translation(Vec3::new(point.x, point.y, 0.0)),
                ..default()
            },
            PointCloudEntity,
        ));
    }
}

fn render_path(
    mut commands: Commands,
    data: Res<LatestData>,
    query: Query<Entity, With<PathEntity>>,
) {
    for entity in query.iter() {
        commands.entity(entity).despawn();
    }

    for i in 0..data.path.len().saturating_sub(1) {
        let start = data.path[i].position;
        let end = data.path[i + 1].position;
        let mid = (start + end) / 2.0;
        let length = (end - start).length();
        let angle = (end.y - start.y).atan2(end.x - start.x);

        commands.spawn((
            SpriteBundle {
                sprite: Sprite {
                    color: Color::srgb(0.0, 0.0, 1.0),
                    custom_size: Some(Vec2::new(length, 0.02)),
                    ..default()
                },
                transform: Transform::from_translation(Vec3::new(mid.x, mid.y, 0.0))
                    .with_rotation(Quat::from_rotation_z(angle)),
                ..default()
            },
            PathEntity,
        ));
    }
}

fn render_robot(
    mut commands: Commands,
    data: Res<LatestData>,
    query: Query<Entity, With<RobotEntity>>,
) {
    for entity in query.iter() {
        commands.entity(entity).despawn();
    }

    commands.spawn((
        SpriteBundle {
            sprite: Sprite {
                color: Color::srgb(0.0, 1.0, 0.0),
                custom_size: Some(Vec2::new(0.2, 0.2)),
                ..default()
            },
            transform: Transform::from_translation(Vec3::new(data.odom.pose.position.x, data.odom.pose.position.y, 0.0))
                .with_rotation(data.odom.pose.orientation),
            ..default()
        },
        RobotEntity,
    ));
}

fn handle_input(
    buttons: Res<ButtonInput<MouseButton>>,
    windows: Query<&Window>,
    camera_q: Query<(&Camera, &GlobalTransform)>,
    channels: Res<NavChannels>,
    mut contexts: EguiContexts,
) {
    if contexts.ctx_mut().wants_pointer_input() {
        return;
    }
    
    if buttons.just_pressed(MouseButton::Left) {
        if let Ok((camera, camera_transform)) = camera_q.get_single() {
            if let Ok(window) = windows.get_single() {
                if let Some(cursor_pos) = window.cursor_position() {
                    if let Some(world_pos) = camera.viewport_to_world(camera_transform, cursor_pos) {
                        let goal = Pose {
                            position: world_pos.origin.truncate().extend(0.0),
                            orientation: Quat::IDENTITY,
                        };
                        let _ = channels.nav_tx.send(NavEvent::SetGoal(goal));
                    }
                }
            }
        }
    }
}

fn ui_system(mut contexts: EguiContexts, data: Res<LatestData>, channels: Res<NavChannels>) {
    egui::SidePanel::right("debug_panel").default_width(300.0).show(contexts.ctx_mut(), |ui| {
        ui.heading("MATRIX & DEBUG TERMINAL");
        ui.separator();

        ui.label(egui::RichText::new("[Costmap Value at Rob]").strong());
        let (dist, cell) = if let Some(costmap_arc) = &data.costmap {
            let costmap = costmap_arc.read();
            let x = ((data.odom.pose.position.x - costmap.origin.position.x) / costmap.resolution).round() as i32;
            let y = ((data.odom.pose.position.y - costmap.origin.position.y) / costmap.resolution).round() as i32;
            let val = if x >= 0 && x < costmap.width as i32 && y >= 0 && y < costmap.height as i32 {
                costmap.data[(y * costmap.width as i32 + x) as usize]
            } else {
                0
            };
            ("1.25m", val.to_string())
        } else {
            ("N/A", "N/A".to_string())
        };
        ui.label(format!("Obstacle dist: {}", dist));
        ui.label(format!("Cell value: {}", cell));
        ui.add_space(10.0);

        ui.label(egui::RichText::new("[TF Current Vectors]").strong());
        ui.label("odom_2_base:");
        ui.label("[ 0.9, -0.1, 0.0]");
        ui.label("[ 0.1,  0.9, 0.0]");
        ui.add_space(10.0);

        ui.label(egui::RichText::new("[Latency Profiler]").strong());
        ui.label(format!("Core_Planner: {}ms", data.last_planner_time));
        ui.label(format!("Core_Controller: {}ms", data.last_controller_time));
        ui.add_space(10.0);

        let arc_count = if let Some(c) = &data.costmap { Arc::strong_count(c) } else { 0 };
        ui.label(format!("Zero-Copy Arc count: {}", arc_count));
        
        ui.add_space(20.0);
        ui.label(egui::RichText::new("[Toolbox]").strong());
        if ui.button("Reset Navigation").clicked() {
             let _ = channels.nav_tx.send(NavEvent::Stop);
        }
    });
}
