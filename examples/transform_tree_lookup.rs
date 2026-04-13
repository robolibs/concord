use glam::{DQuat, DVec3};

use concord::{Transform, TransformTree};

#[derive(Debug, Clone, Copy)]
struct World;
#[derive(Debug, Clone, Copy)]
struct Base;
#[derive(Debug, Clone, Copy)]
struct Camera;

fn main() {
    let mut tree = TransformTree::new();
    tree.register_frame::<World>("world");
    tree.register_frame::<Base>("base");
    tree.register_frame::<Camera>("camera");

    tree.set_transform(
        "world",
        "base",
        Transform::<World, Base>::from_qt(DQuat::IDENTITY, DVec3::new(1.0, 2.0, 0.0)),
    );
    tree.set_transform(
        "base",
        "camera",
        Transform::<Base, Camera>::from_qt(DQuat::IDENTITY, DVec3::new(0.0, 0.0, 1.0)),
    );

    let tf = tree.lookup("world", "camera").expect("path exists");
    println!("camera in world: {:?}", tf.translation);
}
