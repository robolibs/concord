use datapod::{Point, Quaternion};

use concord::{TimedTransformTree, Transform};

#[derive(Debug, Clone, Copy)]
struct World;
#[derive(Debug, Clone, Copy)]
struct Odom;

fn main() {
    let mut tree = TimedTransformTree::new();
    tree.register_frame::<World>("world");
    tree.register_frame::<Odom>("odom");

    tree.set_transform(
        "world",
        "odom",
        Transform::<World, Odom>::from_qt(Quaternion::identity(), Point::new(0.0, 0.0, 0.0)),
        1.0,
    );
    tree.set_transform(
        "world",
        "odom",
        Transform::<World, Odom>::from_qt(Quaternion::identity(), Point::new(10.0, 0.0, 0.0)),
        2.0,
    );

    let tf = tree
        .lookup("world", "odom", 1.5)
        .expect("interpolated transform");
    println!("odom in world at t=1.5: {:?}", tf.translation);
}
