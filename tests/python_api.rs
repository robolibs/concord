#![cfg(feature = "python")]

use pyo3::prelude::*;
use pyo3::types::{PyDict, PyModule};

#[test]
fn python_module_exposes_conversion_and_tree_surface() {
    Python::with_gil(|py| {
        let module = PyModule::new(py, "concord").expect("module");
        concord::python::register_python_module(&module).expect("register");

        let wgs_to_ecf = module.getattr("wgs_to_ecf").expect("wgs_to_ecf");
        let ecf = wgs_to_ecf
            .call1(((52.0_f64, 4.0_f64, 10.0_f64),))
            .expect("wgs_to_ecf");
        let ecf_tuple: (f64, f64, f64) = ecf.extract().expect("ecf tuple");
        assert!(ecf_tuple.0.abs() > 1_000.0);

        let wgs_to_utm = module.getattr("wgs_to_utm").expect("wgs_to_utm");
        let utm = wgs_to_utm
            .call1(((52.0_f64, 4.0_f64, 10.0_f64),))
            .expect("wgs_to_utm")
            .downcast_into::<PyDict>()
            .expect("utm dict");
        let zone: i32 = utm
            .get_item("zone")
            .expect("zone item")
            .expect("zone")
            .extract()
            .expect("zone value");
        assert_eq!(zone, 31);

        let wgs_to_enu = module.getattr("wgs_to_enu").expect("wgs_to_enu");
        let enu = wgs_to_enu
            .call1((
                (52.0_f64, 4.0_f64, 10.0_f64),
                (52.0001_f64, 4.0002_f64, 12.0_f64),
            ))
            .expect("wgs_to_enu")
            .downcast_into::<PyDict>()
            .expect("enu dict");
        let east: f64 = enu
            .get_item("east")
            .expect("east item")
            .expect("east")
            .extract()
            .expect("east value");
        assert!(east.abs() > 1.0);

        let cls = module.getattr("TransformTree").expect("TransformTree");
        let tree = cls.call0().expect("tree");
        tree.call_method1("register_frame", ("world",))
            .expect("register world");
        tree.call_method1("register_frame", ("base",))
            .expect("register base");
        tree.call_method1("register_frame", ("camera",))
            .expect("register camera");
        tree.call_method1(
            "set_transform",
            (
                "world",
                "base",
                (0.0_f64, 0.0_f64, 0.0_f64, 1.0_f64),
                (1.0_f64, 2.0_f64, 0.0_f64),
            ),
        )
        .expect("set world->base");
        tree.call_method1(
            "set_transform",
            (
                "base",
                "camera",
                (0.0_f64, 0.0_f64, 0.0_f64, 1.0_f64),
                (0.0_f64, 0.0_f64, 1.0_f64),
            ),
        )
        .expect("set base->camera");

        let can_transform: bool = tree
            .call_method1("can_transform", ("world", "camera"))
            .expect("can_transform")
            .extract()
            .expect("extract bool");
        assert!(can_transform);

        let lookup = tree
            .call_method1("lookup", ("world", "camera"))
            .expect("lookup")
            .downcast_into::<PyDict>()
            .expect("lookup dict");
        let translation: (f64, f64, f64) = lookup
            .get_item("translation")
            .expect("translation item")
            .expect("translation")
            .extract()
            .expect("translation tuple");
        assert_eq!(translation, (1.0, 2.0, 1.0));
    });
}
