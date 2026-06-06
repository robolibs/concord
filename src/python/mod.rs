use datapod::{Point, Quaternion};
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyModule};
use pyo3::wrap_pyfunction;

use crate::{Enu, Geo, Ned, Transform, TransformTree, Utm, Wgs, convert, enu_to_ned, ned_to_enu};

fn py_runtime_error(err: crate::Error) -> PyErr {
    PyRuntimeError::new_err(err.to_string())
}

fn geo_tuple(value: impl Into<Geo>) -> (f64, f64, f64) {
    let value = value.into();
    (value.latitude, value.longitude, value.altitude)
}

fn ecf_tuple(value: crate::Ecf) -> (f64, f64, f64) {
    (value.x, value.y, value.z)
}

fn utm_dict<'py>(py: Python<'py>, utm: Utm) -> PyResult<Bound<'py, PyDict>> {
    let dict = PyDict::new(py);
    dict.set_item("zone", utm.zone)?;
    dict.set_item("band", utm.band.to_string())?;
    dict.set_item("easting", utm.easting)?;
    dict.set_item("northing", utm.northing)?;
    dict.set_item("altitude", utm.altitude)?;
    Ok(dict)
}

fn enu_dict<'py>(py: Python<'py>, enu: Enu) -> PyResult<Bound<'py, PyDict>> {
    let dict = PyDict::new(py);
    dict.set_item("east", enu.east())?;
    dict.set_item("north", enu.north())?;
    dict.set_item("up", enu.up())?;
    dict.set_item("origin", geo_tuple(enu.origin))?;
    Ok(dict)
}

fn ned_dict<'py>(py: Python<'py>, ned: Ned) -> PyResult<Bound<'py, PyDict>> {
    let dict = PyDict::new(py);
    dict.set_item("north", ned.north())?;
    dict.set_item("east", ned.east())?;
    dict.set_item("down", ned.down())?;
    dict.set_item("origin", geo_tuple(ned.origin))?;
    Ok(dict)
}

fn transform_dict<'py>(
    py: Python<'py>,
    rotation: Quaternion,
    translation: Point,
) -> PyResult<Bound<'py, PyDict>> {
    let dict = PyDict::new(py);
    dict.set_item("rotation", (rotation.x, rotation.y, rotation.z, rotation.w))?;
    dict.set_item("translation", (translation.x, translation.y, translation.z))?;
    Ok(dict)
}

#[pyfunction]
fn wgs_to_ecf(wgs: (f64, f64, f64)) -> (f64, f64, f64) {
    ecf_tuple(crate::to_ecf(Wgs::new(wgs.0, wgs.1, wgs.2)))
}

#[pyfunction]
fn ecf_to_wgs(ecf: (f64, f64, f64)) -> (f64, f64, f64) {
    geo_tuple(crate::to_wgs(crate::Ecf::new(ecf.0, ecf.1, ecf.2)))
}

#[pyfunction]
#[pyo3(signature = (ecf, tolerance=1e-15))]
fn ecf_to_wgs_optimized(ecf: (f64, f64, f64), tolerance: f64) -> (f64, f64, f64) {
    geo_tuple(crate::to_wgs_optimized(
        crate::Ecf::new(ecf.0, ecf.1, ecf.2),
        tolerance,
    ))
}

#[pyfunction]
fn wgs_to_utm(py: Python<'_>, wgs: (f64, f64, f64)) -> PyResult<Py<PyAny>> {
    let utm = crate::to_utm(Wgs::new(wgs.0, wgs.1, wgs.2)).map_err(py_runtime_error)?;
    Ok(utm_dict(py, utm)?.into_any().unbind())
}

#[pyfunction]
fn utm_to_wgs(
    zone: i32,
    band: &str,
    easting: f64,
    northing: f64,
    altitude: f64,
) -> PyResult<(f64, f64, f64)> {
    let band = band
        .chars()
        .next()
        .ok_or_else(|| PyValueError::new_err("band must contain at least one character"))?;
    let wgs = crate::utm_to_wgs(Utm {
        zone,
        band: band as u32,
        easting,
        northing,
        altitude,
    })
    .map_err(py_runtime_error)?;
    Ok(geo_tuple(wgs))
}

#[pyfunction]
fn wgs_to_enu(
    py: Python<'_>,
    origin: (f64, f64, f64),
    wgs: (f64, f64, f64),
) -> PyResult<Py<PyAny>> {
    Ok(enu_dict(
        py,
        crate::to_enu(
            Geo::new(origin.0, origin.1, origin.2),
            Wgs::new(wgs.0, wgs.1, wgs.2),
        ),
    )?
    .into_any()
    .unbind())
}

#[pyfunction]
fn wgs_to_ned(
    py: Python<'_>,
    origin: (f64, f64, f64),
    wgs: (f64, f64, f64),
) -> PyResult<Py<PyAny>> {
    Ok(ned_dict(
        py,
        crate::to_ned(
            Geo::new(origin.0, origin.1, origin.2),
            Wgs::new(wgs.0, wgs.1, wgs.2),
        ),
    )?
    .into_any()
    .unbind())
}

#[pyfunction]
fn enu_to_wgs(enu: (f64, f64, f64), origin: (f64, f64, f64)) -> (f64, f64, f64) {
    geo_tuple(crate::to_wgs_from_enu(Enu::new(
        enu.0,
        enu.1,
        enu.2,
        Geo::new(origin.0, origin.1, origin.2),
    )))
}

#[pyfunction]
fn ned_to_wgs(ned: (f64, f64, f64), origin: (f64, f64, f64)) -> (f64, f64, f64) {
    geo_tuple(crate::to_wgs_from_ned(Ned::new(
        ned.0,
        ned.1,
        ned.2,
        Geo::new(origin.0, origin.1, origin.2),
    )))
}

#[pyfunction]
fn enu_to_ned_dict(
    py: Python<'_>,
    enu: (f64, f64, f64),
    origin: (f64, f64, f64),
) -> PyResult<Py<PyAny>> {
    Ok(ned_dict(
        py,
        enu_to_ned(Enu::new(
            enu.0,
            enu.1,
            enu.2,
            Geo::new(origin.0, origin.1, origin.2),
        )),
    )?
    .into_any()
    .unbind())
}

#[pyfunction]
fn ned_to_enu_dict(
    py: Python<'_>,
    ned: (f64, f64, f64),
    origin: (f64, f64, f64),
) -> PyResult<Py<PyAny>> {
    Ok(enu_dict(
        py,
        ned_to_enu(Ned::new(
            ned.0,
            ned.1,
            ned.2,
            Geo::new(origin.0, origin.1, origin.2),
        )),
    )?
    .into_any()
    .unbind())
}

#[pyfunction]
fn convert_wgs_to_enu(
    py: Python<'_>,
    wgs: (f64, f64, f64),
    origin: (f64, f64, f64),
) -> PyResult<Py<PyAny>> {
    let enu = convert(Wgs::new(wgs.0, wgs.1, wgs.2))
        .with_ref(Geo::new(origin.0, origin.1, origin.2))
        .to::<Enu>()
        .build()
        .map_err(py_runtime_error)?;
    Ok(enu_dict(py, enu)?.into_any().unbind())
}

#[pyclass(name = "TransformTree")]
pub struct PyTransformTree {
    inner: TransformTree,
}

#[pymethods]
impl PyTransformTree {
    #[new]
    fn new() -> Self {
        Self {
            inner: TransformTree::new(),
        }
    }

    fn register_frame(&mut self, name: &str) {
        self.inner.register_frame::<()>(name);
    }

    #[pyo3(signature = (to_frame, from_frame, rotation=(0.0, 0.0, 0.0, 1.0), translation=(0.0, 0.0, 0.0)))]
    fn set_transform(
        &mut self,
        to_frame: &str,
        from_frame: &str,
        rotation: (f64, f64, f64, f64),
        translation: (f64, f64, f64),
    ) -> PyResult<()> {
        if !self.inner.has_frame(to_frame) {
            return Err(PyValueError::new_err(format!("unknown frame: {to_frame}")));
        }
        if !self.inner.has_frame(from_frame) {
            return Err(PyValueError::new_err(format!(
                "unknown frame: {from_frame}"
            )));
        }

        let tf = Transform::<(), ()>::from_qt(
            Quaternion::new(rotation.3, rotation.0, rotation.1, rotation.2).normalized(),
            Point::new(translation.0, translation.1, translation.2),
        );
        self.inner.set_transform(to_frame, from_frame, tf);
        Ok(())
    }

    fn lookup<'py>(
        &mut self,
        py: Python<'py>,
        to_frame: &str,
        from_frame: &str,
    ) -> PyResult<Bound<'py, PyDict>> {
        let tf = self.inner.lookup(to_frame, from_frame).ok_or_else(|| {
            PyValueError::new_err(format!("no transform path from {from_frame} to {to_frame}"))
        })?;
        transform_dict(py, tf.rotation, tf.translation)
    }

    fn can_transform(&mut self, to_frame: &str, from_frame: &str) -> bool {
        self.inner.can_transform(to_frame, from_frame)
    }

    fn frame_names(&self) -> Vec<String> {
        self.inner.frame_names()
    }

    fn frame_count(&self) -> usize {
        self.inner.frame_count()
    }

    fn transform_count(&self) -> usize {
        self.inner.transform_count()
    }

    fn clear(&mut self) {
        self.inner.clear();
    }
}

pub fn register_python_module(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(wgs_to_ecf, module)?)?;
    module.add_function(wrap_pyfunction!(ecf_to_wgs, module)?)?;
    module.add_function(wrap_pyfunction!(ecf_to_wgs_optimized, module)?)?;
    module.add_function(wrap_pyfunction!(wgs_to_utm, module)?)?;
    module.add_function(wrap_pyfunction!(utm_to_wgs, module)?)?;
    module.add_function(wrap_pyfunction!(wgs_to_enu, module)?)?;
    module.add_function(wrap_pyfunction!(wgs_to_ned, module)?)?;
    module.add_function(wrap_pyfunction!(enu_to_wgs, module)?)?;
    module.add_function(wrap_pyfunction!(ned_to_wgs, module)?)?;
    module.add_function(wrap_pyfunction!(enu_to_ned_dict, module)?)?;
    module.add_function(wrap_pyfunction!(ned_to_enu_dict, module)?)?;
    module.add_function(wrap_pyfunction!(convert_wgs_to_enu, module)?)?;
    module.add_class::<PyTransformTree>()?;
    Ok(())
}

#[pymodule]
fn concord(module: &Bound<'_, PyModule>) -> PyResult<()> {
    register_python_module(module)
}
