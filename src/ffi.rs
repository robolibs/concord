//! C ABI for concord.
//!
//! Conventions: opaque Box-backed handles (free with the matching
//! *_free); fallible calls return bool/int with the reason in the
//! thread-local concord_last_error_message(); borrowed views are valid
//! only for the lifetime documented by the handle they came from.
//!
//! `include/concord.h` is generated from this file by cbindgen.

// extern "C" fns take raw pointers from C and deref them by design.
#![allow(clippy::not_unsafe_ptr_arg_deref)]

use std::cell::RefCell;
use std::ffi::{CStr, CString, c_char};
use std::ptr;

use crate::{
    Ecf, Enu, Geo, Ned, Transform, TransformTree, Utm, Wgs, convert, enu_to_ned, ned_to_enu,
};

thread_local! {
    static LAST_ERROR: RefCell<Option<CString>> = const { RefCell::new(None) };
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConcordGeo3 {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConcordEcf3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConcordUtm {
    pub zone: i32,
    pub band: u32,
    pub easting: f64,
    pub northing: f64,
    pub altitude: f64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConcordEnuPoint {
    pub east: f64,
    pub north: f64,
    pub up: f64,
    pub origin: ConcordGeo3,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConcordNedPoint {
    pub north: f64,
    pub east: f64,
    pub down: f64,
    pub origin: ConcordGeo3,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConcordVec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConcordQuat {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConcordTransform {
    pub rotation: ConcordQuat,
    pub translation: ConcordVec3,
}

pub struct ConcordTransformTree {
    tree: TransformTree,
}

impl From<ConcordGeo3> for Geo {
    fn from(value: ConcordGeo3) -> Self {
        Self::new(value.latitude, value.longitude, value.altitude)
    }
}

impl From<Geo> for ConcordGeo3 {
    fn from(value: Geo) -> Self {
        Self {
            latitude: value.latitude,
            longitude: value.longitude,
            altitude: value.altitude,
        }
    }
}

impl From<Ecf> for ConcordEcf3 {
    fn from(value: Ecf) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

impl From<ConcordEcf3> for Ecf {
    fn from(value: ConcordEcf3) -> Self {
        Self::new(value.x, value.y, value.z)
    }
}

impl From<Utm> for ConcordUtm {
    fn from(value: Utm) -> Self {
        Self {
            zone: value.zone,
            band: value.band,
            easting: value.easting,
            northing: value.northing,
            altitude: value.altitude,
        }
    }
}

impl TryFrom<ConcordUtm> for Utm {
    type Error = crate::Error;

    fn try_from(value: ConcordUtm) -> crate::Result<Self> {
        // Validate the codepoint is a real char; Utm stores it as u32 directly.
        let _ = char::from_u32(value.band)
            .ok_or_else(|| crate::Error::InvalidArgument("invalid UTM band codepoint".into()))?;
        Ok(Self {
            zone: value.zone,
            band: value.band,
            easting: value.easting,
            northing: value.northing,
            altitude: value.altitude,
        })
    }
}

impl From<Enu> for ConcordEnuPoint {
    fn from(value: Enu) -> Self {
        Self {
            east: value.east(),
            north: value.north(),
            up: value.up(),
            origin: value.origin.into(),
        }
    }
}

impl From<ConcordEnuPoint> for Enu {
    fn from(value: ConcordEnuPoint) -> Self {
        Self::new(value.east, value.north, value.up, value.origin.into())
    }
}

impl From<Ned> for ConcordNedPoint {
    fn from(value: Ned) -> Self {
        Self {
            north: value.north(),
            east: value.east(),
            down: value.down(),
            origin: value.origin.into(),
        }
    }
}

impl From<ConcordNedPoint> for Ned {
    fn from(value: ConcordNedPoint) -> Self {
        Self::new(value.north, value.east, value.down, value.origin.into())
    }
}

impl From<datapod::Point> for ConcordVec3 {
    fn from(value: datapod::Point) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

impl From<ConcordVec3> for datapod::Point {
    fn from(value: ConcordVec3) -> Self {
        Self::new(value.x, value.y, value.z)
    }
}

impl From<datapod::Quaternion> for ConcordQuat {
    fn from(value: datapod::Quaternion) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
            w: value.w,
        }
    }
}

impl From<ConcordQuat> for datapod::Quaternion {
    fn from(value: ConcordQuat) -> Self {
        datapod::Quaternion::new(value.w, value.x, value.y, value.z).normalized()
    }
}

fn clear_last_error() {
    LAST_ERROR.with(|slot| {
        *slot.borrow_mut() = None;
    });
}

fn set_last_error(message: impl Into<String>) {
    let message = message.into().replace('\0', " ");
    LAST_ERROR.with(|slot| {
        *slot.borrow_mut() = Some(
            CString::new(message).unwrap_or_else(|_| CString::new("concord ffi error").unwrap()),
        );
    });
}

fn ok() -> bool {
    clear_last_error();
    true
}

fn fail(message: impl Into<String>) -> bool {
    set_last_error(message);
    false
}

fn cstr_to_str<'a>(value: *const c_char, label: &str) -> crate::Result<&'a str> {
    if value.is_null() {
        return Err(crate::Error::InvalidArgument(format!(
            "null {label} pointer"
        )));
    }
    let cstr = unsafe { CStr::from_ptr(value) };
    cstr.to_str()
        .map_err(|_| crate::Error::InvalidArgument(format!("{label} must be valid UTF-8")))
}

fn write_out<T>(out: *mut T, value: T) -> bool {
    if out.is_null() {
        return fail("null output pointer");
    }
    unsafe {
        ptr::write(out, value);
    }
    ok()
}

fn tree_from_ptr_mut<'a>(
    tree: *mut ConcordTransformTree,
) -> crate::Result<&'a mut ConcordTransformTree> {
    if tree.is_null() {
        return Err(crate::Error::InvalidArgument(
            "null transform tree handle".into(),
        ));
    }
    Ok(unsafe { &mut *tree })
}

fn tree_from_ptr<'a>(tree: *const ConcordTransformTree) -> crate::Result<&'a ConcordTransformTree> {
    if tree.is_null() {
        return Err(crate::Error::InvalidArgument(
            "null transform tree handle".into(),
        ));
    }
    Ok(unsafe { &*tree })
}

#[unsafe(no_mangle)]
pub extern "C" fn concord_last_error_message() -> *const c_char {
    LAST_ERROR.with(|slot| {
        slot.borrow()
            .as_ref()
            .map_or(ptr::null(), |message| message.as_ptr())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn concord_wgs_to_ecf(wgs: ConcordGeo3) -> ConcordEcf3 {
    clear_last_error();
    crate::to_ecf(Wgs::from(wgs)).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn concord_ecf_to_wgs(ecf: ConcordEcf3) -> ConcordGeo3 {
    clear_last_error();
    crate::to_wgs(Ecf::from(ecf)).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn concord_ecf_to_wgs_optimized(ecf: ConcordEcf3, tolerance: f64) -> ConcordGeo3 {
    clear_last_error();
    crate::to_wgs_optimized(Ecf::from(ecf), tolerance).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn concord_wgs_to_utm(wgs: ConcordGeo3, out_utm: *mut ConcordUtm) -> bool {
    match crate::to_utm(Wgs::from(wgs)) {
        Ok(utm) => write_out(out_utm, utm.into()),
        Err(err) => fail(err.to_string()),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn concord_utm_to_wgs(utm: ConcordUtm, out_wgs: *mut ConcordGeo3) -> bool {
    match Utm::try_from(utm).and_then(crate::utm_to_wgs) {
        Ok(wgs) => write_out(out_wgs, wgs.into()),
        Err(err) => fail(err.to_string()),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn concord_wgs_to_enu(origin: ConcordGeo3, wgs: ConcordGeo3) -> ConcordEnuPoint {
    clear_last_error();
    crate::to_enu(origin.into(), wgs.into()).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn concord_wgs_to_ned(origin: ConcordGeo3, wgs: ConcordGeo3) -> ConcordNedPoint {
    clear_last_error();
    crate::to_ned(origin.into(), wgs.into()).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn concord_enu_to_wgs(enu: ConcordEnuPoint) -> ConcordGeo3 {
    clear_last_error();
    crate::to_wgs_from_enu(Enu::from(enu)).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn concord_ned_to_wgs(ned: ConcordNedPoint) -> ConcordGeo3 {
    clear_last_error();
    crate::to_wgs_from_ned(Ned::from(ned)).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn concord_enu_to_ned_point(enu: ConcordEnuPoint) -> ConcordNedPoint {
    clear_last_error();
    enu_to_ned(Enu::from(enu)).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn concord_ned_to_enu_point(ned: ConcordNedPoint) -> ConcordEnuPoint {
    clear_last_error();
    ned_to_enu(Ned::from(ned)).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn concord_convert_wgs_to_enu(
    wgs: ConcordGeo3,
    origin: ConcordGeo3,
    out_enu: *mut ConcordEnuPoint,
) -> bool {
    match convert(Wgs::from(wgs))
        .with_ref(origin.into())
        .to::<Enu>()
        .build()
    {
        Ok(enu) => write_out(out_enu, enu.into()),
        Err(err) => fail(err.to_string()),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn concord_transform_tree_new() -> *mut ConcordTransformTree {
    clear_last_error();
    Box::into_raw(Box::new(ConcordTransformTree {
        tree: TransformTree::new(),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn concord_transform_tree_free(tree: *mut ConcordTransformTree) {
    if tree.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(tree));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn concord_transform_tree_register_frame(
    tree: *mut ConcordTransformTree,
    name: *const c_char,
) -> bool {
    let result = (|| -> crate::Result<()> {
        let tree = tree_from_ptr_mut(tree)?;
        let name = cstr_to_str(name, "frame name")?;
        tree.tree.register_frame::<()>(name);
        Ok(())
    })();

    match result {
        Ok(()) => ok(),
        Err(err) => fail(err.to_string()),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn concord_transform_tree_set_transform(
    tree: *mut ConcordTransformTree,
    to_frame: *const c_char,
    from_frame: *const c_char,
    transform: ConcordTransform,
) -> bool {
    let result = (|| -> crate::Result<()> {
        let tree = tree_from_ptr_mut(tree)?;
        let to_frame = cstr_to_str(to_frame, "to_frame")?;
        let from_frame = cstr_to_str(from_frame, "from_frame")?;
        let tf =
            Transform::<(), ()>::from_qt(transform.rotation.into(), transform.translation.into());
        tree.tree.set_transform(to_frame, from_frame, tf);
        if !tree.tree.has_frame(to_frame) {
            return Err(crate::Error::NotFound(format!("unknown frame: {to_frame}")));
        }
        if !tree.tree.has_frame(from_frame) {
            return Err(crate::Error::NotFound(format!(
                "unknown frame: {from_frame}"
            )));
        }
        if !tree.tree.can_transform(to_frame, from_frame) {
            return Err(crate::Error::NotFound(format!(
                "failed to set transform between {to_frame} and {from_frame}"
            )));
        }
        Ok(())
    })();

    match result {
        Ok(()) => ok(),
        Err(err) => fail(err.to_string()),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn concord_transform_tree_lookup(
    tree: *mut ConcordTransformTree,
    to_frame: *const c_char,
    from_frame: *const c_char,
    out_transform: *mut ConcordTransform,
) -> bool {
    let result = (|| -> crate::Result<ConcordTransform> {
        let tree = tree_from_ptr_mut(tree)?;
        let to_frame = cstr_to_str(to_frame, "to_frame")?;
        let from_frame = cstr_to_str(from_frame, "from_frame")?;
        let tf = tree.tree.lookup(to_frame, from_frame).ok_or_else(|| {
            crate::Error::NotFound(format!("no transform path from {from_frame} to {to_frame}"))
        })?;
        Ok(ConcordTransform {
            rotation: tf.rotation.into(),
            translation: tf.translation.into(),
        })
    })();

    match result {
        Ok(tf) => write_out(out_transform, tf),
        Err(err) => fail(err.to_string()),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn concord_transform_tree_can_transform(
    tree: *mut ConcordTransformTree,
    to_frame: *const c_char,
    from_frame: *const c_char,
) -> bool {
    let result = (|| -> crate::Result<bool> {
        let tree = tree_from_ptr_mut(tree)?;
        let to_frame = cstr_to_str(to_frame, "to_frame")?;
        let from_frame = cstr_to_str(from_frame, "from_frame")?;
        Ok(tree.tree.can_transform(to_frame, from_frame))
    })();

    match result {
        Ok(value) => {
            clear_last_error();
            value
        }
        Err(err) => fail(err.to_string()),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn concord_transform_tree_frame_count(tree: *const ConcordTransformTree) -> usize {
    match tree_from_ptr(tree) {
        Ok(tree) => {
            clear_last_error();
            tree.tree.frame_count()
        }
        Err(err) => {
            set_last_error(err.to_string());
            0
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn concord_transform_tree_transform_count(
    tree: *const ConcordTransformTree,
) -> usize {
    match tree_from_ptr(tree) {
        Ok(tree) => {
            clear_last_error();
            tree.tree.transform_count()
        }
        Err(err) => {
            set_last_error(err.to_string());
            0
        }
    }
}
