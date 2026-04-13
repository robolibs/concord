use concord::{Wgs, to_ecf};

#[test]
fn crate_exposes_basic_wgs_to_ecf_path() {
    let ecf = to_ecf(Wgs::new(0.0, 0.0, 0.0));
    assert!(ecf.x.is_finite());
}
