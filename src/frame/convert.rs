use crate::{
    core::{Error, Result},
    earth::{
        Ecf, Geo, Utm, Wgs, lat_rad, local_axes::r_enu_from_ecf, lon_rad, to_ecf, to_utm, to_wgs,
        utm_to_wgs,
    },
    frame::{Enu, Ned, enu_to_ned, ned_to_enu},
    math::{mat3_mul_vec, mat3_transpose},
};

pub fn to_enu(origin: Geo, wgs: Wgs) -> Enu {
    let point_ecf = to_ecf(wgs);
    let origin_ecf = to_ecf(origin);
    let delta = point_ecf - origin_ecf;
    let enu = mat3_mul_vec(r_enu_from_ecf(lat_rad(origin), lon_rad(origin)), delta);
    Enu::new(enu.x, enu.y, enu.z, origin)
}

pub fn to_ned(origin: Geo, wgs: Wgs) -> Ned {
    let enu = to_enu(origin, wgs);
    Ned::new(enu.north(), enu.east(), -enu.up(), origin)
}

pub fn to_wgs_from_enu(enu: Enu) -> Wgs {
    let origin = enu.origin;
    let rotation = r_enu_from_ecf(lat_rad(origin), lon_rad(origin));
    let delta_ecf = mat3_mul_vec(mat3_transpose(rotation), enu.local);
    let origin_ecf = to_ecf(origin);
    to_wgs(origin_ecf + delta_ecf)
}

pub fn to_wgs_from_ned(ned: Ned) -> Wgs {
    to_wgs_from_enu(Enu::new(ned.east(), ned.north(), -ned.down(), ned.origin))
}

pub trait ConvertInto<Target> {
    fn convert_into(self, reference: Option<Geo>) -> Result<Target>;
}

impl ConvertInto<Ecf> for Wgs {
    fn convert_into(self, _reference: Option<Geo>) -> Result<Ecf> {
        Ok(to_ecf(self))
    }
}

impl ConvertInto<Wgs> for Ecf {
    fn convert_into(self, _reference: Option<Geo>) -> Result<Wgs> {
        Ok(to_wgs(self))
    }
}

impl ConvertInto<Utm> for Wgs {
    fn convert_into(self, _reference: Option<Geo>) -> Result<Utm> {
        to_utm(self)
    }
}

impl ConvertInto<Wgs> for Utm {
    fn convert_into(self, _reference: Option<Geo>) -> Result<Wgs> {
        utm_to_wgs(self)
    }
}

impl ConvertInto<Enu> for Wgs {
    fn convert_into(self, reference: Option<Geo>) -> Result<Enu> {
        match reference {
            Some(origin) => Ok(to_enu(origin, self)),
            None => Err(Error::InvalidArgument(
                "Wgs -> Enu requires reference origin".into(),
            )),
        }
    }
}

impl ConvertInto<Ned> for Wgs {
    fn convert_into(self, reference: Option<Geo>) -> Result<Ned> {
        match reference {
            Some(origin) => Ok(to_ned(origin, self)),
            None => Err(Error::InvalidArgument(
                "Wgs -> Ned requires reference origin".into(),
            )),
        }
    }
}

impl ConvertInto<Wgs> for Enu {
    fn convert_into(self, _reference: Option<Geo>) -> Result<Wgs> {
        Ok(to_wgs_from_enu(self))
    }
}

impl ConvertInto<Wgs> for Ned {
    fn convert_into(self, _reference: Option<Geo>) -> Result<Wgs> {
        Ok(to_wgs_from_ned(self))
    }
}

impl ConvertInto<Ned> for Enu {
    fn convert_into(self, _reference: Option<Geo>) -> Result<Ned> {
        Ok(enu_to_ned(self))
    }
}

impl ConvertInto<Enu> for Ned {
    fn convert_into(self, _reference: Option<Geo>) -> Result<Enu> {
        Ok(ned_to_enu(self))
    }
}

pub struct ConvertBuilder<T> {
    state: Result<T>,
    reference: Option<Geo>,
}

impl<T> ConvertBuilder<T> {
    pub fn with_ref(mut self, reference: Geo) -> Self {
        self.reference = Some(reference);
        self
    }

    pub fn with_datum(self, reference: Geo) -> Self {
        self.with_ref(reference)
    }

    pub fn to<Target>(self) -> ConvertBuilder<Target>
    where
        T: ConvertInto<Target>,
    {
        let reference = self.reference;
        let state = match self.state {
            Ok(value) => value.convert_into(reference),
            Err(error) => Err(error),
        };

        ConvertBuilder { state, reference }
    }

    pub fn build(self) -> Result<T> {
        self.state
    }
}

pub fn convert<T>(value: T) -> ConvertBuilder<T> {
    ConvertBuilder {
        state: Ok(value),
        reference: None,
    }
}
