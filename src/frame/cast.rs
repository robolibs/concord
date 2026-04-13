use crate::frame::{Enu, Flu, Frd, Ned};

pub fn enu_to_ned(enu: Enu) -> Ned {
    Ned::new(enu.north(), enu.east(), -enu.up(), enu.origin)
}

pub fn ned_to_enu(ned: Ned) -> Enu {
    Enu::new(ned.east(), ned.north(), -ned.down(), ned.origin)
}

pub fn frd_to_flu(frd: Frd) -> Flu {
    Flu::new(frd.forward(), -frd.right(), -frd.down())
}

pub fn flu_to_frd(flu: Flu) -> Frd {
    Frd::new(flu.forward(), -flu.left(), -flu.up())
}

pub trait FrameCast<To> {
    fn frame_cast(self) -> To;
}

impl FrameCast<Enu> for Enu {
    fn frame_cast(self) -> Enu {
        self
    }
}

impl FrameCast<Ned> for Ned {
    fn frame_cast(self) -> Ned {
        self
    }
}

impl FrameCast<Frd> for Frd {
    fn frame_cast(self) -> Frd {
        self
    }
}

impl FrameCast<Flu> for Flu {
    fn frame_cast(self) -> Flu {
        self
    }
}

impl FrameCast<Ned> for Enu {
    fn frame_cast(self) -> Ned {
        enu_to_ned(self)
    }
}

impl FrameCast<Enu> for Ned {
    fn frame_cast(self) -> Enu {
        ned_to_enu(self)
    }
}

impl FrameCast<Flu> for Frd {
    fn frame_cast(self) -> Flu {
        frd_to_flu(self)
    }
}

impl FrameCast<Frd> for Flu {
    fn frame_cast(self) -> Frd {
        flu_to_frd(self)
    }
}

pub fn frame_cast<To, From>(from: From) -> To
where
    From: FrameCast<To>,
{
    from.frame_cast()
}
