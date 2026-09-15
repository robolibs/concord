-- concord's directory environment. Loaded when you `cd` here, unloaded when you leave.

oslo.direnv.nix_develop()

oslo.direnv.path_add("./target/debug")
oslo.direnv.path_add("./target/release")

oslo.env.set("TOP_HEAD", oslo.sys.pwd())

-- Shared with every other robolibs checkout so the Wayland/NVIDIA detection lives in one place.
oslo.source("/home/bresilla/data/code/robolibs/.display.sh")

oslo.env.set_alias("_b", "make build")
oslo.env.set_alias("_c", "make compile")
oslo.env.set_alias("_r", "make run")
oslo.env.set_alias("_t", "make test")
