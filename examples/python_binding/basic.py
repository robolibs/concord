import concord


origin = (52.0, 4.0, 10.0)
point = (52.0001, 4.0002, 12.0)

print("wgs_to_ecf:", concord.wgs_to_ecf(point))
print("wgs_to_utm:", concord.wgs_to_utm(point))
print("wgs_to_enu:", concord.wgs_to_enu(origin, point))
print("wgs_to_ned:", concord.wgs_to_ned(origin, point))

tree = concord.TransformTree()
tree.register_frame("world")
tree.register_frame("base")
tree.register_frame("camera")

tree.set_transform("world", "base", (0.0, 0.0, 0.0, 1.0), (1.0, 2.0, 0.0))
tree.set_transform("base", "camera", (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 1.0))

print("frame_names:", tree.frame_names())
print("can_transform:", tree.can_transform("world", "camera"))
print("lookup:", tree.lookup("world", "camera"))
