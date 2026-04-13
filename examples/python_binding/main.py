import concord


origin = (48.8566, 2.3522, 35.0)
point = (48.8570, 2.3530, 40.0)

enu = concord.convert_wgs_to_enu(point, origin)
ned = concord.enu_to_ned_dict((enu["east"], enu["north"], enu["up"]), origin)
roundtrip = concord.ned_to_wgs((ned["north"], ned["east"], ned["down"]), origin)

print("origin:", origin)
print("point:", point)
print("enu:", enu)
print("ned:", ned)
print("roundtrip:", roundtrip)

tree = concord.TransformTree()
for name in ("map", "odom", "base_link", "camera"):
    tree.register_frame(name)

tree.set_transform("map", "odom", (0.0, 0.0, 0.0, 1.0), (10.0, 0.0, 0.0))
tree.set_transform("odom", "base_link", (0.0, 0.0, 0.0, 1.0), (1.0, 2.0, 0.0))
tree.set_transform("base_link", "camera", (0.0, 0.0, 0.0, 1.0), (0.2, 0.0, 1.5))

print("map<-camera:", tree.lookup("map", "camera"))
