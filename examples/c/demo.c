#include <stdio.h>

#include "../../include/concord.h"

int main(void) {
  ConcordGeo3 origin = {52.0, 4.0, 10.0};
  ConcordGeo3 point = {52.0001, 4.0002, 12.0};

  ConcordEcf3 ecf = concord_wgs_to_ecf(point);
  printf("ecf=(%.3f, %.3f, %.3f)\n", ecf.x, ecf.y, ecf.z);

  ConcordUtm utm;
  if (!concord_wgs_to_utm(point, &utm)) {
    fprintf(stderr, "wgs_to_utm failed: %s\n", concord_last_error_message());
    return 1;
  }
  printf(
      "utm=(zone=%d, band=%c, easting=%.3f, northing=%.3f)\n",
      utm.zone,
      (char)utm.band,
      utm.easting,
      utm.northing);

  ConcordEnuPoint enu = concord_wgs_to_enu(origin, point);
  ConcordNedPoint ned = concord_enu_to_ned_point(enu);
  printf("enu=(%.3f, %.3f, %.3f)\n", enu.east, enu.north, enu.up);
  printf("ned=(%.3f, %.3f, %.3f)\n", ned.north, ned.east, ned.down);

  ConcordTransformTree* tree = concord_transform_tree_new();
  if (tree == NULL) {
    fprintf(stderr, "transform_tree_new failed: %s\n", concord_last_error_message());
    return 1;
  }

  if (!concord_transform_tree_register_frame(tree, "world") ||
      !concord_transform_tree_register_frame(tree, "base") ||
      !concord_transform_tree_register_frame(tree, "camera")) {
    fprintf(stderr, "register_frame failed: %s\n", concord_last_error_message());
    concord_transform_tree_free(tree);
    return 1;
  }

  ConcordTransform world_to_base = {
      .rotation = {0.0, 0.0, 0.0, 1.0},
      .translation = {1.0, 2.0, 0.0},
  };
  ConcordTransform base_to_camera = {
      .rotation = {0.0, 0.0, 0.0, 1.0},
      .translation = {0.0, 0.0, 1.0},
  };

  if (!concord_transform_tree_set_transform(tree, "world", "base", world_to_base) ||
      !concord_transform_tree_set_transform(tree, "base", "camera", base_to_camera)) {
    fprintf(stderr, "set_transform failed: %s\n", concord_last_error_message());
    concord_transform_tree_free(tree);
    return 1;
  }

  ConcordTransform lookup;
  if (!concord_transform_tree_lookup(tree, "world", "camera", &lookup)) {
    fprintf(stderr, "lookup failed: %s\n", concord_last_error_message());
    concord_transform_tree_free(tree);
    return 1;
  }

  printf(
      "world<-camera translation=(%.3f, %.3f, %.3f)\n",
      lookup.translation.x,
      lookup.translation.y,
      lookup.translation.z);
  printf("frame_count=%zu\n", concord_transform_tree_frame_count(tree));
  printf("transform_count=%zu\n", concord_transform_tree_transform_count(tree));

  concord_transform_tree_free(tree);
  return 0;
}
