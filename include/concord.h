#ifndef CONCORD_H
#define CONCORD_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define A_M 6378137.0

#define F (1.0 / 298.257223563)

#define B_M (A_M * (1.0 - F))

#define E2 (F * (2.0 - F))

#define EP2 (E2 / (1.0 - E2))

#define E4 (E2 * E2)

#define E6 (E4 * E2)

#define K0 0.9996

typedef struct ConcordTransformTree ConcordTransformTree;

typedef struct FrameTag FrameTag;

typedef struct {
  double x;
  double y;
  double z;
} ConcordEcf3;

typedef struct {
  double latitude;
  double longitude;
  double altitude;
} ConcordGeo3;

typedef struct {
  int32_t zone;
  uint32_t band;
  double easting;
  double northing;
  double altitude;
} ConcordUtm;

typedef struct {
  double east;
  double north;
  double up;
  ConcordGeo3 origin;
} ConcordEnuPoint;

typedef struct {
  double north;
  double east;
  double down;
  ConcordGeo3 origin;
} ConcordNedPoint;

typedef struct {
  double x;
  double y;
  double z;
  double w;
} ConcordQuat;

typedef struct {
  double x;
  double y;
  double z;
} ConcordVec3;

typedef struct {
  ConcordQuat rotation;
  ConcordVec3 translation;
} ConcordTransform;









#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

const char *concord_last_error_message(void);

ConcordEcf3 concord_wgs_to_ecf(ConcordGeo3 wgs);

ConcordGeo3 concord_ecf_to_wgs(ConcordEcf3 ecf);

ConcordGeo3 concord_ecf_to_wgs_optimized(ConcordEcf3 ecf, double tolerance);

bool concord_wgs_to_utm(ConcordGeo3 wgs, ConcordUtm *out_utm);

bool concord_utm_to_wgs(ConcordUtm utm, ConcordGeo3 *out_wgs);

ConcordEnuPoint concord_wgs_to_enu(ConcordGeo3 origin, ConcordGeo3 wgs);

ConcordNedPoint concord_wgs_to_ned(ConcordGeo3 origin, ConcordGeo3 wgs);

ConcordGeo3 concord_enu_to_wgs(ConcordEnuPoint enu);

ConcordGeo3 concord_ned_to_wgs(ConcordNedPoint ned);

ConcordNedPoint concord_enu_to_ned_point(ConcordEnuPoint enu);

ConcordEnuPoint concord_ned_to_enu_point(ConcordNedPoint ned);

bool concord_convert_wgs_to_enu(ConcordGeo3 wgs, ConcordGeo3 origin, ConcordEnuPoint *out_enu);

ConcordTransformTree *concord_transform_tree_new(void);

void concord_transform_tree_free(ConcordTransformTree *tree);

bool concord_transform_tree_register_frame(ConcordTransformTree *tree, const char *name);

bool concord_transform_tree_set_transform(ConcordTransformTree *tree,
                                          const char *to_frame,
                                          const char *from_frame,
                                          ConcordTransform transform);

bool concord_transform_tree_lookup(ConcordTransformTree *tree,
                                   const char *to_frame,
                                   const char *from_frame,
                                   ConcordTransform *out_transform);

bool concord_transform_tree_can_transform(ConcordTransformTree *tree,
                                          const char *to_frame,
                                          const char *from_frame);

uintptr_t concord_transform_tree_frame_count(const ConcordTransformTree *tree);

uintptr_t concord_transform_tree_transform_count(const ConcordTransformTree *tree);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  /* CONCORD_H */
