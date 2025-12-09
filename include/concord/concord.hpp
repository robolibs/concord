#pragma once

// Core functionality
#include "types/bound.hpp"
#include "types/euler.hpp"
#include "types/point.hpp"
#include "types/pose.hpp"
#include "types/quaternion.hpp"
#include "types/size.hpp"

// Basic geometric primitives and shapes
#include "geometry/bounding.hpp"
#include "geometry/grid/grid.hpp"
#include "geometry/layer/layer.hpp"
#include "geometry/path.hpp"
#include "geometry/polygon/polygon.hpp"
#include "geometry/primitives/primitives.hpp"

// Advanced spatial types
#include "geographic/coordinate_utils.hpp"
#include "geographic/crs/crs.hpp"
#include "geographic/projections/projections.hpp"
#include "geographic/transformations/transformations.hpp"
#include "geographic/wgs_to_enu.hpp"
#include "geographic/wgs_to_utm.hpp"

// Spatial algorithms
#include "algorithms/convex_hull/convex_hull.hpp"
#include "algorithms/distance/distance.hpp"
#include "algorithms/intersection/intersection.hpp"
#include "algorithms/spatial_algorithms.hpp"
#include "algorithms/triangulation/triangulation.hpp"

// Spatial indexing structures
#include "geometry/path.hpp"
#include "geometry/primitives/circle.hpp"
#include "geometry/primitives/line.hpp"
#include "geometry/primitives/rectangle.hpp"
#include "geometry/primitives/square.hpp"
#include "indexing/indexing.hpp"
// Polygon and polygon algorithms
#include "geometry/polygon/partition.hpp"
#include "geometry/polygon/polygon.hpp"
// Grid and grid operations
#include "geometry/grid/grid.hpp"

// Advanced spatial types
#include "geometry/bounding.hpp"

// Coordinate system conversions
#include "geographic/coordinate_utils.hpp"
#include "geographic/wgs_to_enu.hpp"
#include "geographic/wgs_to_utm.hpp"

// Spatial algorithms and operations
#include "algorithms/spatial_algorithms.hpp"

// Spatial indexing structures
#include "indexing/hash_grid/spatial_hash_grid.hpp"

namespace concord {
    // Library capabilities
    constexpr bool HAS_MATHEMATICAL_TYPES = true;
    constexpr bool HAS_SPATIAL_INDEXING = true;
    constexpr bool HAS_ADVANCED_ALGORITHMS = true;
    constexpr bool HAS_MULTIPLE_DATUMS = true;
} // namespace concord
