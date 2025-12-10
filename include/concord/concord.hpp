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
#include "geometry/polygon/partition.hpp"
#include "geometry/polygon/partitioner.hpp"
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
#include "algorithms/algorithms.hpp"

// Spatial indexing structures
#include "indexing/hash_grid/spatial_hash_grid.hpp"
#include "indexing/indexing.hpp"

namespace concord {
    // Library capabilities
    constexpr bool HAS_MATHEMATICAL_TYPES = true;
    constexpr bool HAS_SPATIAL_INDEXING = true;
    constexpr bool HAS_ADVANCED_ALGORITHMS = true;
    constexpr bool HAS_MULTIPLE_DATUMS = true;
} // namespace concord
