#pragma once

// All spatial indexing structures
#include "hash_grid/spatial_hash_grid.hpp"
#include "quadtree/quadtree.hpp"
#include "rtree/rtree.hpp"

// Unified spatial indexing interface
#include "spatial_index.hpp"

namespace concord {
    namespace indexing {
        // All indexing structures are available through their individual headers
        // Use spatial_index.hpp for a unified interface across all spatial indices
    } // namespace indexing
} // namespace concord
