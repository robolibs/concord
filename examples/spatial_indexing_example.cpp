#include <concord/concord.hpp>
#include <concord/indexing/spatial_index.hpp>
#include <iostream>
#include <vector>
#include <random>
#include <chrono>

using namespace concord;
using namespace concord::indexing;

// Example data structure to store in spatial indices
struct POI {
    std::string name;
    std::string category;
    
    POI(const std::string& n, const std::string& c) : name(n), category(c) {}
    
    bool operator==(const POI& other) const {
        return name == other.name && category == other.category;
    }
};

int main() {
    std::cout << "=== Concord Spatial Indexing Example ===" << std::endl;

    // Define our spatial bounds
    AABB world_bounds(Point(-1000, -1000, 0), Point(1000, 1000, 0));
    
    // Create different types of spatial indices
    auto point_index = indexing::spatial_indexing::createPointIndex<POI>(world_bounds);
    auto fast_index = indexing::spatial_indexing::createFastPointIndex<POI>(50.0); // 50 unit cell size
    auto geometry_index = indexing::spatial_indexing::createGeometryIndex<POI>();
    
    std::cout << "Created spatial indices" << std::endl;

    // Generate random POI data
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> pos_dist(-500, 500);
    std::uniform_real_distribution<> size_dist(1.0, 20.0);
    
    std::vector<std::string> categories = {"restaurant", "hotel", "shop", "park", "museum"};
    std::vector<Point> poi_locations;
    std::vector<AABB> poi_bounds;
    
    const size_t num_pois = 1000;
    
    std::cout << "Generating " << num_pois << " POIs..." << std::endl;
    
    // Insert POIs into all three indices
    for (size_t i = 0; i < num_pois; ++i) {
        Point location(pos_dist(gen), pos_dist(gen), 0);
        poi_locations.push_back(location);
        
        std::string name = "POI_" + std::to_string(i);
        std::string category = categories[i % categories.size()];
        POI poi(name, category);
        
        // For point-based indices
        point_index.insert(location, poi);
        fast_index.insert(location, poi);
        
        // For geometry index, create a small bounding box around the point
        double size = size_dist(gen);
        AABB bounds(Point(location.x - size/2, location.y - size/2, 0),
                   Point(location.x + size/2, location.y + size/2, 0));
        poi_bounds.push_back(bounds);
        geometry_index.insert(bounds, poi);
    }
    
    std::cout << "Inserted POIs into indices" << std::endl;
    std::cout << "Point index size: " << point_index.size() << std::endl;
    std::cout << "Fast index size: " << fast_index.size() << std::endl; 
    std::cout << "Geometry index size: " << geometry_index.size() << std::endl;

    // Performance comparison: Radius queries
    Point query_center(0, 0, 0);
    double query_radius = 100.0;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // QuadTree radius query
    auto quad_results = point_index.queryRadius(query_center, query_radius);
    auto quad_time = std::chrono::high_resolution_clock::now();
    
    // SpatialHashGrid radius query  
    auto hash_results = fast_index.queryRadius(query_center, query_radius);
    auto hash_time = std::chrono::high_resolution_clock::now();
    
    // RTree radius query
    auto rtree_results = geometry_index.queryRadius(query_center, query_radius);
    auto rtree_time = std::chrono::high_resolution_clock::now();
    
    std::cout << "\\n=== Radius Query Results (radius=" << query_radius << ") ===" << std::endl;
    std::cout << "QuadTree: " << quad_results.size() << " results in " 
              << std::chrono::duration_cast<std::chrono::microseconds>(quad_time - start_time).count() 
              << " μs" << std::endl;
    std::cout << "HashGrid: " << hash_results.size() << " results in "
              << std::chrono::duration_cast<std::chrono::microseconds>(hash_time - quad_time).count() 
              << " μs" << std::endl;
    std::cout << "RTree: " << rtree_results.size() << " results in "
              << std::chrono::duration_cast<std::chrono::microseconds>(rtree_time - hash_time).count() 
              << " μs" << std::endl;

    // Rectangle query example
    AABB query_rect(Point(-50, -50, 0), Point(50, 50, 0));
    auto rect_results = point_index.queryRect(query_rect);
    std::cout << "\\nRectangle query found " << rect_results.size() << " POIs" << std::endl;

    // K-nearest neighbors example
    size_t k = 5;
    auto knn_results = point_index.kNearestNeighbors(query_center, k);
    std::cout << "\\n" << k << " nearest neighbors to origin:" << std::endl;
    for (const auto& entry : knn_results) {
        double distance = entry.point.distance_to(query_center);
        std::cout << "  " << entry.data.name << " (" << entry.data.category 
                  << ") at distance " << distance << std::endl;
    }

    // Polygon query example
    Polygon triangle;
    triangle.addPoint(Point(0, 100, 0));
    triangle.addPoint(Point(-50, -50, 0));
    triangle.addPoint(Point(50, -50, 0));
    
    auto polygon_results = point_index.queryPolygon(triangle);
    std::cout << "\\nPolygon query found " << polygon_results.size() << " POIs inside triangle" << std::endl;

    // Demonstrate removal
    if (!quad_results.empty()) {
        const auto& to_remove = quad_results[0];
        bool removed = point_index.remove(to_remove.point, to_remove.data);
        std::cout << "\\nRemoved POI: " << (removed ? "success" : "failed") << std::endl;
        std::cout << "Point index size after removal: " << point_index.size() << std::endl;
    }

    // Usage recommendations
    std::cout << "\\n=== Usage Recommendations ===" << std::endl;
    
    // Recommend cell size for hash grid
    double recommended_cell_size = indexing::spatial_indexing::utils::recommendCellSize(poi_locations, 10.0);
    std::cout << "Recommended cell size for this dataset: " << recommended_cell_size << std::endl;
    
    std::cout << "\\nChoose your spatial index based on use case:" << std::endl;
    std::cout << "- PointIndex (QuadTree): Best for exact spatial queries, k-NN, balanced performance" << std::endl;
    std::cout << "- FastPointIndex (HashGrid): Fastest for high-throughput point queries, approximate results" << std::endl;
    std::cout << "- GeometryIndex (RTree): Best for complex geometries, objects with spatial extent" << std::endl;
    
    return 0;
}