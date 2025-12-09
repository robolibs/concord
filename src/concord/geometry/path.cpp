#include "concord/geometry/path.hpp"

namespace concord {

    Path::Path(const std::vector<Point> &pts) : points(pts) {}

    void Path::addPoint(const Point &p) { 
        points.emplace_back(p); 
    }

    void Path::clear() noexcept { 
        points.clear(); 
    }

    std::size_t Path::size() const noexcept { 
        return points.size(); 
    }

    bool Path::empty() const noexcept { 
        return points.empty(); 
    }

    Point &Path::operator[](std::size_t idx) { 
        return points.at(idx); 
    }

    const Point &Path::operator[](std::size_t idx) const { 
        return points.at(idx); 
    }

    auto Path::begin() noexcept { 
        return points.begin(); 
    }

    auto Path::end() noexcept { 
        return points.end(); 
    }

    auto Path::begin() const noexcept { 
        return points.begin(); 
    }

    auto Path::end() const noexcept { 
        return points.end(); 
    }

    const std::vector<Point> &Path::getPoints() const noexcept { 
        return points; 
    }

} // namespace concord
