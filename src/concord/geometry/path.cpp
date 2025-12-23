#include "concord/geometry/path.hpp"

namespace concord {

    Path::Path(const std::vector<Pose> &poses) : poses_(poses) {}

    void Path::add(const Pose &p) { poses_.emplace_back(p); }

    void Path::clear() noexcept { poses_.clear(); }

    std::size_t Path::size() const noexcept { return poses_.size(); }

    bool Path::empty() const noexcept { return poses_.empty(); }

    Pose &Path::operator[](std::size_t idx) { return poses_.at(idx); }

    const Pose &Path::operator[](std::size_t idx) const { return poses_.at(idx); }

    std::vector<Pose>::iterator Path::begin() noexcept { return poses_.begin(); }

    std::vector<Pose>::iterator Path::end() noexcept { return poses_.end(); }

    std::vector<Pose>::const_iterator Path::begin() const noexcept { return poses_.begin(); }

    std::vector<Pose>::const_iterator Path::end() const noexcept { return poses_.end(); }

    const std::vector<Pose> &Path::get() const noexcept { return poses_; }

} // namespace concord
