#include "concord/types/euler.hpp"
#include "concord/types/quaternion.hpp"
#include <cmath>

namespace concord {

    Euler::Euler(double roll_, double pitch_, double yaw_) : roll(roll_), pitch(pitch_), yaw(yaw_) {}

    Euler::Euler(const Quaternion &q) noexcept {
        // roll (x-axis rotation)
        double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2.0 * (q.w * q.y - q.z * q.x);
        if (std::abs(sinp) >= 1.0) {
            pitch = std::copysign(M_PI / 2.0, sinp);
        } else {
            pitch = std::asin(sinp);
        }

        // yaw (z-axis rotation)
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    bool Euler::is_set() const { return roll != 0.0 || pitch != 0.0 || yaw != 0.0; }

    double Euler::yaw_cos() const { return std::cos(yaw * 0.5); }

    double Euler::yaw_sin() const { return std::sin(yaw * 0.5); }

    Euler Euler::operator+(const Euler &other) const {
        return Euler{roll + other.roll, pitch + other.pitch, yaw + other.yaw};
    }

    Euler Euler::operator-(const Euler &other) const {
        return Euler{roll - other.roll, pitch - other.pitch, yaw - other.yaw};
    }

    Euler Euler::operator*(double scale) const { return Euler{roll * scale, pitch * scale, yaw * scale}; }

    bool Euler::operator==(const Euler &other) const {
        return roll == other.roll && pitch == other.pitch && yaw == other.yaw;
    }

    bool Euler::operator!=(const Euler &other) const { return !(*this == other); }

    Euler Euler::normalized() const {
        auto normalize_angle = [](double angle) {
            while (angle > M_PI)
                angle -= 2.0 * M_PI;
            while (angle < -M_PI)
                angle += 2.0 * M_PI;
            return angle;
        };
        return Euler{normalize_angle(roll), normalize_angle(pitch), normalize_angle(yaw)};
    }

    Mat3d Euler::to_rotation_matrix() const {
        double cr = std::cos(roll), sr = std::sin(roll);
        double cp = std::cos(pitch), sp = std::sin(pitch);
        double cy = std::cos(yaw), sy = std::sin(yaw);

        Mat3d R;
        R[0][0] = cy * cp;
        R[0][1] = cy * sp * sr - sy * cr;
        R[0][2] = cy * sp * cr + sy * sr;
        R[1][0] = sy * cp;
        R[1][1] = sy * sp * sr + cy * cr;
        R[1][2] = sy * sp * cr - cy * sr;
        R[2][0] = -sp;
        R[2][1] = cp * sr;
        R[2][2] = cp * cr;
        return R;
    }

    Vec3d Euler::rotate(const Vec3d &v) const {
        Mat3d R = to_rotation_matrix();
        return R * v;
    }

} // namespace concord
