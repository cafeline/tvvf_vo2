#include "tvvf_vo_c/core/types.hpp"
#include <cmath>
#include <algorithm>

namespace tvvf_vo_c {

// Position implementations
double Position::distance_to(const Position& other) const {
    double dx = x - other.x;
    double dy = y - other.y;
    return std::sqrt(dx * dx + dy * dy);
}

double Position::squared_distance_to(const Position& other) const {
    double dx = x - other.x;
    double dy = y - other.y;
    return dx * dx + dy * dy;
}

Position Position::operator+(const Position& other) const {
    return Position(x + other.x, y + other.y);
}

Position Position::operator-(const Position& other) const {
    return Position(x - other.x, y - other.y);
}

Position Position::operator*(double scalar) const {
    return Position(x * scalar, y * scalar);
}

bool Position::operator==(const Position& other) const {
    const double epsilon = 1e-9;
    return std::abs(x - other.x) < epsilon && std::abs(y - other.y) < epsilon;
}

// Velocity implementations
double Velocity::magnitude() const {
    return std::sqrt(vx * vx + vy * vy);
}

Velocity Velocity::normalize() const {
    double mag = magnitude();
    if (mag < 1e-8) {
        return Velocity(0.0, 0.0);
    }
    return Velocity(vx / mag, vy / mag);
}

Velocity Velocity::operator+(const Velocity& other) const {
    return Velocity(vx + other.vx, vy + other.vy);
}

Velocity Velocity::operator-(const Velocity& other) const {
    return Velocity(vx - other.vx, vy - other.vy);
}

Velocity Velocity::operator*(double scalar) const {
    return Velocity(vx * scalar, vy * scalar);
}

// DynamicObstacle implementations
Position DynamicObstacle::predict_position(double time_delta) const {
    double predicted_x = position.x + velocity.vx * time_delta;
    double predicted_y = position.y + velocity.vy * time_delta;
    return Position(predicted_x, predicted_y);
}


} // namespace tvvf_vo_c
