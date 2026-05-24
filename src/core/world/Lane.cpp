#include "core/world/Lane.hpp"

#include <algorithm>
#include <cmath>

#include "core/math/Constants.hpp"

Lane::Lane(const std::vector<core::Vec2>& waypoints) : points(waypoints), totalLength(0.f) {
    accumulatedDistances.push_back(0.f);
    for (std::size_t i = 1; i < points.size(); ++i) {
        const float dist = (points[i] - points[i - 1]).length();
        totalLength += dist;
        accumulatedDistances.push_back(totalLength);
    }
}

core::Vec2 Lane::getPositionAt(float s) const {
    if (points.empty()) return {0.f, 0.f};
    if (s <= 0.f)            return points.front();
    if (s >= totalLength)    return points.back();

    for (std::size_t i = 1; i < accumulatedDistances.size(); ++i) {
        if (s <= accumulatedDistances[i]) {
            const float s0 = accumulatedDistances[i - 1];
            const float s1 = accumulatedDistances[i];
            const float ratio = (s - s0) / (s1 - s0);
            return points[i - 1] + (points[i] - points[i - 1]) * ratio;
        }
    }
    return points.back();
}

float Lane::getHeadingAt(float s) const {
    if (points.size() < 2) return 0.f;

    constexpr float lookAhead = 5.0f;

    const core::Vec2 currentPos = getPositionAt(s);
    core::Vec2 nextPos = getPositionAt(std::min(s + lookAhead, totalLength));
    core::Vec2 diff = nextPos - currentPos;

    // Si on est au tout dernier point, on calcule l'angle en regardant en arriere.
    if (diff.x == 0.f && diff.y == 0.f) {
        const core::Vec2 prevPos = getPositionAt(std::max(0.f, s - lookAhead));
        diff = currentPos - prevPos;
    }
    return std::atan2(diff.y, diff.x) * core::math::RAD2DEG;
}
