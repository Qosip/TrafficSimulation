#include "core/world/Lane.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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

LaneProjection Lane::project(core::Vec2 p, float sMin, float sMax) const {
    LaneProjection best;
    if (points.size() < 2) return best;

    float bestDist2 = std::numeric_limits<float>::infinity();

    for (std::size_t i = 1; i < points.size(); ++i) {
        const float s0 = accumulatedDistances[i - 1];
        const float s1 = accumulatedDistances[i];
        if (s1 < sMin || s0 > sMax) continue;          // segment hors fenetre

        const core::Vec2 a   = points[i - 1];
        const core::Vec2 b   = points[i];
        const core::Vec2 seg = b - a;
        const float segLen2  = seg.x * seg.x + seg.y * seg.y;
        if (segLen2 < 1e-6f) continue;

        // Parametre du pied de projection sur le segment, clamp [0,1].
        float t = ((p.x - a.x) * seg.x + (p.y - a.y) * seg.y) / segLen2;
        t = std::clamp(t, 0.f, 1.f);

        const core::Vec2 foot{ a.x + seg.x * t, a.y + seg.y * t };
        const core::Vec2 d = p - foot;
        const float dist2  = d.x * d.x + d.y * d.y;
        if (dist2 >= bestDist2) continue;

        bestDist2     = dist2;
        best.valid    = true;
        best.s        = s0 + t * (s1 - s0);
        // Ecart lateral SIGNE : cross(tangente_unitaire, p - pied).
        const float segLen = std::sqrt(segLen2);
        best.lateral = (seg.x * d.y - seg.y * d.x) / segLen;
    }
    return best;
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
