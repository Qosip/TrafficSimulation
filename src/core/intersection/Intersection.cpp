// src/core/intersection/Intersection.cpp
#include "core/intersection/Intersection.hpp"

#include <algorithm>
#include <cmath>

#include "core/intersection/AimPolicy.hpp"
#include "core/intersection/FixedPriorityPolicy.hpp"
#include "core/intersection/IIntersectionPolicy.hpp"
#include "core/intersection/OrcaPolicy.hpp"
#include "core/intersection/P2PPolicy.hpp"
#include "core/intersection/PlatooningPolicy.hpp"
#include "core/intersection/PriorityRightPolicy.hpp"
#include "core/intersection/RoundaboutPolicy.hpp"
#include "core/intersection/StopPolicy.hpp"
#include "core/intersection/TrafficLightPolicy.hpp"

namespace {

std::unique_ptr<core::intersection::IIntersectionPolicy>
makePolicyFor(RegulationType type) {
    using namespace core::intersection;
    switch (type) {
        case RegulationType::TRAFFIC_LIGHT:  return std::make_unique<TrafficLightPolicy>();
        case RegulationType::PRIORITY_RIGHT: return std::make_unique<PriorityRightPolicy>();
        case RegulationType::STOP:           return std::make_unique<StopPolicy>();
        case RegulationType::ROUNDABOUT:     return std::make_unique<RoundaboutPolicy>();
        case RegulationType::YIELD:          return std::make_unique<PriorityRightPolicy>(); // approximation
        case RegulationType::FIXED_PRIORITY: return std::make_unique<FixedPriorityPolicy>();
        case RegulationType::P2P:            return std::make_unique<P2PPolicy>();
        case RegulationType::AIM:            return std::make_unique<AimPolicy>();
        case RegulationType::VIRTUAL_PLATOON:return std::make_unique<PlatooningPolicy>();
        case RegulationType::ORCA:           return std::make_unique<OrcaPolicy>();
        default:                             return std::make_unique<PriorityRightPolicy>();
    }
}

} // namespace

Intersection::Intersection(int id_, RegulationType type_)
    : id(id_), type(type_), policy_(makePolicyFor(type_)),
      reqMutex_(std::make_unique<std::mutex>())
{}

Intersection::Intersection(Intersection&&) noexcept            = default;
Intersection& Intersection::operator=(Intersection&&) noexcept = default;

Intersection::Intersection(const Intersection& o)
    : id(o.id), type(o.type),
      coveredTiles(o.coveredTiles), approaches(o.approaches),
      lightTimer(o.lightTimer), currentPhase(o.currentPhase),
      greenDuration(o.greenDuration), orangeDuration(o.orangeDuration),
      stopMajorHorizontal_(o.stopMajorHorizontal_),
      policy_(makePolicyFor(o.type)),
      reqMutex_(std::make_unique<std::mutex>())   // mutex propre (non copiable)
{}

Intersection& Intersection::operator=(const Intersection& o) {
    if (this == &o) return *this;
    id             = o.id;
    type           = o.type;
    coveredTiles   = o.coveredTiles;
    approaches     = o.approaches;
    lightTimer     = o.lightTimer;
    currentPhase   = o.currentPhase;
    greenDuration  = o.greenDuration;
    orangeDuration = o.orangeDuration;
    stopMajorHorizontal_ = o.stopMajorHorizontal_;
    policy_        = makePolicyFor(o.type);
    if (!reqMutex_) reqMutex_ = std::make_unique<std::mutex>();
    return *this;
}

Intersection::~Intersection() = default;

void Intersection::addCoveredTile(core::TileCoord tile) { coveredTiles.push_back(tile); }
void Intersection::addApproach(const Approach& approach) { approaches.push_back(approach); }
void Intersection::clearApproaches() { approaches.clear(); }

void Intersection::update(float dt) {
    clock_ += dt;
    if (type == RegulationType::TRAFFIC_LIGHT) updateTrafficLight(dt);
}

void Intersection::updateTrafficLight(float dt) {
    lightTimer += dt;
    const float phaseDuration = (currentPhase == 0 || currentPhase == 2) ? greenDuration : orangeDuration;

    if (lightTimer >= phaseDuration) {
        lightTimer = 0.f;
        currentPhase = (currentPhase + 1) % 4;

        for (auto& app : approaches) {
            const bool isNS = (app.direction == Approach::Direction::NORTH ||
                               app.direction == Approach::Direction::SOUTH);

            if      (currentPhase == 0) app.hasGreen = isNS;
            else if (currentPhase == 2) app.hasGreen = !isNS;
            else                        app.hasGreen = false;
        }
    }
}

LightState Intersection::getLightState(Approach::Direction dir) const {
    if (type != RegulationType::TRAFFIC_LIGHT) return LightState::GREEN;
    const bool isNS = (dir == Approach::Direction::NORTH || dir == Approach::Direction::SOUTH);

    if (currentPhase == 0) return isNS ? LightState::GREEN  : LightState::RED;
    if (currentPhase == 1) return isNS ? LightState::ORANGE : LightState::RED;
    if (currentPhase == 2) return isNS ? LightState::RED    : LightState::GREEN;
    return isNS ? LightState::RED : LightState::ORANGE;
}

core::intersection::Decision
Intersection::request(const core::intersection::PolicyContext& ctx) const {
    if (policy_) {
        // Serialise les acces concurrents a l'etat mutable de CETTE policy
        // (cout negligeable en mono-thread : mutex non contendu). reqMutex_ ne
        // peut etre nul que sur une instance deplacee (jamais interrogee).
        if (reqMutex_) {
            std::lock_guard<std::mutex> lock(*reqMutex_);
            return policy_->request(ctx, *this);
        }
        return policy_->request(ctx, *this);
    }
    core::intersection::Decision d;
    d.canEnter = true;
    return d;
}

void Intersection::setRegulation(RegulationType newType) {
    if (newType == type) return;
    type   = newType;
    policy_ = makePolicyFor(newType);
    // Repart sur une phase de feu propre (sans effet pour les autres modes).
    lightTimer   = 0.f;
    currentPhase = 0;
}

bool Intersection::coversTile(int gridX, int gridY) const {
    for (const auto& tile : coveredTiles) {
        if (tile.x == gridX && tile.y == gridY) return true;
    }
    return false;
}

bool Intersection::containsWorldPoint(core::Vec2 p, float ts) const {
    if (ts <= 0.f) return false;
    const int gridX = static_cast<int>(std::floor(p.x / ts));
    const int gridY = static_cast<int>(std::floor(p.y / ts));
    return coversTile(gridX, gridY);
}

int                                  Intersection::getId()           const { return id; }
RegulationType                       Intersection::getType()         const { return type; }
const std::vector<core::TileCoord>&  Intersection::getCoveredTiles() const { return coveredTiles; }
const std::vector<Approach>&         Intersection::getApproaches()   const { return approaches; }

core::Vec2 Intersection::getWorldCenter(float ts) const {
    if (coveredTiles.empty()) return {0.f, 0.f};
    core::Vec2 c{0.f, 0.f};
    for (const auto& t : coveredTiles) {
        c.x += t.x * ts + ts / 2.f;
        c.y += t.y * ts + ts / 2.f;
    }
    const float n = static_cast<float>(coveredTiles.size());
    return { c.x / n, c.y / n };
}

float Intersection::getOuterRadius(float ts) const {
    if (coveredTiles.empty()) return ts;
    float minX = 1e9f, minY = 1e9f, maxX = -1e9f, maxY = -1e9f;
    for (const auto& t : coveredTiles) {
        const float cx = t.x * ts + ts / 2.f;
        const float cy = t.y * ts + ts / 2.f;
        minX = std::min(minX, cx); maxX = std::max(maxX, cx);
        minY = std::min(minY, cy); maxY = std::max(maxY, cy);
    }
    const float bw = (maxX - minX) + ts;
    const float bh = (maxY - minY) + ts;
    return std::max(bw, bh) / 2.f;
}

float Intersection::getLaneRadius(float ts) const {
    const float outerR = getOuterRadius(ts);
    // Grand rond-point (> 4 tiles) : ilot central, on roule au milieu de
    // l'anneau [outerR - ts, outerR]. Mini (2x2) : pas d'ilot, cercle ts/2.
    const bool large = coveredTiles.size() > 4;
    return large ? (outerR - ts * 0.5f) : (outerR * 0.5f);
}
