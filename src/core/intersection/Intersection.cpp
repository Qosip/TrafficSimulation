// src/core/intersection/Intersection.cpp
#include "core/intersection/Intersection.hpp"

#include "core/intersection/IIntersectionPolicy.hpp"
#include "core/intersection/PriorityRightPolicy.hpp"
#include "core/intersection/TrafficLightPolicy.hpp"

namespace {

std::unique_ptr<core::intersection::IIntersectionPolicy>
makePolicyFor(RegulationType type) {
    using namespace core::intersection;
    switch (type) {
        case RegulationType::TRAFFIC_LIGHT:  return std::make_unique<TrafficLightPolicy>();
        case RegulationType::PRIORITY_RIGHT: return std::make_unique<PriorityRightPolicy>();
        default: return std::make_unique<PriorityRightPolicy>();  // fallback raisonnable
    }
}

} // namespace

Intersection::Intersection(int id_, RegulationType type_)
    : id(id_), type(type_), policy_(makePolicyFor(type_))
{}

Intersection::Intersection(Intersection&&) noexcept            = default;
Intersection& Intersection::operator=(Intersection&&) noexcept = default;

Intersection::Intersection(const Intersection& o)
    : id(o.id), type(o.type),
      coveredTiles(o.coveredTiles), approaches(o.approaches),
      lightTimer(o.lightTimer), currentPhase(o.currentPhase),
      greenDuration(o.greenDuration), orangeDuration(o.orangeDuration),
      policy_(makePolicyFor(o.type))
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
    policy_        = makePolicyFor(o.type);
    return *this;
}

Intersection::~Intersection() = default;

void Intersection::addCoveredTile(core::TileCoord tile) { coveredTiles.push_back(tile); }
void Intersection::addApproach(const Approach& approach) { approaches.push_back(approach); }

void Intersection::update(float dt) {
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
    if (policy_) return policy_->request(ctx, *this);
    core::intersection::Decision d;
    d.canEnter = true;
    return d;
}

bool Intersection::coversTile(int gridX, int gridY) const {
    for (const auto& tile : coveredTiles) {
        if (tile.x == gridX && tile.y == gridY) return true;
    }
    return false;
}

int                                  Intersection::getId()           const { return id; }
RegulationType                       Intersection::getType()         const { return type; }
const std::vector<core::TileCoord>&  Intersection::getCoveredTiles() const { return coveredTiles; }
const std::vector<Approach>&         Intersection::getApproaches()   const { return approaches; }
