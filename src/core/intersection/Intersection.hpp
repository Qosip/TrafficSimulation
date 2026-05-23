// src/core/intersection/Intersection.hpp
//
// Etape 4 + Wave 3 : SFML purge + composition d'une IIntersectionPolicy.
#pragma once

#include <memory>
#include <vector>

#include "core/intersection/IntersectionTypes.hpp"
#include "core/math/TileCoord.hpp"

class IAgent;

namespace core::intersection {
struct PolicyContext;
struct Decision;
class  IIntersectionPolicy;
} // namespace core::intersection

class Intersection {
private:
    int            id;
    RegulationType type;

    std::vector<core::TileCoord> coveredTiles;
    std::vector<Approach>        approaches;

    // Feux tricolores
    float lightTimer     = 0.f;
    int   currentPhase   = 0;
    float greenDuration  = 5.f;
    float orangeDuration = 1.5f;

    void updateTrafficLight(float dt);

    // Wave 3 : la regulation effective vit dans la policy injectee.
    std::unique_ptr<core::intersection::IIntersectionPolicy> policy_;

public:
    Intersection(int id, RegulationType type);
    Intersection(Intersection&&) noexcept;             // unique_ptr non copiable -> custom moves
    Intersection& operator=(Intersection&&) noexcept;
    Intersection(const Intersection&);                 // copie : recree une policy
    Intersection& operator=(const Intersection&);
    ~Intersection();

    void addCoveredTile(core::TileCoord tile);
    void addApproach(const Approach& approach);
    void update(float dt);

    // Wave 5 : seule API publique de regulation. Decision riche pour pilotage IDM.
    core::intersection::Decision request(const core::intersection::PolicyContext& ctx) const;

    int                                 getId() const;
    RegulationType                      getType() const;
    const std::vector<core::TileCoord>& getCoveredTiles() const;
    const std::vector<Approach>&        getApproaches() const;
    bool                                coversTile(int gridX, int gridY) const;
    LightState                          getLightState(Approach::Direction dir) const;
};
