// src/core/intersection/Intersection.hpp
//
// Etape 4 + Wave 3 : SFML purge + composition d'une IIntersectionPolicy.
#pragma once

#include <memory>
#include <vector>

#include "core/intersection/IntersectionTypes.hpp"
#include "core/math/TileCoord.hpp"
#include "core/math/Vec2.hpp"

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

    // Horloge locale (s) : avance a chaque update(dt). Utilisee par la policy
    // AIM pour raisonner en fenetres de reservation temporelles absolues.
    float clock_ = 0.f;

    // STOP : axe PRINCIPAL (prioritaire). true = horizontal (E-O) prioritaire,
    // l'axe vertical (N-S) porte le panneau STOP et cede. C'est un STOP 2 voies,
    // pas un all-way : la route principale ne s'arrete jamais.
    bool stopMajorHorizontal_ = true;

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
    void clearApproaches();
    void update(float dt);

    // STOP 2 voies : axe prioritaire. true = horizontal (E-O) prioritaire,
    // l'axe vertical (N-S) porte le panneau STOP. La route principale ne
    // s'arrete jamais ; seule la branche secondaire cede.
    bool isStopMajorAxisHorizontal() const { return stopMajorHorizontal_; }
    void setStopMajorAxisHorizontal(bool horizontal) { stopMajorHorizontal_ = horizontal; }

    // Wave 5 : seule API publique de regulation. Decision riche pour pilotage IDM.
    core::intersection::Decision request(const core::intersection::PolicyContext& ctx) const;

    // Change la strategie de regulation a la volee (recree la policy associee).
    // Permet de comparer les modes sur la MEME geometrie sans rebatir la scene.
    void setRegulation(RegulationType newType);

    // Temps local ecoule (s). Avance avec update(dt).
    float                               now() const { return clock_; }

    int                                 getId() const;
    RegulationType                      getType() const;
    const std::vector<core::TileCoord>& getCoveredTiles() const;
    const std::vector<Approach>&        getApproaches() const;
    bool                                coversTile(int gridX, int gridY) const;
    LightState                          getLightState(Approach::Direction dir) const;

    // --- Geometrie (source unique : renderer / policies / lane partagent) ---
    // Centre du barycentre des tiles couvertes (coords monde).
    core::Vec2 getWorldCenter(float tileSize) const;
    // Rayon exterieur derive de la bounding box (max(largeur, hauteur) / 2).
    float      getOuterRadius(float tileSize) const;
    // Rayon de la trajectoire circulaire (milieu de l'anneau roulable).
    float      getLaneRadius(float tileSize) const;
};
