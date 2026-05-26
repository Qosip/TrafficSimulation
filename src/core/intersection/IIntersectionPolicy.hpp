// src/core/intersection/IIntersectionPolicy.hpp
//
// Interface PASSIVE de regulation des intersections.
// L'agent demande son droit de passage en fournissant son contexte cinematique.
// La policy retourne une decision riche (et pas un simple bool) pour permettre
// au moteur de comportement (IDM + Virtual Leader) de modeler un freinage doux,
// un arret precis ou une attente predictible.
//
// Wave 3 du refactor : extraction de la logique geometrique hardcodee
// qui residait jusque-la dans Intersection::canPass.
#pragma once

#include <memory>
#include <vector>

#include "core/intersection/IntersectionTypes.hpp"   // Approach::Direction, LightState
#include "core/math/Vec2.hpp"

class IAgent;
class Intersection;

namespace core::intersection {

// Decision produite par une policy en reponse a une requete d'agent.
struct Decision {
    // Vrai si l'agent peut s'engager (ou continuer) dans l'intersection.
    bool  canEnter = false;

    // Si false : l'agent doit ralentir / s'arreter avant la ligne d'arret.
    // L'agent construira un "leader virtuel" a stopLineGap pixels devant lui,
    // de vitesse zero, et le passera a l'IDM.
    bool  shouldStop = false;
    float stopLineGap = 0.f;

    // Estimation du temps que la situation reste defavorable (info diagnostique).
    // 0.f = inconnu. Utile pour le scheduling / l'attente en STOP/YIELD.
    float yieldUntilT = 0.f;
};

// Snapshot du demandeur passe a la policy.
struct AgentContext {
    Vec2  position;
    float speed = 0.f;        // px/s
    float heading = 0.f;      // degres
    float length = 0.f;       // longueur carrosserie (px)
    float accel = 0.f;        // acceleration max (px/s^2) -> temps pour degager un STOP
    Approach::Direction from = Approach::Direction::NORTH;
};

// Snapshot global passe a la policy. Garde un *pointeur* sur le vecteur
// global d'agents pour permettre l'observation passive (gap acceptance,
// detection de conflits) sans copie.
//
// IMPORTANT : `others` peut contenir `selfAgent`. Les policies DOIVENT
// filtrer le self via comparaison de pointeurs, sinon l'agent peut se
// detecter lui-meme comme conflit (false-positive d'auto-yield).
struct PolicyContext {
    AgentContext  self;
    const IAgent* selfAgent = nullptr;
    float         tileSize  = 50.f;
    const std::vector<std::unique_ptr<IAgent>>* others = nullptr;
};

class IIntersectionPolicy {
public:
    virtual ~IIntersectionPolicy() = default;

    // Repond a la question : "compte tenu du contexte, l'agent peut-il
    // s'engager dans l'intersection, et sinon comment doit-il freiner ?"
    //
    // L'implementation a un accès lecture-seule a l'intersection courante
    // (etat des feux, geometrie, approches).
    virtual Decision request(const PolicyContext& ctx,
                             const Intersection& inter) const = 0;
};

} // namespace core::intersection
