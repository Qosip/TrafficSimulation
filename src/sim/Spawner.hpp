// src/sim/Spawner.hpp
//
// Generateur de trafic STOCHASTIQUE et hautement parametrable (pour l'analyse
// future des flux). Centralise la creation de vehicules : tirage du TYPE
// (voiture / camion) et d'un PROFIL comportemental (normal / agressif / prudent)
// selon une distribution configurable, calcul du chemin A*, application de la
// personnalite (qui module dynamiquement les parametres cinematiques IDM :
// temps de reaction T, acceleration aMax, freinage bComf, conformite vitesse)
// puis bruit gaussien par-conducteur.
//
// Seede -> reproductible. Sans dependance UI : reutilisable par le banc d'essai
// Monte-Carlo (headless) comme par la session visuelle.
#pragma once

#include <cstdint>
#include <memory>

#include "core/agent/IAgent.hpp"
#include "core/agent/Personality.hpp"
#include "core/math/Rng.hpp"
#include "core/math/TileCoord.hpp"

class World;

namespace sim {

// Distribution stochastique pilotant la generation. Tous les champs sont
// exposes pour edition live (panneau Monte-Carlo). Les poids n'ont pas besoin
// de sommer a 1 : ils sont normalises a la volee.
struct SpawnProfile {
    // --- Type de vehicule (poids relatifs) ---
    float wCar   = 0.85f;
    float wTruck = 0.15f;

    // --- Profil comportemental des VOITURES (poids relatifs). Chaque profil
    //     modifie les parametres IDM via Personality. Les camions gardent
    //     toujours le profil "truckDriver" (inertie / prudence). ---
    float wNormal     = 0.50f;
    float wAggressive = 0.25f;
    float wCalm       = 0.25f;

    // --- Heterogeneite gaussienne additionnelle par-conducteur ---
    bool  gaussianHeterogeneity = true;
    float driverSigma           = 0.15f;   // ecart-type du bruit (T / aMax / vitesse)
};

class Spawner {
public:
    Spawner(SpawnProfile profile, std::uint64_t seed);

    // Cree un vehicule au tile 'start' avec route vers 'goal'. Renvoie nullptr
    // si aucun chemin n'existe (l'appelant saute alors ce point d'apparition).
    std::unique_ptr<IAgent> spawn(World& world,
                                  core::TileCoord start,
                                  core::TileCoord goal);

    SpawnProfile&       profile()       { return profile_; }
    const SpawnProfile& profile() const { return profile_; }

    // Expose le RNG pour que l'appelant tire dans le MEME flux seede (choix de
    // l'origine, du virage...) et conserve la reproductibilite globale.
    core::Rng& rng() { return rng_; }

private:
    SpawnProfile profile_;
    core::Rng    rng_;

    bool                     rollTruck();
    core::agent::Personality rollPersonality(bool isTruck);
};

} // namespace sim
