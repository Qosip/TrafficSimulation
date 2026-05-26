// src/io/ScenarioCatalog.hpp
//
// Catalogue de scenarios PRE-CONFIGURES pour la presentation.
//
// Chaque entree reconstruit integralement (world + agents) une scene jouable en
// UN clic depuis le menu principal. Le catalogue couvre :
//   * les cas les plus simples (route libre, croisement a 2),
//   * un scenario par mode de regulation de carrefour (priorite, STOP, feux,
//     cedez, priorite fixe, P2P, AIM, peloton, rond-point),
//   * les cas limites / degrades (arrivee simultanee 4 vehicules, forte densite
//     P2P facon "four-way stop", depassement, convoi non depassable, panne),
//   * un scenario MASSIF "ville XXXXL" (centaines de vehicules, scalabilite).
//
// Aucune dependance UI : la couche ImGui (main.cpp) se contente d'iterer le
// catalogue et d'appeler 'build'. Cela garde les scenarios testables hors UI.
#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "core/agent/IAgent.hpp"
#include "core/world/World.hpp"

namespace scene {

// Signature d'un constructeur de scene : reconstruit world ET agents en place.
using ScenarioBuildFn =
    std::function<void(std::unique_ptr<World>&,
                       std::vector<std::unique_ptr<IAgent>>&)>;

struct ScenarioDef {
    std::string     name;         // libelle court (menu deroulant)
    std::string     category;     // regroupement d'affichage
    std::string     description;  // phrase pedagogique (affichee sous le menu)
    ScenarioBuildFn build;        // (re)construit la scene
};

// Catalogue complet. L'ordre du vecteur = ordre d'affichage (regroupe par
// 'category', dans l'ordre d'apparition). Construit une seule fois (statique).
const std::vector<ScenarioDef>& scenarioCatalog();

} // namespace scene
