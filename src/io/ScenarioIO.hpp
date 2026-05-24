// src/io/ScenarioIO.hpp
//
// Import/export texte des scenarios de simulation.
// Extraction depuis main.cpp -- decouple la couche fichier du shell UI.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "core/agent/IAgent.hpp"
#include "core/world/World.hpp"

namespace io {

// Serialise World + agents vers un fichier .txt simple (1 ligne = 1 entite).
// Retourne true si le fichier a pu etre ecrit.
bool exportScenario(const std::string& filename,
                    const World& world,
                    const std::vector<std::unique_ptr<IAgent>>& agents);

// Deserialise. outWorld est (re)cree depuis zero, outAgents est vide-puis-rempli.
// Retourne true sur succes (au moins le World a ete cree).
bool importScenario(const std::string& filename,
                    std::unique_ptr<World>& outWorld,
                    std::vector<std::unique_ptr<IAgent>>& outAgents);

} // namespace io
