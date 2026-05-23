// src/render/IRenderer.hpp
//
// Interface abstraite du moteur de rendu.
// Le noyau de simulation n'en depend pas : il appelle juste les methodes
// via cette interface, ce qui permet de brancher :
//   - SfmlRenderer   : rendu temps reel
//   - NullRenderer   : aucun rendu (mode headless / Monte-Carlo)
//   - (futur)        : exports video, replay, etc.
//
// Aucun symbole SFML n'apparait ici. Les implementations peuvent en
// utiliser librement dans leur .cpp.
#pragma once

class World;
class IAgent;

namespace render {

class IRenderer {
public:
    virtual ~IRenderer() = default;

    // Dessine la carte raster (route, herbe, pointilles).
    // L'implementation est libre de cacher le rendu en interne.
    virtual void drawWorldMap(World& world) = 0;

    // Dessine les signaux d'intersection (feux, ceder le passage...).
    virtual void drawIntersections(const World& world) = 0;

    // Trainee lumineuse de debogage indiquant les flux de circulation.
    virtual void drawFlowDebug(const World& world, float time) = 0;

    // Carrosserie d'un agent (position + heading + size + color).
    virtual void drawAgent(const IAgent& agent) = 0;

    // Surcouche de debug (cone de vision, trajectoire, detections).
    virtual void drawAgentDebug(const IAgent& agent) = 0;
};

} // namespace render
