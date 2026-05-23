// src/render/SfmlRenderer.hpp
//
// Implementation SFML du IRenderer.
// Centralise TOUT le code qui appelle sf::RenderTarget::draw(...).
// Aucune autre classe du projet ne doit plus parler a SFML pour dessiner.
#pragma once

#include <SFML/Graphics/RenderTarget.hpp>
#include <SFML/Graphics/RenderTexture.hpp>
#include <SFML/Graphics/Sprite.hpp>

#include "render/IRenderer.hpp"

namespace render {

class SfmlRenderer : public IRenderer {
public:
    explicit SfmlRenderer(sf::RenderTarget& target);

    void drawWorldMap(World& world) override;
    void drawIntersections(const World& world) override;
    void drawFlowDebug(const World& world, float time) override;
    void drawAgent(const IAgent& agent) override;
    void drawAgentDebug(const IAgent& agent) override;

    // Force la regeneration de la texture cache au prochain drawWorldMap.
    void invalidateMapCache() { mapCacheValid_ = false; }

private:
    void rebuildMapCache(const World& world);

    sf::RenderTarget& target_;
    sf::RenderTexture mapTexture_;
    sf::Sprite        mapSprite_;
    bool              mapCacheValid_ = false;
};

} // namespace render
