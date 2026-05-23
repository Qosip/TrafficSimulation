// src/render/NullRenderer.hpp
//
// Implementation no-op du IRenderer : utilisee en mode headless
// (tests Monte-Carlo, simulations massives sans fenetre).
#pragma once

#include "render/IRenderer.hpp"

namespace render {

class NullRenderer : public IRenderer {
public:
    void drawWorldMap(World& /*world*/) override {}
    void drawIntersections(const World& /*world*/) override {}
    void drawFlowDebug(const World& /*world*/, float /*time*/) override {}
    void drawAgent(const IAgent& /*agent*/) override {}
    void drawAgentDebug(const IAgent& /*agent*/) override {}
};

} // namespace render
