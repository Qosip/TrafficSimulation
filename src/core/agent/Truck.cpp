// src/core/agent/Truck.cpp
#include "core/agent/Truck.hpp"

Truck::Truck(float startX, float startY) : Vehicle(startX, startY) {
    maxSpeed        = 120.f;   // Le camion va moins vite
    maxAcceleration = 30.f;    // Accelere lentement

    bodySize  = {80.f, 30.f};
    bodyColor = {255, 0, 0, 255}; // Red

    setPersonality(core::agent::profiles::truckDriver());

    // Profil IDM camion : prudent, time headway plus long, decel limitee.
    core::behavior::IdmParams p;
    p.T     = 2.0f;
    p.s0    = 6.f;
    p.aMax  = maxAcceleration;
    p.bComf = 50.f;
    p.delta = 4.f;
    setIdmParams(p);
}
