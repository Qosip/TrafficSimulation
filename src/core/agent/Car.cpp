// src/core/agent/Car.cpp
#include "core/agent/Car.hpp"

Car::Car(float startX, float startY) : Vehicle(startX, startY, 50.f) {
    maxSpeed        = 200.f;   // px/s -- borne sup du v0 IDM
    maxAcceleration = 150.f;
    currentSpeed    = 0.f;

    bodySize  = {30.f, 15.f};
    bodyColor = {0, 0, 255, 255}; // Blue

    // Profil IDM voiture : reactive, time headway court, decel confortable elevee.
    core::behavior::IdmParams p;
    p.T     = 1.2f;
    p.s0    = 5.f;
    p.aMax  = maxAcceleration;
    p.bComf = 80.f;
    p.delta = 4.f;
    setIdmParams(p);
}
