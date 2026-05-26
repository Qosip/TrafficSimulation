// src/core/agent/Car.cpp
#include "core/agent/Car.hpp"

Car::Car(float startX, float startY) : Vehicle(startX, startY, 50.f) {
    maxSpeed        = 200.f;   // px/s -- borne sup du v0 IDM
    maxAcceleration = 150.f;
    currentSpeed    = 0.f;

    bodySize  = {30.f, 15.f};
    bodyColor = {0, 0, 255, 255}; // Blue

    setPersonality(core::agent::profiles::normalDriver());

    // Profil IDM voiture : reactive, time headway court, decel confortable elevee.
    // T pilote la distance de declenchement du freinage (terme v*T) : plus court
    // => on freine PLUS TARD ; bComf eleve => freinage PLUS FRANC a l'approche.
    core::behavior::IdmParams p;
    p.T     = 1.0f;
    p.s0    = 5.f;
    p.aMax  = maxAcceleration;
    p.bComf = 120.f;
    p.delta = 4.f;
    setIdmParams(p);   // applique automatiquement la personnalite
}
