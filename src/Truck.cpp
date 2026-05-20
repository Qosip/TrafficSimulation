// src/Truck.cpp
#include "Truck.hpp"

Truck::Truck(float startX, float startY) : Vehicle(startX, startY) {
    maxSpeed = 120.f;       // Le camion va moins vite
    maxAcceleration = 30.f; // Il accélère très lentement (lourd)

    shape.setSize(sf::Vector2f(80.f, 30.f)); // Plus grand
    shape.setOrigin(40.f, 15.f);
    shape.setFillColor(sf::Color::Red);
}