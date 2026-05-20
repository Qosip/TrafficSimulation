// src/Car.cpp
#include "Car.hpp"

Car::Car(float startX, float startY) : Vehicle(startX, startY, 50.f) {
    maxSpeed = 200.f;          // En pixels par seconde
    maxAcceleration = 150.f;   // Capacités du moteur
    currentSpeed = 0.f;        // On démarre à l'arrêt

    shape.setSize(sf::Vector2f(30.f, 15.f));
    shape.setFillColor(sf::Color::Blue);
    shape.setOrigin(15.f, 7.5f); // Centre de rotation
}