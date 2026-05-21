// src/Car.hpp
#pragma once
#include "Vehicle.hpp"

class Car : public Vehicle {
public:
    Car(float startX, float startY);
    std::string getType() const override { return "CAR"; }
};