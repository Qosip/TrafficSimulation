// src/Truck.hpp
#pragma once
#include "core/agent/Vehicle.hpp"

class Truck : public Vehicle {
public:
    Truck(float startX, float startY);
    std::string getType() const override { return "TRUCK"; }
};