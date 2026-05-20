// src/Perception.hpp
#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>

class World;
class IAgent;


// Ce qu'un véhicule peut détecter
enum class DetectedType {
    VEHICLE,
    // Extensible : PEDESTRIAN, SIGNAL, OBSTACLE, EMERGENCY...
};

// Un objet détecté par le cône de vision
struct DetectedObject {
    const IAgent* agent;        // Pointeur vers l'agent détecté
    DetectedType type;
    float distance;             // Distance depuis notre position
    float relativeAngle;        // Angle relatif par rapport à notre heading (-180 à 180)
    sf::Vector2f position;      // Position monde de l'objet
};

// Résultat de la perception : ce que le véhicule "voit"
struct PerceptionResult {
    std::vector<DetectedObject> detected;

    // L'obstacle le plus proche directement devant nous (dans un cône étroit)
    bool hasDirectObstacle = false;
    float directObstacleDistance = 0.f;

    // Est-ce qu'on approche d'une intersection ?
    bool approachingIntersection = false;
};

// Paramètres du cône de vision (chaque type de véhicule peut les customiser)
struct VisionParams {
    float range = 150.f;           // Portée en pixels
    float halfAngleDeg = 30.f;     // Demi-angle du cône (30° = cône de 60° total)
    float directHalfAngle = 8.f;   // Cône étroit pour "obstacle direct devant moi"
    float intersectionLookAhead = 120.f;  // Distance à laquelle on commence à scanner les intersections
};

class Perception {
public:
    // Scanne les agents visibles depuis une position/heading donnés
    static PerceptionResult scan(
        sf::Vector2f myPosition,
        float myHeadingDeg,
        const IAgent* myself,
        const std::vector<std::unique_ptr<IAgent>>& agents,
        const World& world,
        const VisionParams& params
    );

    // Dessine le cône de vision en debug
    static void drawDebugCone(
        sf::RenderWindow& window,
        sf::Vector2f position,
        float headingDeg,
        const VisionParams& params,
        const PerceptionResult& result
    );
};