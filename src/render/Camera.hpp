// src/Camera.hpp
#pragma once
#include <SFML/Graphics.hpp>

class Camera {
private:
    sf::View view;
    float moveSpeed;
    float zoomLevel;
    float minZoom;
    float maxZoom;
    float zoomStep;
    float baseWidth;
    float baseHeight;

    // Drag à la souris
    bool isDragging;
    sf::Vector2i lastMousePos;

public:
    Camera(float centerX, float centerY, float viewWidth, float viewHeight);

    void handleEvent(const sf::Event& event, const sf::RenderWindow& window);
    void updateViewport(float windowWidth, float windowHeight, float panelWidth);
    void applyTo(sf::RenderWindow& window) const;
    void resetTo(sf::RenderWindow& window) const;
    sf::Vector2f screenToWorld(const sf::RenderWindow& window, sf::Vector2i screenPos) const;

    const sf::View& getView() const;
    float getZoomLevel() const;
    void setMoveSpeed(float speed);
};