// src/Camera.cpp
#include "Camera.hpp"

Camera::Camera(float centerX, float centerY, float viewWidth, float viewHeight)
    : moveSpeed(500.f), zoomLevel(1.f), minZoom(0.25f), maxZoom(4.f), zoomStep(1.15f),
      baseWidth(viewWidth), baseHeight(viewHeight), isDragging(false)
{
    view.setSize(viewWidth, viewHeight);
    view.setCenter(centerX, centerY);
}

void Camera::handleEvent(const sf::Event& event, const sf::RenderWindow& window) {
    // --- Zoom molette ---
    if (event.type == sf::Event::MouseWheelScrolled) {
        if (event.mouseWheelScroll.delta > 0) {
            zoomLevel /= zoomStep;
            if (zoomLevel < minZoom) zoomLevel = minZoom;
        } else {
            zoomLevel *= zoomStep;
            if (zoomLevel > maxZoom) zoomLevel = maxZoom;
        }
        view.setSize(baseWidth * zoomLevel, baseHeight * zoomLevel);
    }

    // --- Drag souris (clic droit maintenu) ---
    if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Right) {
        isDragging = true;
        lastMousePos = sf::Mouse::getPosition(window);
    }
    if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Right) {
        isDragging = false;
    }
    if (event.type == sf::Event::MouseMoved && isDragging) {
        sf::Vector2i currentMousePos = sf::Mouse::getPosition(window);
        sf::Vector2i delta = lastMousePos - currentMousePos;

        // Convertir le delta pixel en delta monde (proportionnel au zoom)
        float scaleX = view.getSize().x / (float)window.getSize().x;
        float scaleY = view.getSize().y / (float)window.getSize().y;

        view.move(delta.x * scaleX, delta.y * scaleY);
        lastMousePos = currentMousePos;
    }
}

void Camera::applyTo(sf::RenderWindow& window) const {
    window.setView(view);
}

void Camera::resetTo(sf::RenderWindow& window) const {
    window.setView(window.getDefaultView());
}

sf::Vector2f Camera::screenToWorld(const sf::RenderWindow& window, sf::Vector2i screenPos) const {
    return window.mapPixelToCoords(screenPos, view);
}

const sf::View& Camera::getView() const {
    return view;
}

float Camera::getZoomLevel() const {
    return zoomLevel;
}

void Camera::setMoveSpeed(float speed) {
    moveSpeed = speed;
}