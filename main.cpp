// main.cpp
#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>
#include "Car.hpp"
#include "Truck.hpp"
#include "World.hpp"

int main() {
    sf::RenderWindow window(sf::VideoMode(800, 800), "Simulateur MAS - Sandbox");

    // Centrage de la fenêtre
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
    window.setPosition(sf::Vector2i((desktop.width / 2) - 400, (desktop.height / 2) - 400));
    window.setFramerateLimit(60);

    sf::Clock clock;

    // --- INITIALISATION DE L'ENVIRONNEMENT ---
    World world(800, 800, 50.f);

    // --- INITIALISATION DES AGENTS ---
    std::vector<std::unique_ptr<IAgent>> agents;

    // 1. Le Camion (Commence sur la voie du bas, roule vers l'Est)
    auto myTruck = std::make_unique<Truck>(425.f, 425.f);
    std::vector<sf::Vector2i> truckRoute;
    truckRoute.push_back(sf::Vector2i(8, 8));  // Reste sur Y=8 (voie du bas)
    truckRoute.push_back(sf::Vector2i(12, 8));
    truckRoute.push_back(sf::Vector2i(16, 8));
    dynamic_cast<Truck*>(myTruck.get())->setPath(truckRoute);
    agents.push_back(std::move(myTruck));

    // 2. La Voiture (Commence derrière le camion, puis tourne au Sud)
    auto myCar = std::make_unique<Car>(25.f, 425.f);
    std::vector<sf::Vector2i> carRoute;
    carRoute.push_back(sf::Vector2i(2, 8));
    carRoute.push_back(sf::Vector2i(5, 8));
    carRoute.push_back(sf::Vector2i(7, 8));    // Centre de l'intersection
    carRoute.push_back(sf::Vector2i(7, 10));   // Début de la descente

    // ON AJOUTE DES CASES POUR PROLONGER LA LIGNE DROITE
    carRoute.push_back(sf::Vector2i(7, 15));
    carRoute.push_back(sf::Vector2i(7, 20));
    carRoute.push_back(sf::Vector2i(7, 25));   // Sort loin en bas de l'écran

    dynamic_cast<Car*>(myCar.get())->setPath(carRoute);
    agents.push_back(std::move(myCar));
    bool debugMode = false;
    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();

        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();

            // Dans main.cpp (Boucle des événements)
            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
                int gridX = event.mouseButton.x / 50;
                int gridY = event.mouseButton.y / 50;

                // On pose une route vers la droite par défaut pour le test
                world.setTile(gridX, gridY, TileType::ROAD_RIGHT);
            }
            // --- Touche D pour activer/désactiver ---
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::D) {
                debugMode = !debugMode;
                if (!debugMode) {
                    // Si on quitte le mode debug, on désélectionne tout le monde
                    for (auto& agent : agents) agent->setSelected(false);
                }
            }

            // --- Clic de souris ---
            if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left && debugMode) {
                    // Convertit les pixels de l'écran en coordonnées du monde 2D
                    sf::Vector2f mousePos = window.mapPixelToCoords(sf::Mouse::getPosition(window));

                    for (auto& agent : agents) {
                        if (agent->contains(mousePos)) {
                            agent->setSelected(true);
                        } else {
                            agent->setSelected(false); // Désélectionne les autres
                        }
                    }
                }
            }
        }

        // --- UPDATE ---
        for (auto& agent : agents) {
            agent->update(dt);
        }

        // --- RENDER ---
        // C'est le World qui gère le nettoyage de l'écran (grass) et le dessin des routes
        world.draw(window);

        // Dessin des agents
        for (auto& agent : agents) {
            agent->draw(window);

            // Si le mode debug est actif, on affiche les infos par-dessus
            if (debugMode) {
                agent->drawDebug(window);
            }
        }

        window.display();
    }

    return 0;
}