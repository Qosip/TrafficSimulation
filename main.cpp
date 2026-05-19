#include <SFML/Graphics.hpp>

int main() {
    // Init window
    sf::RenderWindow window(sf::VideoMode(800, 800), "Traffic simulation");
    window.setFramerateLimit(60); // 60 fps

    // Loop
    while (window.isOpen()) {

        // --- Event ---
        sf::Event event;
        while (window.pollEvent(event)) {
            // Permet de fermer la fenêtre proprement
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        // --- Update ---

        // --- Draw ---
        window.clear(sf::Color(50, 50, 50));

        // Rectangle test
        sf::RectangleShape testCar(sf::Vector2f(40.0f, 20.0f));
        testCar.setFillColor(sf::Color::Red);
        testCar.setPosition(380.0f, 390.0f); // Middle of the screen
        window.draw(testCar);

        window.display();
    }

    return 0;
}