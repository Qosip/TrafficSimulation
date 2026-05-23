// src/core/math/Rng.hpp
//
// Generateur pseudo-aleatoire SEEDABLE et reproductible.
// Aucun appel a rand() ni std::random_device dans le reste du core :
// tout passe par cette classe. Indispensable pour les runs Monte-Carlo
// et la reproductibilite des snapshot tests.
#pragma once

#include <cstdint>
#include <random>

namespace core {

class Rng {
public:
    explicit Rng(std::uint64_t seed = 0xC0FFEEULL) : engine_(seed) {}

    void reseed(std::uint64_t seed) { engine_.seed(seed); }

    // Tirage uniforme dans [min, max].
    float uniform(float min, float max) {
        std::uniform_real_distribution<float> d(min, max);
        return d(engine_);
    }

    // Tirage uniforme entier dans [min, max].
    int uniformInt(int min, int max) {
        std::uniform_int_distribution<int> d(min, max);
        return d(engine_);
    }

    // Distribution gaussienne (utile pour profils psychologiques bruites).
    float normal(float mean, float stddev) {
        std::normal_distribution<float> d(mean, stddev);
        return d(engine_);
    }

    bool bernoulli(float p) {
        std::bernoulli_distribution d(p);
        return d(engine_);
    }

    std::mt19937_64& engine() { return engine_; }

private:
    std::mt19937_64 engine_;
};

} // namespace core
