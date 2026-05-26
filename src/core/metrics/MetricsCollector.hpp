// src/core/metrics/MetricsCollector.hpp
//
// Banc d'essai quantitatif (testbed) du rapport SMA : transforme le simulateur
// en instrument de mesure. Collecte, par echantillonnage a chaque pas de temps,
// les indicateurs cles reconnus en ingenierie du trafic :
//
//   * Throughput (debit)        : vehicules evacues par minute (fenetre glissante).
//   * Delay (temps perdu)       : trajet reel - trajet en flux libre (proxy).
//   * Jerk accumule             : integrale de |da/dt| (confort, effet accordeon).
//   * TTC (Time-To-Collision)   : metrique de securite primaire (min + violations).
//   * Vitesse moyenne du reseau.
//
// Concu pour etre 100% observationnel : ne lit que l'interface publique IAgent
// (vitesse, position, cap, VIN, motif de blocage). Aucun couplage au moteur
// decisionnel -> reutilisable tel quel par le runner Monte-Carlo headless.
#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

class IAgent;

namespace core::metrics {

// Seuil de securite : un TTC sous cette valeur (s) est un incident critique.
inline constexpr float kCriticalTTC = 1.5f;

struct AggregateMetrics {
    int   activeVehicles    = 0;
    int   completedVehicles = 0;
    float throughputPerMin  = 0.f;   // completions sur la derniere fenetre (60 s)
    float meanDelaySec      = 0.f;   // delay moyen des vehicules arrives
    float meanSpeed         = 0.f;   // px/s, agents actifs
    float minTTC            = 999.f; // s, plus petit TTC du dernier echantillon
    int   ttcViolations     = 0;     // cumul de paires sous kCriticalTTC
    int   totalStops        = 0;     // cumul d'evenements "arret complet"
    float meanJerkCompleted = 0.f;   // jerk accumule moyen par vehicule arrive
    float simTime           = 0.f;   // temps simule ecoule (s)
};

class MetricsCollector {
public:
    void reset();

    // A appeler APRES integrate() (etat cinematique a jour), une fois par pas
    // de simulation de duree dt (fixe).
    void sample(const std::vector<std::unique_ptr<IAgent>>& agents, float dt);

    const AggregateMetrics& aggregate() const { return agg_; }

    // Series temporelles (echantillonnees ~2 Hz) pour les graphes ImGui.
    const std::vector<float>& seriesThroughput() const { return histThroughput_; }
    const std::vector<float>& seriesDelay()      const { return histDelay_; }
    const std::vector<float>& seriesSpeed()      const { return histSpeed_; }
    const std::vector<float>& seriesMinTTC()     const { return histMinTTC_; }

    // Export CSV : resume agrege + series temporelles. true si ecrit.
    bool exportCsv(const std::string& path) const;

private:
    struct PerAgent {
        float firstTime  = 0.f;
        float travelTime = 0.f;
        float distance   = 0.f;   // ∫ v dt (px)
        float maxSpeed   = 0.f;   // proxy de la vitesse de flux libre
        float prevSpeed  = 0.f;
        float prevAccel  = 0.f;
        float accumJerk  = 0.f;   // ∫ |da/dt| dt
        bool  stopped    = false; // hysteresis pour compter les arrets
        bool  completed  = false;
        bool  seen       = false;
    };

    std::unordered_map<int, PerAgent> agents_;
    AggregateMetrics agg_;
    float simTime_ = 0.f;

    std::vector<float> completionTimes_;   // pour le debit glissant
    float sumDelay_ = 0.f;
    int   countDelay_ = 0;
    float sumJerk_  = 0.f;

    float sampleAccum_ = 0.f;
    std::vector<float> histThroughput_, histDelay_, histSpeed_, histMinTTC_;

    void computeTtc(const std::vector<std::unique_ptr<IAgent>>& agents);
    static void pushHist(std::vector<float>& v, float val);
};

} // namespace core::metrics
