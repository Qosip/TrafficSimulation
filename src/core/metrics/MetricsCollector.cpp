// src/core/metrics/MetricsCollector.cpp
#include "core/metrics/MetricsCollector.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>

#include "core/agent/IAgent.hpp"
#include "core/math/Constants.hpp"
#include "core/math/Vec2.hpp"

namespace core::metrics {

namespace {
constexpr float kSamplePeriod   = 0.5f;   // s entre deux points de serie
constexpr std::size_t kMaxHist  = 1200;   // ~10 min de serie a 2 Hz
constexpr float kThroughputWin  = 60.f;   // fenetre glissante de debit (s)
constexpr float kStopEnter      = 3.f;    // px/s : seuil "arrete"
constexpr float kStopExit       = 15.f;   // px/s : seuil "reparti" (hysteresis)
constexpr float kTtcMaxDist     = 140.f;  // px : au-dela, paire ignoree pour TTC
} // namespace

void MetricsCollector::reset() {
    agents_.clear();
    agg_ = AggregateMetrics{};
    simTime_ = 0.f;
    completionTimes_.clear();
    sumDelay_ = 0.f; countDelay_ = 0; sumJerk_ = 0.f;
    sampleAccum_ = 0.f;
    histThroughput_.clear(); histDelay_.clear();
    histSpeed_.clear();      histMinTTC_.clear();
}

void MetricsCollector::pushHist(std::vector<float>& v, float val) {
    v.push_back(val);
    if (v.size() > kMaxHist) v.erase(v.begin());
}

void MetricsCollector::sample(const std::vector<std::unique_ptr<IAgent>>& agents,
                              float dt) {
    if (dt <= 0.f) return;
    simTime_ += dt;

    float sumSpeed = 0.f;
    int   active   = 0;

    for (const auto& a : agents) {
        if (!a) continue;
        const int vin = a->getVehicleId();
        if (vin < 0) continue;

        PerAgent& pa = agents_[vin];
        const float speed = a->getSpeed();
        if (!pa.seen) {
            pa.seen      = true;
            pa.firstTime = simTime_;
            pa.prevSpeed = speed;
            pa.prevAccel = 0.f;
        }

        const bool atGoal = (a->getBlockReason() == core::agent::BlockReason::AT_GOAL);

        if (!pa.completed) {
            // Cinematique observee -> acceleration et jerk par differences finies.
            const float accel = (speed - pa.prevSpeed) / dt;
            const float jerk  = (accel - pa.prevAccel) / dt;
            pa.accumJerk += std::abs(jerk) * dt;
            pa.prevSpeed  = speed;
            pa.prevAccel  = accel;

            pa.distance   += speed * dt;
            pa.maxSpeed    = std::max(pa.maxSpeed, speed);
            pa.travelTime += dt;

            // Comptage des arrets complets (hysteresis anti-rebond).
            if (!pa.stopped && speed < kStopEnter) {
                pa.stopped = true;
                ++agg_.totalStops;
            } else if (pa.stopped && speed > kStopExit) {
                pa.stopped = false;
            }

            // Arrivee a destination -> fige les metriques de fin de course.
            if (atGoal) {
                pa.completed = true;
                const float freeFlow = pa.distance / std::max(pa.maxSpeed, 1.f);
                const float delay    = std::max(0.f, pa.travelTime - freeFlow);
                sumDelay_ += delay; ++countDelay_;
                sumJerk_  += pa.accumJerk;
                completionTimes_.push_back(simTime_);
                ++agg_.completedVehicles;
            }
        }

        if (!pa.completed) {
            sumSpeed += speed;
            ++active;
        }
    }

    agg_.activeVehicles = active;
    agg_.meanSpeed      = (active > 0) ? (sumSpeed / static_cast<float>(active)) : 0.f;
    agg_.meanDelaySec   = (countDelay_ > 0) ? (sumDelay_ / static_cast<float>(countDelay_)) : 0.f;
    agg_.meanJerkCompleted = (countDelay_ > 0) ? (sumJerk_ / static_cast<float>(countDelay_)) : 0.f;
    agg_.simTime        = simTime_;

    // Debit : completions dans la fenetre glissante -> deja "par minute".
    while (!completionTimes_.empty() &&
           completionTimes_.front() < simTime_ - kThroughputWin) {
        completionTimes_.erase(completionTimes_.begin());
    }
    const float winSpan = std::min(simTime_, kThroughputWin);
    agg_.throughputPerMin = (winSpan > 1e-3f)
        ? (static_cast<float>(completionTimes_.size()) * 60.f / winSpan)
        : 0.f;

    // Echantillonnage des series + TTC a ~2 Hz (limite le cout O(n^2)).
    sampleAccum_ += dt;
    if (sampleAccum_ >= kSamplePeriod) {
        sampleAccum_ = 0.f;
        computeTtc(agents);
        pushHist(histThroughput_, agg_.throughputPerMin);
        pushHist(histDelay_,      agg_.meanDelaySec);
        pushHist(histSpeed_,      agg_.meanSpeed);
        pushHist(histMinTTC_,     std::min(agg_.minTTC, 20.f));
    }
}

void MetricsCollector::computeTtc(const std::vector<std::unique_ptr<IAgent>>& agents) {
    float minTtc = std::numeric_limits<float>::infinity();

    for (std::size_t i = 0; i < agents.size(); ++i) {
        const IAgent* ai = agents[i].get();
        if (!ai) continue;
        const Vec2  pi = ai->getPosition();
        const float si = ai->getSpeed();
        const float ri = ai->getHeading() * math::DEG2RAD;
        const Vec2  vi{ std::cos(ri) * si, std::sin(ri) * si };

        for (std::size_t j = i + 1; j < agents.size(); ++j) {
            const IAgent* aj = agents[j].get();
            if (!aj) continue;
            const Vec2  pj = aj->getPosition();
            Vec2 rel = pj - pi;
            const float dist = rel.length();
            if (dist < 1e-3f || dist > kTtcMaxDist) continue;

            const float sj = aj->getSpeed();
            const float rj = aj->getHeading() * math::DEG2RAD;
            const Vec2  vj{ std::cos(rj) * sj, std::sin(rj) * sj };

            // Vitesse de rapprochement le long de l'axe qui les relie.
            const Vec2  n = rel / dist;
            const Vec2  relVel = vj - vi;
            const float closing = -(n.x * relVel.x + n.y * relVel.y);
            if (closing <= 1.f) continue;                 // s'eloignent / quasi statiques

            const float gap = std::max(0.f, dist - (ai->getLength() + aj->getLength()) * 0.5f);
            const float ttc = gap / closing;
            if (ttc < minTtc) minTtc = ttc;
            if (ttc < kCriticalTTC) ++agg_.ttcViolations;
        }
    }

    agg_.minTTC = std::isfinite(minTtc) ? minTtc : 999.f;
}

bool MetricsCollector::exportCsv(const std::string& path) const {
    std::ofstream f(path);
    if (!f.is_open()) return false;

    f << "# Resume metriques\n";
    f << "metric,value\n";
    f << "sim_time_s,"          << agg_.simTime           << "\n";
    f << "completed_vehicles,"  << agg_.completedVehicles << "\n";
    f << "active_vehicles,"     << agg_.activeVehicles    << "\n";
    f << "throughput_per_min,"  << agg_.throughputPerMin  << "\n";
    f << "mean_delay_s,"        << agg_.meanDelaySec      << "\n";
    f << "mean_speed_px_s,"     << agg_.meanSpeed         << "\n";
    f << "min_ttc_s,"           << agg_.minTTC            << "\n";
    f << "ttc_violations,"      << agg_.ttcViolations     << "\n";
    f << "total_stops,"         << agg_.totalStops        << "\n";
    f << "mean_jerk,"           << agg_.meanJerkCompleted << "\n";

    f << "\n# Series temporelles (echantillon ~2 Hz)\n";
    f << "sample,throughput_per_min,mean_delay_s,mean_speed,min_ttc\n";
    const std::size_t n = histThroughput_.size();
    for (std::size_t k = 0; k < n; ++k) {
        const float tp  = histThroughput_[k];
        const float dl  = (k < histDelay_.size())  ? histDelay_[k]  : 0.f;
        const float sp  = (k < histSpeed_.size())  ? histSpeed_[k]  : 0.f;
        const float tt  = (k < histMinTTC_.size()) ? histMinTTC_[k] : 0.f;
        f << k << "," << tp << "," << dl << "," << sp << "," << tt << "\n";
    }
    return true;
}

bool MetricsCollector::exportJson(const std::string& path) const {
    std::ofstream f(path);
    if (!f.is_open()) return false;

    // Ecrit un tableau JSON de floats a partir d'une serie.
    auto writeArray = [&f](const std::vector<float>& s) {
        f << "[";
        for (std::size_t k = 0; k < s.size(); ++k) {
            if (k) f << ", ";
            f << s[k];
        }
        f << "]";
    };

    f << "{\n";
    f << "  \"summary\": {\n";
    f << "    \"sim_time_s\": "         << agg_.simTime           << ",\n";
    f << "    \"completed_vehicles\": " << agg_.completedVehicles << ",\n";
    f << "    \"active_vehicles\": "    << agg_.activeVehicles    << ",\n";
    f << "    \"throughput_per_min\": " << agg_.throughputPerMin  << ",\n";
    f << "    \"mean_delay_s\": "       << agg_.meanDelaySec      << ",\n";
    f << "    \"mean_speed_px_s\": "    << agg_.meanSpeed         << ",\n";
    f << "    \"min_ttc_s\": "          << agg_.minTTC            << ",\n";
    f << "    \"ttc_violations\": "     << agg_.ttcViolations     << ",\n";
    f << "    \"total_stops\": "        << agg_.totalStops        << ",\n";
    f << "    \"mean_jerk\": "          << agg_.meanJerkCompleted << "\n";
    f << "  },\n";
    f << "  \"series\": {\n";
    f << "    \"throughput_per_min\": "; writeArray(histThroughput_); f << ",\n";
    f << "    \"mean_delay_s\": ";       writeArray(histDelay_);      f << ",\n";
    f << "    \"mean_speed\": ";         writeArray(histSpeed_);      f << ",\n";
    f << "    \"min_ttc\": ";            writeArray(histMinTTC_);     f << "\n";
    f << "  }\n";
    f << "}\n";
    return true;
}

} // namespace core::metrics
