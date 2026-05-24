// tests/test_baseline_snapshot.cpp
//
// FILET DE SECURITE ETAPE 0 du refactor.
// Verrouille le comportement actuel de la simulation : avant tout refactor,
// le scenario par defaut tourne pendant N ticks fixes et chaque agent doit
// retrouver les memes (x, y, speed) qu'a la generation du baseline.
//
// Workflow :
//   1. Premier lancement (baseline.csv absent) -> ecrit le fichier et passe.
//   2. Lancements suivants -> charge le fichier et compare avec tolerance.
//   3. Pour re-generer le baseline apres un changement *volontaire* :
//      supprimer tests/baseline.csv, relancer simtests.
//
// La boucle de simulation est appellee avec un dt FIXE (1/60s) pour
// garantir le determinisme, contrairement au mode interactif de main.cpp
// qui derive du wall-clock.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "core/agent/IAgent.hpp"
#include "io/SceneBuilder.hpp"
#include "core/world/World.hpp"

#ifndef BASELINE_PATH
#define BASELINE_PATH "baseline.csv"
#endif

namespace {

struct AgentSample {
    int    step;
    int    agentIdx;
    double posX;
    double posY;
    double speed;
    double heading;
};

constexpr float    kDt          = 1.0f / 60.0f;
constexpr int      kTotalSteps  = 600;          // 10 s simulees
constexpr int      kSampleEvery = 60;           // un snapshot par seconde
constexpr double   kPosTol      = 1e-2;         // pixels
constexpr double   kSpeedTol    = 1e-2;         // px/s
constexpr double   kHeadingTol  = 1e-1;         // degres (cycle 360)

std::vector<AgentSample> runScenario() {
    World world(32, 32, 50.f);
    scene::buildDefaultNetwork(world);

    std::vector<std::unique_ptr<IAgent>> agents;
    scene::spawnDefaultAgents(agents, world);

    std::vector<AgentSample> samples;
    samples.reserve(agents.size() * (kTotalSteps / kSampleEvery + 1));

    for (int step = 0; step <= kTotalSteps; ++step) {
        if (step % kSampleEvery == 0) {
            for (size_t i = 0; i < agents.size(); ++i) {
                const auto& a = agents[i];
                samples.push_back({
                    step,
                    static_cast<int>(i),
                    static_cast<double>(a->getPosition().x),
                    static_cast<double>(a->getPosition().y),
                    static_cast<double>(a->getSpeed()),
                    static_cast<double>(a->getHeading())
                });
            }
        }

        if (step == kTotalSteps) break;
        world.updateIntersections(kDt);
        // Wave 5 : pipeline 2 phases. decision -> integrate avec dt fixe.
        for (auto& a : agents) a->computeDecision(agents, world);
        for (auto& a : agents) a->integrate(kDt);
    }
    return samples;
}

void writeBaseline(const std::vector<AgentSample>& samples, const std::string& path) {
    std::ofstream out(path);
    ASSERT_TRUE(out.is_open()) << "Cannot open baseline file for write: " << path;
    out << "step,agent,posX,posY,speed,heading\n";
    out << std::fixed << std::setprecision(6);
    for (const auto& s : samples) {
        out << s.step << ',' << s.agentIdx << ',' << s.posX << ',' << s.posY
            << ',' << s.speed << ',' << s.heading << '\n';
    }
}

std::vector<AgentSample> readBaseline(const std::string& path) {
    std::vector<AgentSample> v;
    std::ifstream in(path);
    if (!in.is_open()) return v;
    std::string line;
    std::getline(in, line); // header
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        AgentSample s{};
        char c;
        ss >> s.step >> c >> s.agentIdx >> c >> s.posX >> c >> s.posY >> c >> s.speed >> c >> s.heading;
        v.push_back(s);
    }
    return v;
}

double normalizeAngleDiff(double deg) {
    while (deg <= -180.0) deg += 360.0;
    while (deg >    180.0) deg -= 360.0;
    return std::abs(deg);
}

} // namespace

TEST(BaselineSnapshot, DefaultScenarioMatchesGoldenTrace) {
    const std::string path = BASELINE_PATH;

    auto current = runScenario();
    ASSERT_FALSE(current.empty()) << "Scenario produced no samples";

    std::ifstream probe(path);
    if (!probe.good()) {
        // Premier passage : on enregistre le baseline et on passe le test.
        writeBaseline(current, path);
        GTEST_SKIP() << "Baseline absent -> ecrit a " << path
                     << " (" << current.size() << " samples). Relancer pour verifier.";
    }
    probe.close();

    auto golden = readBaseline(path);
    ASSERT_EQ(golden.size(), current.size())
        << "Baseline size mismatch. Re-generer en supprimant " << path;

    for (size_t i = 0; i < current.size(); ++i) {
        const auto& g = golden[i];
        const auto& c = current[i];
        EXPECT_EQ(g.step, c.step)         << "step row " << i;
        EXPECT_EQ(g.agentIdx, c.agentIdx) << "agent row " << i;
        EXPECT_NEAR(g.posX, c.posX,   kPosTol)
            << "posX divergent step=" << c.step << " agent=" << c.agentIdx;
        EXPECT_NEAR(g.posY, c.posY,   kPosTol)
            << "posY divergent step=" << c.step << " agent=" << c.agentIdx;
        EXPECT_NEAR(g.speed, c.speed, kSpeedTol)
            << "speed divergent step=" << c.step << " agent=" << c.agentIdx;
        EXPECT_LT(normalizeAngleDiff(g.heading - c.heading), kHeadingTol)
            << "heading divergent step=" << c.step << " agent=" << c.agentIdx;
    }
}

TEST(BaselineSnapshot, ScenarioIsDeterministicAcrossRuns) {
    // Garantit que deux runs successifs produisent strictement la meme trace.
    // Si ce test casse mais que DefaultScenarioMatchesGoldenTrace passe,
    // c'est qu'un etat global cache (RNG non seede, static mutables) s'est glisse.
    auto run1 = runScenario();
    auto run2 = runScenario();
    ASSERT_EQ(run1.size(), run2.size());
    for (size_t i = 0; i < run1.size(); ++i) {
        EXPECT_EQ(run1[i].posX,    run2[i].posX);
        EXPECT_EQ(run1[i].posY,    run2[i].posY);
        EXPECT_EQ(run1[i].speed,   run2[i].speed);
        EXPECT_EQ(run1[i].heading, run2[i].heading);
    }
}
