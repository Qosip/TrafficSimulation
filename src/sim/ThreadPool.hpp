// src/sim/ThreadPool.hpp
//
// Pool de threads PERSISTANT et minimal, dedie a paralleliser la phase de
// decision des agents (computeDecision) sur les machines multi-coeurs.
//
// Pourquoi persistant : la boucle de simulation tourne a 60 Hz (jusqu'a 5
// sous-pas par frame). Creer/detruire des threads a chaque sous-pas couterait
// bien plus que le calcul lui-meme. Les workers sont donc crees UNE fois et
// reveilles a chaque appel a parallelFor (qui est BLOQUANT : il rend la main
// quand toute la plage [0,count) a ete traitee).
//
// Repartition DYNAMIQUE du travail (work-stealing par tranches atomiques) : le
// cout de computeDecision varie d'un agent a l'autre (densite locale, approche
// d'intersection), un decoupage statique desequilibrerait les coeurs.
//
// Le thread appelant PARTICIPE au calcul (il ne dort pas) -> on dimensionne le
// nombre de workers a (hardware_concurrency - 1) pour saturer les coeurs sans
// sur-souscription.
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace sim {

class ThreadPool {
public:
    // threads == 0 -> auto (hardware_concurrency - 1, au moins 1 worker).
    explicit ThreadPool(unsigned threads = 0);
    ~ThreadPool();

    ThreadPool(const ThreadPool&)            = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    // Nombre total de threads participant au calcul (workers + thread appelant).
    unsigned parallelism() const { return workerCount_ + 1; }

    // Appelle body(i) pour chaque i dans [0, count). BLOQUANT : revient quand
    // tout est fait. 'body' doit etre thread-safe vis-a-vis des donnees qu'il
    // touche (ici garanti : chaque agent n'ecrit que son propre etat).
    void parallelFor(std::size_t count, const std::function<void(std::size_t)>& body);

private:
    void workerLoop();
    void runChunks();   // consomme des tranches d'indices jusqu'a epuisement

    unsigned                 workerCount_ = 0;
    std::vector<std::thread> workers_;

    std::mutex              mtx_;
    std::condition_variable cvStart_;
    std::condition_variable cvDone_;

    const std::function<void(std::size_t)>* body_ = nullptr;  // valide pendant parallelFor
    std::size_t              count_     = 0;
    std::atomic<std::size_t> nextIndex_{0};
    std::size_t              remaining_  = 0;   // workers n'ayant pas fini le job courant
    std::uint64_t            generation_ = 0;   // ++ a chaque job (reveil des workers)
    bool                     stop_       = false;
};

} // namespace sim
