// src/sim/ThreadPool.cpp
#include "sim/ThreadPool.hpp"

#include <algorithm>

namespace sim {

namespace {
// Taille de tranche prelevee atomiquement par chaque participant. Assez grande
// pour amortir le fetch_add, assez petite pour equilibrer la charge.
constexpr std::size_t kChunk = 16;
}

ThreadPool::ThreadPool(unsigned threads) {
    if (threads == 0) {
        const unsigned hw = std::thread::hardware_concurrency();
        threads = (hw > 1) ? (hw - 1) : 1;   // le thread appelant calcule aussi
    }
    workerCount_ = threads;
    workers_.reserve(workerCount_);
    for (unsigned i = 0; i < workerCount_; ++i)
        workers_.emplace_back([this] { workerLoop(); });
}

ThreadPool::~ThreadPool() {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        stop_ = true;
        ++generation_;             // reveille les workers pour qu'ils sortent
    }
    cvStart_.notify_all();
    for (auto& t : workers_)
        if (t.joinable()) t.join();
}

void ThreadPool::runChunks() {
    for (;;) {
        const std::size_t start = nextIndex_.fetch_add(kChunk, std::memory_order_relaxed);
        if (start >= count_) break;
        const std::size_t end = std::min(start + kChunk, count_);
        for (std::size_t i = start; i < end; ++i) (*body_)(i);
    }
}

void ThreadPool::workerLoop() {
    std::uint64_t lastGen = 0;
    for (;;) {
        {
            std::unique_lock<std::mutex> lk(mtx_);
            cvStart_.wait(lk, [this, lastGen] { return stop_ || generation_ != lastGen; });
            if (stop_) return;
            lastGen = generation_;
        }

        runChunks();   // body_ / count_ / nextIndex_ stables pour toute la duree du job

        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (--remaining_ == 0) cvDone_.notify_one();
        }
    }
}

void ThreadPool::parallelFor(std::size_t count,
                             const std::function<void(std::size_t)>& body) {
    if (count == 0) return;
    // Mono-thread ou trop peu de travail : execution directe (zero synchro).
    if (workerCount_ == 0) {
        for (std::size_t i = 0; i < count; ++i) body(i);
        return;
    }

    {
        std::lock_guard<std::mutex> lk(mtx_);
        body_      = &body;
        count_     = count;
        nextIndex_.store(0, std::memory_order_relaxed);
        remaining_ = workerCount_;
        ++generation_;
    }
    cvStart_.notify_all();

    runChunks();   // le thread appelant participe au lieu de dormir

    {
        std::unique_lock<std::mutex> lk(mtx_);
        cvDone_.wait(lk, [this] { return remaining_ == 0; });
        body_ = nullptr;   // l'objet 'body' du caller va disparaitre apres retour
    }
}

} // namespace sim
