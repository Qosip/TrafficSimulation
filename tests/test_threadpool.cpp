// tests/test_threadpool.cpp
//
// Couvre le pool de threads (sim::ThreadPool) utilise pour paralleliser la phase
// de decision : chaque indice de [0,N) doit etre traite EXACTEMENT une fois, le
// resultat doit etre identique a une execution sequentielle, et une plage vide
// ne doit rien faire (ni crash). Les ecritures se font sur des indices DISJOINTS
// -> pas de course (c'est precisement le contrat de parallelFor).
#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

#include "sim/ThreadPool.hpp"

TEST(ThreadPool, RunsEachIndexExactlyOnce) {
    sim::ThreadPool pool;
    const std::size_t N = 20000;
    std::vector<int> hits(N, 0);

    pool.parallelFor(N, [&](std::size_t i) { hits[i] += 1; });

    std::size_t total = 0;
    bool allOne = true;
    for (int h : hits) { total += static_cast<std::size_t>(h); if (h != 1) allOne = false; }
    EXPECT_TRUE(allOne)      << "chaque indice doit etre traite une seule fois";
    EXPECT_EQ(total, N);
}

TEST(ThreadPool, ParallelResultMatchesSerial) {
    sim::ThreadPool pool;
    const std::size_t N = 200000;
    std::vector<long long> v(N, 0);

    pool.parallelFor(N, [&](std::size_t i) { v[i] = static_cast<long long>(i) * 2; });

    long long got = 0, expected = 0;
    for (std::size_t i = 0; i < N; ++i) { got += v[i]; expected += static_cast<long long>(i) * 2; }
    EXPECT_EQ(got, expected);
}

TEST(ThreadPool, EmptyRangeIsNoOp) {
    sim::ThreadPool pool;
    int calls = 0;
    pool.parallelFor(0, [&](std::size_t) { ++calls; });
    EXPECT_EQ(calls, 0);
    EXPECT_GE(pool.parallelism(), 1u);
}
