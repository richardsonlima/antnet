/*
 * tests/test_antnet_math.cc
 *
 * Standalone unit tests for the pure AntNet math in antnet_math.h.
 *
 * This deliberately does NOT depend on ns-2 (agent.h, node.h, packet.h, ...).
 * antnet_math.h is the one piece of the AntNet implementation with no ns-2
 * dependency, so it's the one piece we can compile and exercise without a
 * full ns-2.34 source tree + OTcl/TclCL installed. Build & run with:
 *
 *   cd tests && make
 *
 * or directly:
 *
 *   g++ -std=c++11 -Wall -Wextra -o test_antnet_math test_antnet_math.cc && ./test_antnet_math
 */

#include "../antnet_math.h"

#include <cmath>
#include <cstdio>
#include <vector>

static int g_checks = 0;
static int g_failures = 0;

static bool nearly_equal(double a, double b, double eps = 1e-9) {
	return std::fabs(a - b) < eps;
}

#define CHECK(cond) do { \
	g_checks++; \
	if (!(cond)) { \
		g_failures++; \
		std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
	} \
} while (0)

#define CHECK_NEAR(a, b) CHECK(nearly_equal((a), (b)))

////////////////////////////////////////////////////////////////////////////
// reinforce_pheromone / evaporate_pheromone
////////////////////////////////////////////////////////////////////////////

static void test_reinforce_pheromone() {
	// r = 0: no reinforcement update should occur.
	CHECK_NEAR(antnet_math::reinforce_pheromone(0.4, 0.0), 0.4);
	// r = 1: pheromone jumps straight to 1 (fully reinforced link).
	CHECK_NEAR(antnet_math::reinforce_pheromone(0.4, 1.0), 1.0);
	// General case: old + r*(1 - old).
	CHECK_NEAR(antnet_math::reinforce_pheromone(0.4, 0.5), 0.7);
	// Reinforcement never decreases pheromone and never exceeds 1 for r in [0,1].
	for (double old_ph = 0.0; old_ph <= 1.0; old_ph += 0.1) {
		for (double r = 0.0; r <= 1.0; r += 0.1) {
			double v = antnet_math::reinforce_pheromone(old_ph, r);
			CHECK(v >= old_ph - 1e-9);
			CHECK(v <= 1.0 + 1e-9);
		}
	}
}

static void test_evaporate_pheromone() {
	// r = 0: no evaporation.
	CHECK_NEAR(antnet_math::evaporate_pheromone(0.5, 0.0), 0.5);
	// r = 1: pheromone fully evaporates.
	CHECK_NEAR(antnet_math::evaporate_pheromone(0.5, 1.0), 0.0);
	// General case: (1-r)*old.
	CHECK_NEAR(antnet_math::evaporate_pheromone(0.5, 0.3), 0.35);
	// Evaporation never increases pheromone and never goes negative for r in [0,1].
	for (double old_ph = 0.0; old_ph <= 1.0; old_ph += 0.1) {
		for (double r = 0.0; r <= 1.0; r += 0.1) {
			double v = antnet_math::evaporate_pheromone(old_ph, r);
			CHECK(v <= old_ph + 1e-9);
			CHECK(v >= 0.0 - 1e-9);
		}
	}
}

// A single AntNet update() call must leave the pheromone vector for a
// destination still summing to 1 (reinforce the travelled link, evaporate
// the rest) - this is the invariant calc_next()'s probability formula
// relies on.
static void test_update_preserves_normalization() {
	std::vector<double> ph;
	ph.push_back(0.5);
	ph.push_back(0.3);
	ph.push_back(0.2);
	int travelled = 1; // index into ph[] of the link the ant travelled
	double r = 0.3;

	double sum = 0.0;
	for (size_t i = 0; i < ph.size(); i++) {
		ph[i] = (int)i == travelled
			? antnet_math::reinforce_pheromone(ph[i], r)
			: antnet_math::evaporate_pheromone(ph[i], r);
		sum += ph[i];
	}
	CHECK_NEAR(sum, 1.0);
}

////////////////////////////////////////////////////////////////////////////
// link_probability
////////////////////////////////////////////////////////////////////////////

static void test_link_probability_alpha_zero_reduces_to_pheromone() {
	// With no heuristic weight, selection probability is just the pheromone
	// value, regardless of queue length.
	CHECK_NEAR(antnet_math::link_probability(0.6, 0.0, /*link_queue=*/100.0, /*total_queue=*/100.0, /*N=*/2), 0.6);
	CHECK_NEAR(antnet_math::link_probability(0.6, 0.0, /*link_queue=*/0.0, /*total_queue=*/100.0, /*N=*/2), 0.6);
}

static void test_link_probability_sums_to_one() {
	// Per Di Caro & Dorigo: if the pheromone values over the full candidate
	// set sum to 1, and total_queue is the sum of link_queue over that same
	// set, then the resulting selection probabilities also sum to 1.
	double alpha = 0.45;
	int N = 4;
	double ph[4]    = {0.40, 0.30, 0.20, 0.10};
	double queue[4] = {5.0,  0.0,  10.0, 5.0};
	double total_queue = 20.0;

	double sum = 0.0;
	for (int i = 0; i < N; i++) {
		sum += antnet_math::link_probability(ph[i], alpha, queue[i], total_queue, N);
	}
	CHECK_NEAR(sum, 1.0);
}

static void test_link_probability_equal_inputs_split_evenly() {
	// Equal pheromone and equal queue across all candidates: the heuristic
	// term is the same for every link, so the split stays even.
	double alpha = 0.45;
	int N = 3;
	double p = antnet_math::link_probability(1.0 / 3.0, alpha, 10.0, 30.0, N);
	CHECK_NEAR(p, 1.0 / 3.0);
}

static void test_link_probability_prefers_shorter_queue() {
	// Equal pheromone, but link A has a shorter queue than link B: AntNet's
	// heuristic term should make A more likely to be chosen than B. This is
	// the behaviour that was silently discarded by the bug in the original
	// calc_next() (it computed this formula into `thisprob` but then
	// partitioned the selection range using raw pheromone `thisph` instead).
	double alpha = 0.45;
	int N = 2;
	double total_queue = 10.0;
	double prob_short_queue = antnet_math::link_probability(0.5, alpha, /*link_queue=*/2.0, total_queue, N);
	double prob_long_queue  = antnet_math::link_probability(0.5, alpha, /*link_queue=*/8.0, total_queue, N);
	CHECK(prob_short_queue > prob_long_queue);
}

static void test_link_probability_single_neighbor_is_certain() {
	// N=1: the only candidate link is chosen with probability 1 regardless
	// of alpha/queue (1 + alpha*(N-1) == 1 when N==1).
	double p = antnet_math::link_probability(1.0, 0.45, 3.0, 3.0, 1);
	CHECK_NEAR(p, 1.0);
}

int main() {
	test_reinforce_pheromone();
	test_evaporate_pheromone();
	test_update_preserves_normalization();
	test_link_probability_alpha_zero_reduces_to_pheromone();
	test_link_probability_sums_to_one();
	test_link_probability_equal_inputs_split_evenly();
	test_link_probability_prefers_shorter_queue();
	test_link_probability_single_neighbor_is_certain();

	std::printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
	return g_failures == 0 ? 0 : 1;
}
