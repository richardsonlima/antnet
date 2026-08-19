
/*
 * antnet_math.h
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

////////////////////////////////////////////////////////////////////////////////////////
/// \file antnet_math.h
/// \brief Pure AntNet routing math (Di Caro & Dorigo), free of ns-2 dependencies.
///
/// These functions implement the numeric core of the AntNet stochastic routing
/// rule and pheromone update rule. They take plain doubles/ints in and return a
/// plain double, so they can be exercised by a standalone unit test without
/// linking against ns-2 (see tests/test_antnet_math.cc).
////////////////////////////////////////////////////////////////////////////////////////

#ifndef __antnet_math_h__
#define __antnet_math_h__

namespace antnet_math {

////////////////////////////////////////////////////////////////////////////
/// \brief Probability of selecting one candidate link towards a destination.
///
/// Combines the pheromone value on the link with a heuristic bias towards
/// links with a shorter queue, weighted by \p alpha. The probabilities
/// returned for the full set of N candidate links (same qtotal, same N,
/// same alpha) sum to 1, provided the pheromone values themselves sum to 1
/// over that set and \p total_queue is the sum of \p link_queue over the
/// same set.
///
/// \param phvalue       pheromone value of this link for the destination.
/// \param alpha         relative weight of heuristic info vs pheromone (ALPHA).
/// \param link_queue    queue length on this candidate link.
/// \param total_queue   sum of queue lengths over all candidate links; caller
///                      must ensure this is >= 1.0 (substitute 1.0 when the
///                      true sum is 0, i.e. all queues empty) to avoid
///                      dividing by zero.
/// \param num_neighbors number of candidate links (N).
////////////////////////////////////////////////////////////////////////////
inline double link_probability(double phvalue, double alpha, double link_queue,
                                double total_queue, int num_neighbors) {
	return (phvalue + alpha * (1.0 - link_queue / total_queue))
	       / (1.0 + alpha * (num_neighbors - 1));
}

////////////////////////////////////////////////////////////////////////////
/// \brief New pheromone value for the link an ant just travelled.
////////////////////////////////////////////////////////////////////////////
inline double reinforce_pheromone(double old_phvalue, double r) {
	return old_phvalue + r * (1.0 - old_phvalue);
}

////////////////////////////////////////////////////////////////////////////
/// \brief New pheromone value for a link an ant did not travel (evaporation).
////////////////////////////////////////////////////////////////////////////
inline double evaporate_pheromone(double old_phvalue, double r) {
	return (1.0 - r) * old_phvalue;
}

} // namespace antnet_math

#endif
