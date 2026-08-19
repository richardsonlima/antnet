
/*
 * antnet_rtable.cc
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * If you are using this program for any publication, we kindly request that you cite:
 * "Ant Colony Optimisation Based Routing on NS-2", 
 * V. Laxmi, Lavina Jain and M. S. Gaur, 
 * International Conference on Wireless Communication and Sensor Networks (WCSN), 
 * India, December 2006.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA.
 *
 * Author: Lavina Jain
 *
 */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \file antnet_rtable.cc
/// \brief Implementation file for routing table in AntNet
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "antnet_rtable.h"

/// Reinforcement factor. Not read by calc_next()/update() any more (they take
/// r/num_neighbors as explicit parameters - see antnet_rtable.h) since a
/// single shared global was silently overwritten across per-node calls in
/// this single-process simulation. Left in place only because the unused
/// Antnet::update_traffic() traffic-model code (see antnet.cc) still writes
/// to it.
double r;
int NUM_NODES;	///< number of nodes in topology (same for every node, safe as a global)

///////////////////////////////////////////////////
/// Method to add an entry in routing table
/// Parameters:
/// - destination node
/// - neighbor node
/// - pheromone value
///////////////////////////////////////////////////
void antnet_rtable::add_entry(nsaddr_t dest, nsaddr_t next, double phvalue) {
	struct pheromone temp_pheromone;	// create new pheromone structure
	temp_pheromone.neighbor = next;		// set neighbor node address
	temp_pheromone.phvalue = phvalue;	// set pheromone value
	rtable_t::iterator iterRt = rt_.find(dest);
	if(iterRt == rt_.end()) {	// destination entry not in rtable, add new entry
		pheromone_matrix temp;
		temp.push_back(temp_pheromone);
		rt_[dest] = temp;
	}
	else {	// destination entry exists in rtable, add neighbor entry
		pheromone_matrix *temp = &((*iterRt).second);
		temp->push_back(temp_pheromone);
	}
}

///////////////////////////////////////////////////
/// Method to print routing table
///////////////////////////////////////////////////
void antnet_rtable::print() {
	FILE *fp = fopen(file_rtable,"a");
	fprintf(fp,"\tdest\tnext\tphvalue\n");
	for(rtable_t::iterator iter = rt_.begin(); iter != rt_.end(); iter++) {
		pheromone_matrix vect_pheromone = (*iter).second;
		for(pheromone_matrix::iterator iterPh = vect_pheromone.begin(); iterPh != vect_pheromone.end(); iterPh++) {
			fprintf(fp,"\t%d \t%d \t%f \n", (*iter).first, (*iterPh).neighbor, (*iterPh).phvalue);
		}
	}
	fclose(fp);
}

//////////////////////////////////////////////////////////////////////
/// Method to return a randomly chosen destination for a source node
/////////////////////////////////////////////////////////////////////
nsaddr_t antnet_rtable::calc_destination(nsaddr_t source) {
	int tmp_int;
	do {
		tmp_int = rnum->uniform(NUM_NODES);
	} while(tmp_int == source);
	//printf("-----the value of tmp int is %d\n",tmp_int);
	return tmp_int;
}

//////////////////////////////////////////////////////////////////////
/// Method to implement AntNet algorithm
/// Returns next hop node address
/// Parameters:
/// - source node address
/// - destination node address
/// - parent node (to avoid loopback; pass source itself when there is no
///   parent to exclude, e.g. for the first hop of a freshly generated ant)
/// - number of candidate neighbor links at source
/////////////////////////////////////////////////////////////////////
nsaddr_t antnet_rtable::calc_next(nsaddr_t source, nsaddr_t dest, nsaddr_t parent, int num_neighbors) {
	// find routing table entry for destination node
	rtable_t::iterator iter = rt_.find(dest);
	if(DEBUG)
		fprintf(stdout,"in calc_next at source %d dest %d parent %d\n",source,dest,parent);

	if(iter == rt_.end()) {
		// No routing table entry for this destination. Rather than falling
		// off the end of the function (undefined behaviour - this used to
		// return whatever garbage was left on the stack), bounce the packet
		// back the way it came; callers already treat next==parent as a
		// dead end and drop the packet.
		if(DEBUG)
			fprintf(stderr, "calc_next: no rtable entry for dest %d at node %d\n", dest, source);
		return parent;
	}

	pheromone_matrix &vect_pheromone = (*iter).second;
	size_t n = vect_pheromone.size();

	if(DEBUG) {
		fprintf(stdout,"neighbors of %d:\t",source);
		for(size_t k = 0; k < n; k++) {
			fprintf(stdout,"%d\t%f\t",vect_pheromone[k].neighbor, vect_pheromone[k].phvalue);
		}
		fprintf(stdout,"\n");
	}

	// Look up each candidate link's queue length once (each lookup is a
	// round trip through Tcl via get_queue_length()) instead of the 2-3
	// times per link the original implementation performed.
	std::vector<double> queue_len(n);
	double qtotal = 0.0;
	for(size_t k = 0; k < n; k++) {
		Node *node1 = Node::get_node_by_address(source);
		Node *node2 = Node::get_node_by_address(vect_pheromone[k].neighbor);
		queue_len[k] = get_queue_length(node1, node2);
		qtotal += queue_len[k];
	}
	if(qtotal == 0.0)
		qtotal = 1.0;

	// Combine pheromone with the queue-length heuristic into the AntNet
	// selection probability for each candidate link (Di Caro & Dorigo).
	std::vector<double> prob(n);
	double total_excl_parent = 0.0;
	for(size_t k = 0; k < n; k++) {
		prob[k] = antnet_math::link_probability(vect_pheromone[k].phvalue, ALPHA,
		                                         queue_len[k], qtotal, num_neighbors);
		if(vect_pheromone[k].neighbor != parent)
			total_excl_parent += prob[k];
	}

	// Dead end: the only candidate link is the one the ant arrived on.
	if(total_excl_parent <= 0.0)
		return parent;

	// Sample a next hop among the candidate links, excluding the parent
	// link, proportionally to AntNet selection probability.
	double x = rnum->uniform(total_excl_parent);
	double cumulative = 0.0;
	nsaddr_t last_candidate = parent;
	for(size_t k = 0; k < n; k++) {
		if(vect_pheromone[k].neighbor == parent)
			continue;
		last_candidate = vect_pheromone[k].neighbor;
		cumulative += prob[k];
		if(x < cumulative)
			return vect_pheromone[k].neighbor;
	}
	// Floating point rounding may leave a tiny residual mass unclaimed by
	// the loop above; hand it to the last candidate rather than falling
	// through with no return value.
	return last_candidate;
}

///////////////////////////////////////////////////////////////////////////////////
/// Method to update routing table
/// Parameters:
/// - destination node address
/// - neighbor node address
/// - reinforcement factor (the calling agent's own r_factor_)
/// .
/// This method increments and evaporates pheromone values as per AntNet algorithm
///////////////////////////////////////////////////////////////////////////////////
void antnet_rtable::update(nsaddr_t dest, nsaddr_t next, double r_factor) {
	// read routing table entry for destination
	rtable_t::iterator iterRt = rt_.find(dest);
	if(iterRt != rt_.end()) {
		pheromone_matrix &vect_pheromone = (*iterRt).second;
		for(pheromone_matrix::iterator iterPh = vect_pheromone.begin(); iterPh != vect_pheromone.end(); iterPh++) {
			if((*iterPh).neighbor == next)
				// increase ph value for link travelled by ant
				(*iterPh).phvalue = antnet_math::reinforce_pheromone((*iterPh).phvalue, r_factor);
			else
				// evaporate pheromone for other links
				(*iterPh).phvalue = antnet_math::evaporate_pheromone((*iterPh).phvalue, r_factor);
		}
	}
}
