
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

double r;	///< reinforcement factor (value read from tcl script)
int N;		///< Number of neighbors of a node
int NUM_NODES;	///< number of nodes in topology

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
/// - parent node (to avoid loopback)
/////////////////////////////////////////////////////////////////////
nsaddr_t antnet_rtable::calc_next(nsaddr_t source, nsaddr_t dest, nsaddr_t parent) {
	nsaddr_t next, nextn;
	double thisph;
	double thisqueue;
	double thisprob;
	double maxprob = 0.0;
	double maxph = 0.0;
	double lrange = 0.0, urange = 0.0;
	// find routing table entry for destination node
	rtable_t::iterator iter = rt_.find(dest);
	double qtotal = 0.0;
	if(DEBUG) 
		fprintf(stdout,"in calc_next at source %d dest %d parent %d\n",source,dest,parent);
	if(iter != rt_.end()) {
		pheromone_matrix vect_pheromone;
	
		if(DEBUG) {
			vect_pheromone = (*iter).second;
			fprintf(stdout,"neighbors of %d:\t",source);
			for(pheromone_matrix::iterator iterPh = vect_pheromone.begin(); iterPh != vect_pheromone.end(); iterPh++) {
				fprintf(stdout,"%d\t%f\t",(*iterPh).neighbor, (*iterPh).phvalue);
			}
			fprintf(stdout,"\n");
		}

		// read vector of pheromone values for the destination node
		vect_pheromone = (*iter).second;
		for(pheromone_matrix::iterator iterPh = vect_pheromone.begin(); iterPh != vect_pheromone.end(); iterPh++) {
			next = (*iterPh).neighbor;
			Node *node1 = node1->get_node_by_address(source);
			Node *node2 = node2->get_node_by_address(next);
			int temp_len = get_queue_length(node1,node2);
			qtotal += temp_len;
		}
		if(qtotal == 0.0) {
			qtotal = 1.0;
		}
		
		// calculate probability range for parent link
		lrange = 0.0;
		urange = 0.0;
		for(pheromone_matrix::iterator iterPh = vect_pheromone.begin(); iterPh != vect_pheromone.end(); iterPh++) {
			thisph = (*iterPh).phvalue;
			next = (*iterPh).neighbor;
			Node *node1 = node1->get_node_by_address(source);
			Node *node2 = node2->get_node_by_address(next);
			int thisqueue = get_queue_length(node1,node2);
			thisprob = (thisph + ALPHA*(1 - thisqueue/qtotal)) / (1 + ALPHA*(N-1));
			//thisprob = thisph;
			if(next == parent) {
				urange = lrange + (thisph);
				break;
			}
			lrange += (thisph);
		}
		if(urange == 0.0)
			urange = 1.0;
		
		// dead end, loopback
		if(lrange == 0.0 && urange == 1.0) {
		//	printf("return parent %d\n",parent);
			return parent;
		}
		
		// generate random probability value, out of range of parent link
		double tmp_double;
		do {
			tmp_double = rnum->uniform(1.0);
		}while(tmp_double >= lrange && tmp_double < urange);

		// find next hop node corresponding to this range of probability
		lrange = 0.0;
		urange = 0.0;
		for(pheromone_matrix::iterator iterPh = vect_pheromone.begin(); iterPh != vect_pheromone.end(); iterPh++) {
			thisph = (*iterPh).phvalue;
			next = (*iterPh).neighbor;
			Node *node1 = node1->get_node_by_address(source);
			Node *node2 = node2->get_node_by_address(next);
			int thisqueue = get_queue_length(node1,node2);
			thisprob = (thisph + ALPHA*(1 - thisqueue/qtotal)) / (1 + ALPHA*(N-1));
			urange += (thisph);
			if(tmp_double >= lrange && tmp_double < urange) {
				//printf("return next %d\n",next);
				return next;
			}
			lrange = urange;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////
/// Method to update routing table
/// Parameters:
/// - destination node address
/// - neighbor node address
/// .
/// This method increments and evaporates pheromone values as per AntNet algorithm
///////////////////////////////////////////////////////////////////////////////////
void antnet_rtable::update(nsaddr_t dest, nsaddr_t next) {
	
	pheromone_matrix *vect_pheromone;
	pheromone_matrix temp;
	
	// read ruoitng table entry for destination
	rtable_t::iterator iterRt = rt_.find(dest);
	if(iterRt != rt_.end()) {
		vect_pheromone = &((*iterRt).second);
		pheromone_matrix::iterator iterPh = vect_pheromone->begin();
		for(; iterPh != vect_pheromone->end(); iterPh++) {
			double oldph = (*iterPh).phvalue;
			if((*iterPh).neighbor == next)
				(*iterPh).phvalue = oldph + r*(1 - oldph); // increase ph value for link travelled by ant
			else
				(*iterPh).phvalue = (1-r)*oldph;	// evaporate pheromone for other links
		}
	}
}
