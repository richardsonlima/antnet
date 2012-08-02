
/*
 * antnet_rtable.h
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
/// \file antnet_rtable.h
/// \brief Definition file for routing table in AntNet
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __antnet_rtable_h__
#define __antnet_rtable_h__

#include <trace.h>
#include <map>
#include <string>
#include <vector>
#include <classifier-port.h>
#include <random.h>

#include "ant_pkt.h"
#include "antnet_common.h"

////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Represents an entry in routing table
///
/// This structure represents pheromone value corresponding to a neighbor node
////////////////////////////////////////////////////////////////////////////////////////////////
struct pheromone {
	nsaddr_t neighbor;	///< address of neighbor node
	double phvalue;		///< pheromone value
};

/// vector of pheromone values (represents entry in routing table corresponding to a destination)
typedef std::vector<struct pheromone> pheromone_matrix;
/// Routing table
typedef std::map<nsaddr_t, pheromone_matrix> rtable_t;
/// Vector of neighbors all having same and maximum pheromone value
typedef std::vector<nsaddr_t> sameph_t;

/////////////////////////////////////////////////////////////
/// \brief Class to implement routing table
/////////////////////////////////////////////////////////////
class antnet_rtable {
	rtable_t rt_;	///< routing table
	
	RNG *rnum;	///< random number generator
	public:

		/// Constructor
		antnet_rtable() {
			rnum = new RNG((long int)CURRENT_TIME);
		}
		
		/// Method to add an entry in routing table
		// Parameters: destination node, neighbor node, pheromone value
		void add_entry(nsaddr_t destination, nsaddr_t neighbor, double phvalue);
		/// Method to print routing table
		void print();
		/// returns destination node for given source node
		nsaddr_t calc_destination(nsaddr_t source);
		/// returns next hop node for given source destination pair
		// Parameters: source node, destination node, parent node
		nsaddr_t calc_next(nsaddr_t source, nsaddr_t destination, nsaddr_t parent);
		/// updates an entry in routing table
		// Parameters: destination node, neighbor node
		void update(nsaddr_t destination, nsaddr_t neighbor);
};

#endif
