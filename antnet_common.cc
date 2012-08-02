
/*
 * antnet_common.cc
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

////////////////////////////////////////////////////////////////////////////////////////
/// \file antnet_common.cc
/// \brief Implementation file for globally available methods
////////////////////////////////////////////////////////////////////////////////////////

#include "antnet_common.h"

//////////////////////////////////////////////////////
/// Method to return number of neighbors of a node
//////////////////////////////////////////////////////
int
get_num_neighbors(nsaddr_t node_addr) {
	int count = 0;
	Node *nd = nd->get_node_by_address(node_addr);
	neighbor_list_node* nb = nd->neighbor_list_;
	while(nb != NULL) {
		count ++;
		nb = nb->next;
	}	
	return count;
}

//////////////////////////////////////////////////////////////
/// Method to return queue length of link between two nodes
/////////////////////////////////////////////////////////////
int
get_queue_length(Node *node1, Node *node2) {
	Tcl& tcl = Tcl::instance();
	// get-drop-queue method implemented in ns-lib.tcl
	tcl.evalf("[Simulator instance] get-drop-queue %d %d", node1->nodeid(), node2->nodeid());
	DropTail *qa = (DropTail*)TclObject::lookup(tcl.result());
	int len = qa->getlength();
	return len;
}
