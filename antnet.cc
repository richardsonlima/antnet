
/*
 * antnet.cc
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

////////////////////////////////////////////////
/// \file antnet.cc
/// \brief Implementation file for Agent Antnet
////////////////////////////////////////////////

#include "antnet.h"
#include "address.h" // to fascilitate inlines
#include <stdio.h>
#include <stdlib.h>
#include <math.h> 

int hdr_ant_pkt::offset_;	///< to access ant packet header
extern double r;		///< reinforcement factor
extern int N;			///< number of neighbors of a node
extern int NUM_NODES;		///< total number of nodes in the topology

///////////////////////////////////////////////////////////////////////////
/// \brief tcl binding for new packet: Ant
///////////////////////////////////////////////////////////////////////////
static class AntHeaderClass : public PacketHeaderClass {
 	public:
 	AntHeaderClass() : PacketHeaderClass("PacketHeader/Ant",sizeof(hdr_ant_pkt)) {
 		bind_offset(&hdr_ant_pkt::offset_);
 	}
 } class_rtProtoAnt_hdr;

///////////////////////////////////////////////////////////////////////////
/// \brief tcl binding for new Agent: Antnet
///////////////////////////////////////////////////////////////////////////
static class AntnetClass : public TclClass {
	public:
	AntnetClass() : TclClass("Agent/Antnet") {}
	TclObject* create(int argc, const char*const* argv) {
		assert(argc == 5);
		return (new Antnet((nsaddr_t)Address::instance().str2addr(argv[4])));
	}
} class_rtProtoAntnet;
 
///////////////////////////////////////////////////////////////////////////
/// tcl binding for agent parameters
/// default values defined in ns-default.tcl
///////////////////////////////////////////////////////////////////////////
 Antnet::Antnet(nsaddr_t id) : Agent(PT_ANT), ant_timer_(this), dmux_(0) {
	
	bind("num_nodes_", &num_nodes_);	// number of nodes in topology
	bind("num_nodes_x_", &num_nodes_x_);	// number of nodes in row (for regular mesh topology)
	bind("num_nodes_y_", &num_nodes_y_);	// number of nodes in column (for regular mesh topology)
	bind("r_factor_", &r_factor_);		// reinforcement factor
	bind("timer_ant_", &timer_ant_);	// timer for generation of forward ants
	
	ra_addr_ = id;		// agent address 
	ant_seq_num_ = 0;	// initialize sequence number of ant packets to zero
}

/////////////////////////////////////////////////////////////////
/// commands that the agent can handle
/////////////////////////////////////////////////////////////////
int Antnet::command(int argc, const char*const* argv) {
	if (argc == 2) {
		if(strcasecmp(argv[1], "start") == 0) {	// begin AntNet algorithm
			initialize_rtable();	// initialize routing tables
			ant_timer_.resched(0.);	// schedule timer to begin ant generation now
			return TCL_OK;
		}
		else if(strcasecmp(argv[1], "stop") == 0) {	// stop AntNet algorithm
			ant_timer_.cancel();	// cancel any scheduled timers
			return TCL_OK;
		}
		else if (strcasecmp(argv[1], "print_rtable") == 0) {	// print routing tables to a file
			FILE *fp = fopen(file_rtable,"a");	// file name defined in antnet_common.h
			fprintf(fp,"\nRouting table at node %d\n",addr());
			fclose(fp);
			rtable_.print();	// call method to print routing table
			return TCL_OK;
		}
 	}
	else if (argc == 3) {
		// obtain corresponding dmux to carry packets
		if (strcmp(argv[1], "port-dmux") == 0) {
			dmux_ = (PortClassifier*)TclObject::lookup(argv[2]);
			if (dmux_ == 0) {
				fprintf(stderr, "%s: %s lookup of %s failed\n",__FILE__,argv[1],argv[2]);
				return TCL_ERROR;
 			}
			return TCL_OK;
 		}
		// obtain corresponding tracer
		else if (strcmp(argv[1], "log-target") == 0 || strcmp(argv[1], "tracetarget") == 0) {
			logtarget_ = (Trace*)TclObject::lookup(argv[2]);
			if (logtarget_ == 0)
				return TCL_ERROR;
			return TCL_OK;
		}
	}
	// add node1 to neighbor list of node2 and vice-versa (we assume duplex link)
	else if (argc == 4) {
		if(strcmp(argv[1], "add-neighbor") == 0) {
			Node *node1 = (Node*)TclObject::lookup(argv[2]);
			Node *node2 = (Node*)TclObject::lookup(argv[3]);
			add_Neighbor(node1, node2);
			return TCL_OK;
		}
	}
 	// Pass the command to the base class
	return Agent::command(argc, argv);
}

/////////////////////////////////////////////////////////////////
/// Agent recieves ant packets
/////////////////////////////////////////////////////////////////
void Antnet::recv(Packet *p, Handler *h) {
	struct hdr_cmn *ch = HDR_CMN(p);	// common header
	struct hdr_ip *ih = HDR_IP(p);		// ip header
	struct hdr_ant_pkt *ah = HDR_ANT_PKT(p);// ant header
		
	if((int)ih->saddr() == ra_addr()) {
		// If loop, drop the packet
		if (ch->num_forwards() > 0) {
			drop(p, DROP_RTR_ROUTE_LOOP);
		}
		// else if reciever is the source
		else if (ch->num_forwards() == 0){
		
		}
	}
	else {
		// recieved packet is an Ant packet
		if(ch->ptype() == PT_ANT) {
			// call method to handle ant packet
			recv_ant_pkt(p);
		}
		// if not Ant packet, drop
		else {
			drop(p, DROP_RTR_ROUTE_LOOP);
		}
	}
}

/////////////////////////////////////////////////////////////////
/// Method to send a forward ant
/// Called when ant timer expires
/////////////////////////////////////////////////////////////////
void Antnet::send_ant_pkt() {
	nsaddr_t next, dest;
	Packet* p = allocpkt();			// allocate new packet
	struct hdr_cmn* ch = HDR_CMN(p);	// common header
	struct hdr_ip* ih = HDR_IP(p);		// ip header
	struct hdr_ant_pkt* ah = HDR_ANT_PKT(p);// ant header
	
	ah->pkt_type() = FORWARD_ANT;		// set ant type as FORWARD ant
	ah->pkt_src() = addr();			// source address
	ah->pkt_len() = ANT_SIZE;		// length of ant header
	ah->pkt_seq_num() = ant_seq_num_++;	// sequence number
	ah->pkt_start_time() = CURRENT_TIME;	// packet generation time
	dest = rtable_.calc_destination(addr());// generate random destination
	ah->pkt_dst() = dest;			// set packet destination
	ah->pkt_mem_size() = 0;			// initialize size of memory
	ah->pkt_memory_[0].node_addr = addr();	// add source node to memory
	ah->pkt_memory_[0].trip_time = 0.0;	// add trip time to this node to memory
	ah->pkt_mem_size()++;			// increment size of memory
		
	ch->ptype() = PT_ANT;			// set packet type as Ant
	ch->direction() = hdr_cmn::DOWN;	// forward ant
	ch->size() = IP_HDR_LEN + ah->pkt_len();// packet header size
	ch->error() = 0;
	ch->addr_type() = NS_AF_INET;
	// generate next hop as per AntNet algorithm
	next = rtable_.calc_next(addr(), ah->pkt_dst(), addr());
	// if next hop same as this node, release packet
	if(next == addr()) {
		Packet::free(p);
		return;
	}
	ch->next_hop() = next;		// set next hop address in common header
	
	ih->saddr() = addr();		// set source address in ip header
	ih->daddr() = next;		// set destination address in ip header
	ih->ttl() = 2 * (NUM_NODES);	// set time-to-live
	if(DEBUG)
		fprintf(stdout,"sending antnet packet from %d to %d next hop %d\n", ah->pkt_src(), ah->pkt_dst(), ih->daddr());
	
	target_->recv(p);	// send forward ant packet
}

///////////////////////////////////////////////////////////////////
/// Method to recieve Ant packet at the Agent
/// Calls appropriate methods to process forward and backward ants
///////////////////////////////////////////////////////////////////
void Antnet::recv_ant_pkt(Packet* p) {
	struct hdr_ip* ih = HDR_IP(p);		// ip header
	struct hdr_cmn* ch = HDR_CMN(p);	// common header
	struct hdr_ant_pkt* ah = HDR_ANT_PKT(p);// ant header
	
	assert(ih->sport() == RT_PORT);
	assert(ih->dport() == RT_PORT);
	
	if(DEBUG)
		printf("In recv_antnet_pkt() %d at node %d %d source %d dest %d\n", ch->direction(), addr(), ih->daddr(), ah->pkt_src(), ah->pkt_dst());
	
	if(ch->direction() == hdr_cmn::DOWN) {	// forward ant
		if(addr() == ah->pkt_dst()) {	// destination node
			// add this node to memory
			memorize(p);
			// create backward ant
			create_backward_ant_pkt(p);
		}
		else {		// not destination node
			// add this node to memory
			memorize(p);
			// send forward ant to next hop node as determined by AntNet algorithm
			forward_ant_pkt(p);
		}
	}
	else 	if(ch->direction() == hdr_cmn::UP) {	// backward ant
		if(addr() == ah->pkt_dst()) {	// destination node, travel complete
			// update routing table
			update_table(p);
			// release packet
			Packet::free(p);
			return;
		}
		else {		// not destination node
			// update routing table
			update_table(p);
			// send backward ant to next hop node as determined by memory
			backward_ant_pkt(p);
		}
	}
}

///////////////////////////////////////////////////////////////////
/// Method to build meory of forward ant
///////////////////////////////////////////////////////////////////
void Antnet::memorize(Packet* p) {
	struct hdr_ant_pkt* tmp = HDR_ANT_PKT(p);	// ant header
	
	double time = CURRENT_TIME - tmp->pkt_start_time();	// trip time to this node
	
	// If node revisited, there is a loop, remove loop and corresponding memory
	for(int i=0; i<tmp->pkt_mem_size(); i++) {
		if(tmp->pkt_memory_[i].node_addr == addr()) {
			double t = time - tmp->pkt_memory_[i].trip_time;
			tmp->pkt_mem_size() = i+1;
			for(int j=0; j <= i; j++) {
				tmp->pkt_memory_[j].trip_time += t;
			}
			return;
		}
	}
	
	// add current node to memory
	tmp->pkt_memory_[tmp->pkt_mem_size()].node_addr = addr();
	tmp->pkt_memory_[tmp->pkt_mem_size()].trip_time = time;
	tmp->pkt_mem_size() = tmp->pkt_mem_size()+1;

	if(DEBUG) {
		fprintf(stdout,"adding %d to memory of pkt %d\n", addr(), tmp->pkt_seq_num());
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
/// Method to send forward ant packet to next hop node as determined by AntNet algorithm
////////////////////////////////////////////////////////////////////////////////////////
void Antnet::forward_ant_pkt(Packet* p) {
	struct hdr_ip* ih = HDR_IP(p);		// ip header
	struct hdr_cmn* ch = HDR_CMN(p);	// common header
	struct hdr_ant_pkt* ah = HDR_ANT_PKT(p);// ant header

	nsaddr_t parent = ih->saddr();	// parent node
	// find next hop node as per AntNet algorithm
	nsaddr_t next = rtable_.calc_next(addr(), ah->pkt_dst(), parent);
	// if next hop is this node or parent node, dead end, release packet
	if(next == addr() || next == parent) {
		Packet::free(p);
		return;
	}
	
	ch->next_hop() = next;	// set next hop node address in common header
	ih->saddr() = addr();	// set source address in ip header
	ih->daddr() = next;	// set destination address in ip header
	if(DEBUG)
		fprintf(stdout,"forwarding antnet packet from %d source %d dest %d next hop %d\n", addr(), ah->pkt_src(), ah->pkt_dst(), ih->daddr());
	// send packet to next hop node
	target_->recv(p);
}

//////////////////////////////////////////////////////////////////////////
/// Method to create backward ant packet
/// called when forward ant reaches destination node
//////////////////////////////////////////////////////////////////////////
void Antnet::create_backward_ant_pkt(Packet* p) {
	struct hdr_ip* ih = HDR_IP(p);
	struct hdr_cmn* ch = HDR_CMN(p);
	struct hdr_ant_pkt* ah = HDR_ANT_PKT(p);
	
	// swap source and destination address
	nsaddr_t temp = ah->pkt_src();
	ah->pkt_src() = ah->pkt_dst();
	ah->pkt_dst() = temp;
	
	// retrieve last second entry in memory (last entry is this node)
	int index = ah->pkt_mem_size() - 2;
	ch->direction() = hdr_cmn::UP;	// chnge direction to backward Ant
	ch->ptype() = PT_ANT;	// set packet type as Ant
	ch->next_hop() = ah->pkt_memory_[index].node_addr;	// next hop as determined from memory
	ih->saddr() = addr();	// source address
	ih->daddr() = ch->next_hop();	// destination address
	
	if(DEBUG)
		fprintf(stdout,"creating backward antnet packet from %d source %d dest %d next hop %d\n", addr(), ah->pkt_src(), ah->pkt_dst(), ih->daddr());
	// send backward ant packet
	target_->recv(p);
}

//////////////////////////////////////////////////////////////////////////////////
/// Method to send backward ant packet to next hop node as determined from memory
/// called when agent recieves a backward ant
//////////////////////////////////////////////////////////////////////////////////
void Antnet::backward_ant_pkt(Packet* p) {
	struct hdr_ip* ih = HDR_IP(p);
	struct hdr_cmn* ch = HDR_CMN(p);
	struct hdr_ant_pkt* ah = HDR_ANT_PKT(p);
	
	// find node previous to this node in memory
	int index;
	for(int i = ah->pkt_mem_size()-1; i >= 0; i--) {
		if(ah->pkt_memory_[i].node_addr == addr()) {
			index = i-1;
			break;
		}
	}
	// next hop node determined from memory
	ch->next_hop() = ah->pkt_memory_[index].node_addr;
	ch->direction() = hdr_cmn::UP;	// backward ant
	ch->ptype() = PT_ANT;	// packet type = Ant
	ih->saddr() = addr();	// source address
	ih->daddr() = ch->next_hop();	// destination addres
	
	if(DEBUG)
		fprintf(stdout,"forwarding backward antnet packet from %d source %d dest %d next hop %d\n", addr(), ah->pkt_src(), ah->pkt_dst(), ih->daddr());
	// send backward ant to next hop
	target_->recv(p);
}

///////////////////////////////////////////////////////////////
/// Method to return size of observation window
///////////////////////////////////////////////////////////////
int Antnet::get_win_size(nsaddr_t dest) {
	int count = 0;
	window_t::iterator iterWin = window_.find(dest);
	triptime_t win_tt = (*iterWin).second;
	triptime_t::iterator itertt;
	for(itertt = win_tt.begin(); itertt != win_tt.end(); itertt++) {
		count++;
	}
	return count;
}

////////////////////////////////////////////////////////////////////////////
/// Method to update traffic model and calculate reinforcement factor (r).
/// Presently, constant value of r is used.
/// Value of r can be set form tcl script.
/// Hence, traffic model is not used and this method is not called.
///////////////////////////////////////////////////////////////////////////
void Antnet::update_traffic(Packet* p) {
	//update mean, variance, best.
	struct traffic_matrix temp_traffic;
	nsaddr_t dest, next;
	double tt, oldtt;
	double oldvar;
	double varsigma = VARSIGMA;
	
	struct hdr_ant_pkt* ah = HDR_ANT_PKT(p);
	int i;
	for(i=0; ah->pkt_memory_[i].node_addr != addr(); i++);
	double initialtt = ah->pkt_memory_[i].trip_time;
	i++;
	next = ah->pkt_memory_[i].node_addr;
	
	for(int index = i; index < ah->pkt_mem_size(); index++) {
	
				
		dest = ah->pkt_memory_[index].node_addr;
		tt = ah->pkt_memory_[index].trip_time - initialtt;
		
		/* update sample window */
		window_t::iterator iterWin = window_.find(dest);
		if(iterWin != window_.end()) {	// destination entry exists, add to it in window
			(*iterWin).second.push_back(tt);
		}
		else {	// destination entry does not exist, add new dest entry to window
			triptime_t win_tt;
			win_tt.push_back(tt);
			window_[dest] = win_tt;
		}
	}
	
	/* update traffic */
	for(int index = i; index < ah->pkt_mem_size(); index++) {
		
		dest = ah->pkt_memory_[index].node_addr;
		tt = ah->pkt_memory_[index].trip_time - initialtt;

		/* find best trip time from this node to dest */
		window_t::iterator iterWin = window_.find(dest);
		triptime_t win_tt = (*iterWin).second;
		triptime_t::iterator itertt = win_tt.begin();
		double mintt = (*itertt);
		for(; itertt != win_tt.end(); itertt++) {
			if((*itertt) < mintt)
				mintt = (*itertt);
		}
		
		/* update traffic */
		state_t::iterator iterFind = state_.find(dest);
		if(iterFind != state_.end()) {
			// update existing entry
			oldtt = (*iterFind).second.mean_tt;
			(*iterFind).second.mean_tt = oldtt + varsigma * (tt - oldtt);
			oldvar = (*iterFind).second.var_tt;
			(*iterFind).second.var_tt = oldvar*oldvar + varsigma * ((tt - oldtt)*(tt - oldtt) -  oldvar*oldvar);
			(*iterFind).second.best_tt = mintt;
		}
		else {
			// add map entry
			temp_traffic.mean_tt = tt;
			temp_traffic.var_tt = tt;
			temp_traffic.best_tt = mintt;
			state_[dest] = temp_traffic;
		}
	}
	
	/* find r and update pheromone */
	for(int index = i; index < ah->pkt_mem_size(); index++) {
	
		dest = ah->pkt_memory_[index].node_addr;
		tt = ah->pkt_memory_[index].trip_time - initialtt;
		
	
		/* find r */
		double W_best = state_[dest].best_tt;
		double I_inf = W_best;
		double mu = state_[dest].mean_tt;
		double sigma = sqrt(state_[dest].var_tt);
		int w = get_win_size(dest);
		double I_sup = mu + zee * (sigma/sqrt(w));
		if(I_sup == I_inf && I_inf == tt)
			r = 0.0;
		else
			r = c1*(W_best/tt) + c2 * ((I_sup - I_inf) / ((I_sup - I_inf) + (tt - I_inf) ));
		
		if(DEBUG) {
			printf("r = %f\n", r);
		}
	}
}

//////////////////////////////////////////////////////////
/// Method to update routing table
//////////////////////////////////////////////////////////
void Antnet::update_table(Packet* p) {
	
	nsaddr_t dest, next;
		
	struct hdr_ant_pkt* ah = HDR_ANT_PKT(p);	// ant header
	
	// read node visited next to this node from memory
	// this is the nieghbor node for which routing table will be updated
	int i;
	for(i=0; ah->pkt_memory_[i].node_addr != addr(); i++);
	i++;
	next = ah->pkt_memory_[i].node_addr;
	
	if(DEBUG) {
		fprintf(stdout,"updating ph at %d\n", addr());
		fprintf(stdout,"next: %d\n",next);
	}
		
	nsaddr_t node_addr = addr();
	N = get_num_neighbors(node_addr);
	
	// routing table is updated for all the destination nodes that are visited after the neighbor node
	// update pheromone value corresponding to neighbor node and destination nodes visited thereafter
	for(int index = i; index < ah->pkt_mem_size(); index++) {
		// read destination nodef rom memory
		dest = ah->pkt_memory_[index].node_addr;
		// update pheromone valu fro neighbor node and this destination node
		rtable_.update(dest, next);		
	}
}

//////////////////////////////////////////////////////////
/// Method to initialize routing table
//////////////////////////////////////////////////////////
void Antnet::initialize_rtable() {
	//NUM_NODES = num_nodes_x_ * num_nodes_y_;
	NUM_NODES = num_nodes_;		// set number of nodes in topology (read from tcl script)
	r = r_factor_;	// set reinforcement factor (read from tcl script)
	nsaddr_t node_addr = addr();
	int num_nb = get_num_neighbors(node_addr);
	Node *nd = nd->get_node_by_address(addr());
	// add destination entry for each node in topology
	for(int i = 0; i < NUM_NODES; i++) {
		if(addr() != i) {
			// read list of neighbors
			neighbor_list_node* nb = nd->neighbor_list_;
			while(nb != NULL) {
				// read node id of neighbor node
				int neighb = nb->nodeid;
				// initialize equal pheromone value to all neighbor links
				double phvalue = 1.0/num_nb;
				// add routing table entry
				rtable_.add_entry(i, neighb, phvalue);
				// iterate in neighbor list
				nb = nb->next;
			}
		}
	}
	FILE *fp = fopen(file_rtable,"w");
	fclose(fp);
}

//////////////////////////////////////////////////////////
/// Method to print neighbors of a node
//////////////////////////////////////////////////////////
void
Antnet::print_neighbors() {
	nsaddr_t node_addr = addr();
	fprintf(stdout,"addr: %d\tra_addr:%d\n",addr(), ra_addr());
	Node *n = n->get_node_by_address(node_addr);
	fprintf(stdout,"node id: %d\tnode address: %d\n", n->nodeid(), n->address());
	fprintf(stdout,"Neighbors:\n");
	neighbor_list_node* nb = n->neighbor_list_;
	do {
		int neigh = nb->nodeid;
		printf("%d\n",neigh);
		nb = nb->next;
	}while(nb != NULL);
}

//////////////////////////////////////////////////////////
/// Method to add neighbors of a node
/// Parameters: addresses of two neighbor nodes (n1, n2)
/// We assume duplex links
/// - Add n1 to neighbor list of n2
/// - Add n2 to neighbor list of n1
//////////////////////////////////////////////////////////
void
Antnet::add_Neighbor(Node *n1, Node *n2) {
	n1->addNeighbor(n2);
	n2->addNeighbor(n1);
}

//////////////////////////////////////////////////////////
/// Method to reset Ant timer
//////////////////////////////////////////////////////////
void Antnet::reset_ant_timer() {
	ant_timer_.resched(timer_ant_);
}

//////////////////////////////////////////////////////////
/// Method to handle Ant timer expire event
//////////////////////////////////////////////////////////
void Ant_timer::expire(Event *e) {
	// generate forward ant
	agent_->send_ant_pkt();
	// reschedule timer
	agent_->reset_ant_timer();
}
