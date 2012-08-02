
/*
 * ant_pkt.h
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
/// \file ant_pkt.h
/// \brief Defines ANT packet
///
/// This file defines packet structure that represents an ant for AntNet algorithm
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __ant_pkt_h__
#define __ant_pkt_h__

#include <packet.h>
#include <list>
#include "antnet_common.h"

/// Forward ant identifier
#define FORWARD_ANT 0x01
/// Backward ant identifier
#define BACKWARD_ANT 0x02
/// Size of ant packet
#define ANT_SIZE 7
/// Macro to access ant header
#define HDR_ANT_PKT(p) hdr_ant_pkt::access(p)

////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Represents memory of an ant
///
/// This structure represents memory of an ant that stores id and trip time of nodes visited by forward ant.
////////////////////////////////////////////////////////////////////////////////////////////////
struct memory{
	nsaddr_t node_addr;	///< node address
	double trip_time;	///< trip time to node
};

////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Ant packet header
////////////////////////////////////////////////////////////////////////////////////////////////
struct hdr_ant_pkt {
        u_int8_t pkt_type_;	///< Packet Type (forward/backward)
	nsaddr_t pkt_src_;	///< address of source node (which originated the packet)
	nsaddr_t pkt_dst_;	///< address of destination node
	u_int16_t pkt_len_;	///< packet length
	u_int8_t pkt_seq_num_;	///< packet sequence number
	double pkt_start_time_;	///< packet start time
	struct memory pkt_memory_[MAX_NUM_NODES];	///< packet's memory
	int pkt_mem_size_;	///< size of memory
	
	inline nsaddr_t& pkt_src() {return pkt_src_;}
	inline nsaddr_t& pkt_dst() {return pkt_dst_;}
	inline u_int16_t& pkt_len() {return pkt_len_;}
	inline u_int8_t& pkt_seq_num() {return pkt_seq_num_;}
	inline double& pkt_start_time() {return pkt_start_time_;}
	inline int& pkt_mem_size() {return pkt_mem_size_;}
	inline u_int8_t& pkt_type() {return pkt_type_;}
		
	static int offset_;
	inline static int& offset() {return offset_;}
	
	inline static hdr_ant_pkt* access(const Packet *p) {
		return (hdr_ant_pkt*)p->access(offset_);
	}
};

#endif
