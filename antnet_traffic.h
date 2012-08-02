
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA.
 *
 * Author: Lavina Jain
 *
 */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \file antnet_rtable.h
/// \brief Defination file for routing table in AntNet
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __antnet_traffic_h__
#define __antnet_traffic_h__

#include <trace.h>
#include <map>
#include <string>
#include <vector>
#include <classifier-port.h>
#include <random.h>

typedef std::vector<double> triptime_t;
typedef std::map<nsaddr_t, triptime_t> window_t;

struct traffic_matrix {
	double mean_tt;
	double var_tt;
	double best_tt;
};
typedef std::map<nsaddr_t, struct traffic_matrix> state_t;

#endif
