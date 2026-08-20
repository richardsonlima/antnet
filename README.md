# AntNet Algorithm for The Network Simulator (NS-2)

![License](https://img.shields.io/badge/license-GPL--2.0-blue.svg)
![NS-2](https://img.shields.io/badge/ns--2-2.34-informational.svg)
![Language](https://img.shields.io/badge/language-C%2B%2B-orange.svg)
![Unit tests](https://img.shields.io/badge/antnet__math%20unit%20tests-497%2F497%20passing-brightgreen.svg)
![Cited](https://img.shields.io/badge/cited%20in-ACM%20ICFNDS%20'18-lightgrey.svg)
![Derivative work](https://img.shields.io/badge/derivative%20work-SFU%20ENSC427%20(2011)-lightgrey.svg)

An **AntNet** routing agent for **ns-2.34**, a faithful, actively-maintained C++ port of Di Caro & Dorigo's Ant Colony Optimization (ACO) routing algorithm, packaged as a drop-in patch for the ns-2 network simulator.

> AntNet is a distributed, adaptive, mobile-agent-based algorithm for best-effort routing in packet-switched networks, in which colonies of software "ants" concurrently explore the network and lay down pheromone trails proportional to path quality, the same stigmergic principle real ant colonies use to find shortest paths to food sources.

## Table of Contents

- [Overview](#overview)
- [Background: AntNet and Ant Colony Optimization](#background-antnet-and-ant-colony-optimization)
- [Provenance and Authorship](#provenance-and-authorship)
- [Cited By](#cited-by)
- [Repository Layout](#repository-layout)
- [Requirements](#requirements)
- [Installation](#installation)
- [Configuration](#configuration)
- [Manual Integration Guide (non-destructive install)](#manual-integration-guide-non-destructive-install)
- [Testing](#testing)
- [Engineering Notes / Code Review Log](#engineering-notes--code-review-log)
- [Known Limitations](#known-limitations)
- [Related Work and Further Reading](#related-work-and-further-reading)
- [How to Cite This Software](#how-to-cite-this-software)
- [License](#license)
- [Acknowledgments](#acknowledgments)

## Overview

This repository implements AntNet as an `Agent/Antnet` object for **ns-2.34**. Each node runs an `Antnet` agent that:

1. Periodically launches a **forward ant** toward a pseudo-random destination (`Antnet::send_ant_pkt`).
2. Lets that ant hop stochastically toward its destination, biased by pheromone concentration and local queue-length feedback (`antnet_rtable::calc_next`), recording the identity and trip time of every node it visits (`Antnet::memorize`).
3. Turns the forward ant around into a **backward ant** at the destination (`Antnet::create_backward_ant_pkt`), which retraces the exact forward path and reinforces the pheromone table at every hop it revisits (`Antnet::update_table`, `antnet_rtable::update`).

The result is a routing table that continuously adapts to congestion and topology changes without any centralized coordination, the network itself is the ants' shared memory (*stigmergy*).

The pure numeric core of the algorithm (pheromone reinforcement/evaporation and the AntNet link-selection probability formula) lives in [`antnet_math.h`](antnet_math.h), independent of any ns-2 type, and is exercised by a standalone unit test suite (see [Testing](#testing)).

## Background: AntNet and Ant Colony Optimization

AntNet was introduced by **Gianni Di Caro and Marco Dorigo** as an application of the Ant Colony Optimization metaphor to adaptive routing in communications networks:

> G. Di Caro and M. Dorigo, ["AntNet: Distributed Stigmergetic Control for Communications Networks"](https://arxiv.org/abs/1105.5449), *Journal of Artificial Intelligence Research*, vol. 9, pp. 317–365, 1998.

Di Caro's own reference material on the algorithm and its follow-ups (AntHocNet for MANETs, Ant Colony System, Swarm Intelligence in general) is maintained at his personal research page:

> Gianni A. Di Caro, [*AntNet*](http://www.giannidicaro.com/antnet.html), IDSIA, Dalle Molle Institute for Artificial Intelligence.

Di Caro's page also lists AntNet's known software implementations for popular network simulators, and speaks directly to this codebase's lineage, in his own words:

> "Lavina Jain made an implementation of AntNet for NS-2 [...] Richardson Lima has also released, more recently, a version of AntNet for NS-2. He also made an AntNet web page. Since I'm not an NS-2 user, I haven't checked these implementations, but I guess the Lima's implementation can be used as a good starting point."
> Gianni A. Di Caro, [*AntNet*: Software implementations of AntNet](http://www.giannidicaro.com/antnet.html)

His page also lists two further primary references beyond the 1998 JAIR paper, for readers who want the full theoretical treatment:

> G. A. Di Caro, *Ant Colony Optimization and its application to adaptive routing in telecommunication networks*, PhD thesis, Applied Sciences, Polytechnic School, Université Libre de Bruxelles, Brussels, Belgium, 2004.
>
> G. A. Di Caro and M. Dorigo, *"Two Ant Colony Algorithms for Best-Effort Routing in Datagram Networks,"* Proceedings of PDCS'98, 10th International Conference on Parallel and Distributed Computing and Systems, Las Vegas, Nevada, Oct. 28–31, 1998 (also Technical Report IRIDIA 98-09).

AntNet's core mechanics, **forward ants** that sample paths and time trip lengths, **backward ants** that retrace the exact same path to reinforce good routes and let others evaporate, and a routing table expressed as per-destination pheromone probability distributions over neighbors, are exactly what `antnet.cc`/`antnet_rtable.cc` implement here for ns-2's discrete-event simulation model.

**New to the underlying biology?** BBC Future's ["What Insects Can Teach Us About Decision-Making"](https://www.bbc.com/future/article/20260810-what-insects-can-teach-us-about-decision-making) (Aug. 2026) is a general-audience explainer on the collective decision-making behaviors in insect colonies, the same biological phenomenon (stigmergy, pheromone-mediated consensus) that AntNet formalizes into a routing algorithm. A gentler starting point than the papers above if you're arriving from outside networking/AI.

## Provenance and Authorship

This codebase has a documented lineage across three phases of academic/open-source work:

1. **Original ns-2 port (ns-2.28)** by **Lavina Jain**, building on the AntNet algorithm for the paper *"Ant Colony Optimisation Based Routing on NS-2"* (V. Laxmi, L. Jain, M. S. Gaur, WCSN, India, December 2006), whose citation request is preserved verbatim in the header of every `antnet.*` / `antnet_rtable.*` / `antnet_common.*` source file in this repository.
2. **Port and behavioral alignment to ns-2.33**, by **Richardson Lima** (contato@richardsonlima.com.br), who adapted Jain's implementation so its routing behavior matches Di Caro's original AntNet proposal, and documented the port and its usage at [antnetalgorithm.blogspot.com](http://antnetalgorithm.blogspot.com.br) and [antnet.wordpress.com](http://antnet.wordpress.com) starting in 2009, the primary public reference for running AntNet inside ns-2 that predates this repository.
3. **Port to ns-2.34** in this repository, with suggestions and improvements from **Matthew Orlinski** (University of Manchester, orlinskm@cs.man.ac.uk) and Richardson Lima, plus a **2026 correctness/security review pass** (see [Engineering Notes](#engineering-notes--code-review-log)) that fixed several memory-safety and algorithmic-correctness bugs inherited from the earlier ports, and added the first automated test coverage the project has had (`antnet_math.h` + `tests/test_antnet_math.cc`).

If you build on this repository, please cite it using the entry in [How to Cite This Software](#how-to-cite-this-software), and consider citing the original AntNet paper above for the algorithm itself.

## Cited By

### Peer-reviewed citation and use

> I. U. Khan (Isra University), I. M. Qureshi (Air University), M. A. Khan (Hamdard University), S. B. H. Shah (Dalian University of Technology), I. S. Ansari (Global College of Engineering and Technology), and M. H. Akhlaq (Isra University), ["Ant Colony based Optimization Technique to Mitigate Dynamic Routing Challenges in Packet based Networks,"](https://doi.org/10.1145/3231053.3231117) *ICFNDS '18: Proceedings of the 2nd International Conference on Future Networks and Distributed Systems*, Amman, Jordan, June 26–27, 2018, Article No. 51, pp. 1–5, ACM. ISBN 978-1-4503-6428-7/18/06.

This isn't just a bibliography entry, the authors state they ran this codebase in NS2 as their ACO/AntNet baseline in a comparative simulation study against Distance Vector, Link State, and Session routing (packet drop rate, bandwidth utilization, end-to-end delay, and throughput, on a 25-router WAN topology with scripted link/router failures):

> "We used antnet package [12] in NS2 to simulate the routing scenario for various comparisons. AntNet's package is based on Ant Colony Optimization (ACO) algorithm, which utilizes the mechanisms behind the shortest path route selection conduct observed in ant colonies [...]"
>
> Reference [12] in that paper: *"Richardson Lima. 2012. AntNet Algorithm - Ant-based routing in networks. https://github.com/richardsonlima/antnet."*

### Peer-reviewed comparative study

> S. Sharma and V. Juneja, ["Improved Ad-hoc Network Using Ant based Protocol and Threshold Signal,"](https://www.ciitresearch.org/dl/index.php/wc/article/view/WC122011009) *CiiT International Journal of Wireless Communication*, vol. 3, no. 16, 2011.

This paper compares a signal-strength-aware routing proposal against AODV and against Lima's AntNet as the ACO baseline, both simulated in NS2:

> "The Antnet algorithm [1] without using signal strength criteria, has been found yield throughput of 2.05 MBPS, whereas proposed algorithm results throughput of 2.2 MBPS which is improved by around 7%. [...] All above said routing algorithms are simulated in to NS2 and their performance is calculated on basis of three parameters [...] End to End Delay, Throughput of system and Packet Delivery Fraction (PDF)."
>
> Reference [1] in that paper: *"Richardson Lima, 'AntNet Algorithm Implementation on NS-2.33 antnet', Networking and Telecommunications Research Group, Computer Science Centre of the Federal University of Pernambuco at Recife, Brazil. Friday, February 27, 2009."*

### PhD thesis

> M. R. Jabbarpour Sattari, [*Ant-based Vehicle Congestion Avoidance Framework using Vehicular Networks*](https://knova.um.edu.my/cgi/viewcontent.cgi?article=7477&context=student_works_2010s), PhD thesis, Faculty of Computer Science and Information Technology, University of Malaya, Kuala Lumpur, 2015.

This thesis (whose findings were also published as a 2014 *Engineering Applications of Artificial Intelligence* paper) used Lima's AntNet documentation as the on-ramp for implementing its own AVCAF framework on top of NS-2.33's existing AntNet support:

> "After successful installation of NS-2.33 and in order to become familiar with ant-based algorithms' simulation and implementation in NS-2, we have implemented AntNet algorithm on NS-2 using manual documentation prepared by Lima (2009). After successful installation and running the AntNet algorithm on NS-2, various setups and modifications have been applied in order to implement and simulate AVCAF in simulator environment."
>
> Reference in that thesis: *"Lima, Richardson. (2009). AntNet algorithm implementation on NS-2.33. Computer Science Center of the Federal University of Pernambuco at Recife, Brazil."*

### Derivative academic work

A Simon Fraser University communications-networks course project used this codebase directly as its implementation base, not just as a citation:

> K.-Y. Lee and J. Qiao, ["Improved Ant Routing for Wireless Ad-Hoc Mesh Networks,"](https://www.sfu.ca/~ljilja/ENSC427/Spring11/Projects/team10/ENSC427_Group10_Final_Report.pdf) ENSC 427 (Communication Networks) final project report, Simon Fraser University, Spring 2011 (presented as ["Viability Analysis of GPS-Assisted Ant Routing Algorithm,"](https://www.sfu.ca/~ljilja/ENSC427/Spring11/Projects/team10/ENSC427_Group10_Final_Presentation.pdf) final presentation, Group #10).

Their report states outright that *"the original Antnet code developed by Lavina Jain was used as a starting base"* and credits *"Antnet, first developed by Lavina Jain and later updated by Richardson Lima"* (citing, as their reference [1]/[7], Lima's Sep. 2009 `antnet.wordpress.com` release post). Their Appendix I reproduces modified excerpts of `ns2.34antnet/antnet_rtable.cc` and `common/node.cc` from this codebase's lineage, adding GPS-assisted next-hop selection on top of `calc_next()`.

Notably, their reproduced `calc_next()` excerpt still shows the exact discrepancy documented and fixed in this repository's [2026-08-19 review](#engineering-notes--code-review-log): it computes `thisprob` (pheromone combined with the queue-length heuristic) but then partitions the next-hop probability range using the raw pheromone `thisph` instead, independent, external confirmation that the bug was real, present in the version circulated since at least 2011, and not something introduced by this repository.

## Repository Layout

| Path | Role |
|---|---|
| `antnet.h` / `antnet.cc` | The `Agent/Antnet` ns-2 agent: Tcl bindings, ant packet send/receive/forward state machine, routing-table initialization. |
| `antnet_rtable.h` / `antnet_rtable.cc` | Per-node routing table: pheromone storage, stochastic next-hop selection (`calc_next`), pheromone reinforcement/evaporation (`update`). |
| `antnet_math.h` | Pure, ns-2-independent AntNet math (pheromone update, link-selection probability). Unit-tested in isolation. |
| `antnet_common.h` / `antnet_common.cc` | Shared constants (`ALPHA`, `MAX_NUM_NODES`, ...) and small ns-2 topology helpers (`get_num_neighbors`, `get_queue_length`). |
| `ant_pkt.h` | The `hdr_ant_pkt` wire format for forward/backward ant packets, including the fixed-size per-ant memory buffer. |
| `antnet_traffic.h` | Data structures for the (currently unused, see [Known Limitations](#known-limitations)) adaptive traffic model. |
| `queue/`, `trace/`, `common/` | Vendored ns-2.34 core files, patched with the minimal set of AntNet integration points (`PT_ANT` packet type, `DropTail::getlength()`, `CMUTrace::format_antnet()`, priority-queue handling for ant packets). |
| `tcl/lib/` | Vendored ns-2.34 Tcl library, patched with AntNet's default parameters and the `get-drop-queue` helper the agent needs. |
| `tests/` | Standalone unit tests for `antnet_math.h` (no ns-2 dependency, see [Testing](#testing)). |

## Requirements

- A full **ns-2.34** source tree (not included in this repository).
- **GCC 4.0** (as required by ns-2.34 itself).
- OTcl / TclCL, as required by any ns-2.34 build.
- A C++11-capable compiler is only needed for `tests/` (the standalone math unit tests); the ns-2 agent code itself targets ns-2.34's C++03-era build.

## Installation

> **Warning:** if you have already modified your ns-2.34 tree (e.g. to add other routing protocols), copying these files over it **may overwrite your changes**. If that's your situation, use the [Manual Integration Guide](#manual-integration-guide-non-destructive-install) below instead of a blind copy.

1. Copy the files in this repository into your ns-2.34 source directory, preserving their relative paths (`antnet_*` files at the ns-2 root, `queue/`, `trace/`, `common/`, `tcl/lib/` merged into the corresponding ns-2 directories).
2. Rebuild ns-2:

   ```shell
   ./configure && make
   ```

3. Reference `Agent/Antnet` from your Tcl simulation script like any other ns-2 routing agent.

## Configuration

`Agent/Antnet` exposes the following Tcl-bindable parameters (defaults shipped in `tcl/lib/ns-default.tcl`):

| Parameter | Default | Meaning |
|---|---|---|
| `num_nodes_` | `16` | Total number of nodes in the topology. Must not exceed `MAX_NUM_NODES` (16, compile-time constant in `antnet_common.h`), see [Known Limitations](#known-limitations). |
| `num_nodes_x_` / `num_nodes_y_` | `4` / `4` | Grid dimensions, for regular mesh topologies. |
| `r_factor_` | `0.001` | Reinforcement factor applied to the pheromone table on every backward-ant update. |
| `timer_ant_` | `0.03` | Interval (seconds) between successive forward-ant generations at a node. |

Start/stop the algorithm per node from Tcl with `$agent start` / `$agent stop`, and dump a node's routing table to `rtable.txt` with `$agent print_rtable`.

## Manual Integration Guide (non-destructive install)

This is a guide for merging AntNet's ns-2 core changes into a tree you've already customized, it is **not a complete instruction set**, but combined with the general guide at [elmurod.net](http://elmurod.net/wps/?p=157) for adding a new ns-2 routing protocol (substitute their example protocol name `wfrp` for `antnet`), it covers the AntNet-specific integration points below.

**`queue/drop-tail.h`**: add to the `DropTail` class:

```c++
public: int getlength();
```

**`queue/drop-tail.cc`**: add the corresponding method:

```c++
int DropTail::getlength() {
    return q_->length();
}
```

**`queue/priqueue.cc`**: add ant packets to the high-priority path:

```c++
case PT_ANT:
    if (ch->direction() == hdr_cmn::UP) {
        recvHighPriority(p, h);
    }
    else {
        Queue::recv(p, h);
    }
    break;
```

**`common/packet.h`**: register the packet type name (anywhere the WFRP tutorial above says to put `PT_WFRP`, put `PT_ANT` instead):

```c++
name_[PT_ANT] = "Ant";
```

**`trace/cmu-trace.cc`**: include the AntNet packet header:

```c++
#include <antnet/ant_pkt.h>
```

and add the trace formatter (ported from the ns-2.33 version by Richardson Lima):

```c++
void
CMUTrace::format_antnet(Packet *p, int offset)
{
    struct hdr_ant_pkt *ah = HDR_ANT_PKT(p);

    if (pt_->tagged()) {
        sprintf(pt_->buffer() + offset,
            "-ant:o %d -ant:s %d -ant:l %d ",
            ah->pkt_src(), ah->pkt_seq_num(), ah->pkt_len());
    }
    else if (newtrace_) {
        sprintf(pt_->buffer() + offset,
            "-P ant -Po %d -Ps %d -Pl %d ",
            ah->pkt_src(), ah->pkt_seq_num(), ah->pkt_len());
    }
    else {
        sprintf(pt_->buffer() + offset,
            "[ant %d %d %d] ",
            ah->pkt_src(), ah->pkt_seq_num(), ah->pkt_len());
    }
}
```

and dispatch to it from `CMUTrace::format(...)`:

```c++
case PT_ANT:
    format_antnet(p, offset);
    break;
```

**`tcl/lib/ns-default.tcl`**: add AntNet's default parameters:

```tcl
# Defaults for Antnet
Agent/Antnet set num_nodes_x_ 4
Agent/Antnet set num_nodes_y_ 4
Agent/Antnet set num_nodes_ 16
Agent/Antnet set r_factor_ 0.001
Agent/Antnet set timer_ant_ 0.03
```

**`tcl/lib/ns-lib.tcl`**: add the queue-length helper the agent's `get_queue_length()` calls into:

```tcl
Simulator instproc get-drop-queue { n1 n2 } {
    $self instvar link_
    set q [$link_($n1:$n2) queue]
    return $q
}
```

## Testing

`antnet_math.h` holds the pure AntNet math (pheromone reinforcement/evaporation, link-selection probability) with **no ns-2 dependency**, so it can be unit tested standalone without an ns-2 source tree:

```shell
cd tests && make test
```

This currently exercises 497 assertions covering reinforcement/evaporation bounds, normalization of the pheromone distribution after an update, and the link-selection probability formula (including the queue-length heuristic that a previous version of `calc_next()` silently discarded, see below).

The rest of this repository (`antnet.cc`, `antnet_rtable.cc`, the `queue/`/`trace/`/`tcl/` patches, ...) cannot be compiled or run in isolation, it's an add-on that expects to live inside a full ns-2.34 source tree (`agent.h`, `node.h`, `packet.h`, OTcl/TclCL, etc. aren't part of this repository). Validating those files requires copying them into an ns-2.34 tree per the instructions above and running a simulation there.

## Engineering Notes / Code Review Log

### 2026-08-19: Correctness and memory-safety review

The AntNet-specific files (`antnet.h/.cc`, `antnet_common.h/.cc`, `antnet_rtable.h/.cc`, `ant_pkt.h`, `antnet_traffic.h`) were reviewed and the following bugs were fixed:

- **`calc_next()` silently ignored its own probability formula.** It computed the correct AntNet selection probability (pheromone + heuristic bias towards shorter queues, weighted by `ALPHA`) into a local `thisprob`, but then partitioned the random-selection range using the raw pheromone value instead, so the queue-length heuristic never actually influenced routing decisions. `calc_next()` now uses the computed probability, and the formula itself was extracted into dependency-free helpers in `antnet_math.h` (see [Testing](#testing)).
- **Missing `return` in `calc_next()`** when a destination had no routing-table entry, which is undefined behavior in C++. It now falls back to returning the parent node.
- **Cross-node global-state bug.** `N` (neighbor count) and `r` (reinforcement factor) were plain global variables read by `antnet_rtable::calc_next()`/`update()`. Since every node's `Antnet` agent runs in the same process in ns-2's single-threaded event loop, one node's routing decision could silently use another node's neighbor count or reinforcement factor. Both are now passed in explicitly by the caller instead.
- **Ant memory buffer overflow.** `Antnet::memorize()` appended to a fixed-size `pkt_memory_[MAX_NUM_NODES]` array with no bounds check; a path longer than `MAX_NUM_NODES` hops overflowed it. It now drops the packet instead, and callers were updated to stop touching the packet when that happens (it was previously used-after-free in that path too).
- **Null-pointer dereference** in `print_neighbors()` for a node with no neighbors (a `do`/`while` dereferenced before checking).
- **Uninitialized index** used to access `pkt_memory_[]` in `backward_ant_pkt()` if the current node wasn't found in the ant's memory; now guarded.
- **Uninitialized-pointer idiom** (`Node *nd = nd->get_node_by_address(...)`) used in a few places to reach what is effectively a static lookup; replaced with `Node::get_node_by_address(...)`.
- **Division by zero** in `initialize_rtable()` for a node with zero neighbors (`1.0/num_nb`).
- **Memory leak**: `antnet_rtable`'s RNG was `new`'d with no matching `delete`; it now has a destructor.
- Reduced `calc_next()` from up to 3 Tcl round-trips per neighbor (`get_queue_length()`) down to 1.

### 2026-08-20: Follow-up review

An independent second pass on the already-reviewed files found and fixed three further issues:

- **Correlated RNG streams across nodes.** `antnet_rtable`'s random-number generator was seeded with `(long int)CURRENT_TIME` alone. Since every node's `Antnet` agent (and its `antnet_rtable` member) is normally constructed during Tcl topology setup (i.e. at simulation time `0` for every node), this gave *every node the same RNG seed*, correlating `calc_destination()`'s random-destination draws and `calc_next()`'s stochastic next-hop draws across the whole topology instead of making them independent per node. The seed is now `CURRENT_TIME + node_address + 1`, so each node's stream is distinct.
- **Unguarded map lookup in `get_win_size()`.** It dereferenced `window_.find(dest)` without checking for `window_.end()`, which is undefined behavior if `dest` has no entry yet. Only reachable through `update_traffic()` (see [Known Limitations](#known-limitations), currently dead code), so latent rather than active, but fixed so it can't resurface if that code path is ever re-enabled.
- **No boundary check between `MAX_NUM_NODES` (compile-time, 16) and `num_nodes_` (Tcl-configurable).** Nothing previously stopped a topology larger than `MAX_NUM_NODES` from being configured; the only symptom would be ants silently dropped once a path exceeded 16 hops (via the buffer-overflow guard added in the 2026-08-19 pass), with no indication of why. `initialize_rtable()` now logs a clear warning when this misconfiguration is detected.

`update_traffic()` (the traffic-model/adaptive-`r` method) remains dead code (it's never called, per the original author's own comment), and was left otherwise alone beyond keeping it compiling and fixing the `get_win_size()` bug it depends on.

## Known Limitations

- **`MAX_NUM_NODES` is a hard, compile-time cap (16)** on both topology size and ant-memory depth. Larger topologies require bumping `MAX_NUM_NODES` in `antnet_common.h` and recompiling; `initialize_rtable()` now warns if `num_nodes_` exceeds it (see above).
- **`update_traffic()` and the adaptive-`r` traffic model are unused.** `r_factor_` is a fixed, Tcl-configured constant instead; the traffic-model code path (`state_`, `window_`, `get_win_size()`) is compiled but never invoked.
- **No automated integration tests.** Only the ns-2-independent routing math in `antnet_math.h` has unit test coverage (see [Testing](#testing)); the ns-2 agent itself can only be validated by running a full ns-2.34 simulation, since this repository intentionally does not vendor the rest of ns-2.
- **Single-process global assumptions.** `NUM_NODES` is still a plain global (all nodes agree on the same topology size, so this is safe), but it is a reminder that this codebase assumes ns-2's single-threaded, single-process simulation model throughout.

## Related Work and Further Reading

The list below situates this implementation in the broader AntNet / ACO-routing literature, as general background reading on the algorithm and its theoretical foundations. For works that specifically cite or build on *this repository*, see [Cited By](#cited-by) instead.

1. G. Di Caro and M. Dorigo, ["AntNet: Distributed Stigmergetic Control for Communications Networks"](https://arxiv.org/abs/1105.5449), *Journal of Artificial Intelligence Research*, vol. 9, pp. 317–365, 1998, the original AntNet algorithm.
2. G. A. Di Caro, *Ant Colony Optimization and its application to adaptive routing in telecommunication networks*, PhD thesis, Université Libre de Bruxelles, 2004.
3. G. A. Di Caro and M. Dorigo, *"Two Ant Colony Algorithms for Best-Effort Routing in Datagram Networks,"* Proceedings of PDCS'98, Las Vegas, Nevada, Oct. 1998.
4. Gianni A. Di Caro, [*AntNet*](http://www.giannidicaro.com/antnet.html), IDSIA, the algorithm author's own reference page, with links to AntNet's descendants (AntHocNet, etc.) and its known software implementations.
5. V. Laxmi, L. Jain, and M. S. Gaur, *"Ant Colony Optimisation Based Routing on NS-2,"* International Conference on Wireless Communication and Sensor Networks (WCSN), India, December 2006, the original ns-2 port this repository's `antnet.*` files trace back to (see [Provenance and Authorship](#provenance-and-authorship)).
6. M. Chandana and S. Thakur, ["Ant-Net: An Adaptive Routing Algorithm,"](https://doi.org/10.1109/ICPEICES.2016.7853616) *2016 IEEE 1st International Conference on Power Electronics, Intelligent Control and Energy Systems (ICPEICES)*, pp. 1–4, 2016.
7. L. Zhang and X. Li, ["The Research and Improvement of AntNet Algorithm,"](https://doi.org/10.1109/CAR.2010.5456609) *2010 2nd International Asia Conference on Informatics in Control, Automation and Robotics (CAR 2010)*, Wuhan, China, 2010. *(An independent AntNet variant evaluated in OPNET, not ns-2; its reference list cites Dorigo et al. and Di Caro & Dorigo's original AntNet paper, not this repository; it is included here as related literature, not as a citation of this codebase.)*

## How to Cite This Software

```bibtex
@misc{antnet,
  author = {LIMA, Richardson Edson de},
  title = {AntNet Algorithm for The Network Simulator (NS2)},
  year = {2009},
  publisher = {GitHub},
  journal = {GitHub repository},
  howpublished = {\url{https://github.com/richardsonlima/antnet}}
}
```

If you use or extend the AntNet algorithm itself (as opposed to this specific ns-2 port), please also cite Di Caro & Dorigo's original 1998 paper (item 1 in [Related Work](#related-work-and-further-reading)).

## License

GNU General Public License v2.0 (GPL-2.0), as stated in the header of every source file in this repository.

## Acknowledgments

- **Gianni Di Caro** and **Marco Dorigo**, for the original AntNet algorithm.
- **Lavina Jain**, **V. Laxmi**, and **M. S. Gaur**, for the original ns-2.28 AntNet implementation.
- **Matthew Orlinski** (University of Manchester), for suggestions and improvements to the ns-2.33/2.34 port.
- Everyone who has filed issues, suggestions, or fixes at [antnetalgorithm.blogspot.com](http://antnetalgorithm.blogspot.com.br) and [antnet.wordpress.com](http://antnet.wordpress.com) over the years.
