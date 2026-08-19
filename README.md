# AntNet Algorithm  for The Network Simulator

## AntNet Algorithm - Ant-based routing in networks

## Antnet for NS2.34

Antnet implementation for ns 2.34 based on the version for ns2.33 by Richardson Lima description available at http://antnetalgorithm.blogspot.com.br and http://antnet.wordpress.com

- Matthew Orlinski at orlinskm@cs.man.ac.uk and Richardson Lima at contato@richardsonlima.com.br with suggestions/improvements

> **Note:** You will need GCC4.0 to install ns 2.34.
If you have made any changes on your NS2.34 (e.g. installing newer protocols), installing with the below instructions MAY overright your changes. If you have made changes and you still want to use this protocol. The changelog below should be helpful to you in getting Antnet into ns2.34 without destroying your additions.


Copy the files provided into NS2.34 directory as is.

Run: 
```shell 
./configure make
```
Finally, enjoy

## Code review notes (2026-08-19)

The AntNet-specific files (`antnet.h/.cc`, `antnet_common.h/.cc`, `antnet_rtable.h/.cc`, `ant_pkt.h`, `antnet_traffic.h`) were reviewed and the following bugs were fixed:

- **`calc_next()` silently ignored its own probability formula.** It computed the correct AntNet selection probability (pheromone + heuristic bias towards shorter queues, weighted by `ALPHA`) into a local `thisprob`, but then partitioned the random-selection range using the raw pheromone value instead — so the queue-length heuristic never actually influenced routing decisions. `calc_next()` now uses the computed probability, and the formula itself was extracted into dependency-free helpers in `antnet_math.h` (see "Tests" below).
- **Missing `return` in `calc_next()`** when a destination had no routing-table entry, which is undefined behavior in C++. It now falls back to returning the parent node.
- **Cross-node global-state bug.** `N` (neighbor count) and `r` (reinforcement factor) were plain global variables read by `antnet_rtable::calc_next()`/`update()`. Since every node's `Antnet` agent runs in the same process in ns-2's single-threaded event loop, one node's routing decision could silently use another node's neighbor count or reinforcement factor. Both are now passed in explicitly by the caller instead.
- **Ant memory buffer overflow.** `Antnet::memorize()` appended to a fixed-size `pkt_memory_[MAX_NUM_NODES]` array with no bounds check; a path longer than `MAX_NUM_NODES` hops overflowed it. It now drops the packet instead, and callers were updated to stop touching the packet when that happens (it was previously used-after-free in that path too).
- **Null-pointer dereference** in `print_neighbors()` for a node with no neighbors (a `do`/`while` dereferenced before checking).
- **Uninitialized index** used to access `pkt_memory_[]` in `backward_ant_pkt()` if the current node wasn't found in the ant's memory; now guarded.
- **Uninitialized-pointer idiom** (`Node *nd = nd->get_node_by_address(...)`) used in a few places to reach what is effectively a static lookup; replaced with `Node::get_node_by_address(...)`.
- **Division by zero** in `initialize_rtable()` for a node with zero neighbors (`1.0/num_nb`).
- **Memory leak**: `antnet_rtable`'s RNG was `new`'d with no matching `delete`; it now has a destructor.
- Reduced `calc_next()` from up to 3 Tcl round-trips per neighbor (`get_queue_length()`) down to 1.

`update_traffic()` (the traffic-model/adaptive-`r` method) is dead code — it's never called, per the original author's own comment — and was left alone beyond keeping it compiling.

### Tests

`antnet_math.h` holds the pure AntNet math (pheromone reinforcement/evaporation, link-selection probability) with no ns-2 dependency, so it can be unit tested standalone without an ns-2 source tree:

```shell
cd tests && make test
```

The rest of this repo (`antnet.cc`, `antnet_rtable.cc`, the `queue/`/`trace/`/`tcl/` patches, ...) cannot be compiled or run in isolation — it's an add-on that expects to live inside a full ns-2.34 source tree (`agent.h`, `node.h`, `packet.h`, OTcl/TclCL, etc. aren't part of this repo). Validating those files requires copying them into an ns-2.34 tree per the instructions below and building/running a simulation there.

## Instructions

Change log for none destructive install
Additions we're made to ns2.34 to get antnet to work. The below is a guide only to be used in troubleshooting and ARE NOT COMPLETE INSTRUCTIONS. Not all changes are documented below but with these and following the clear advice on http://elmurod.net/wps/?p=157 on how to add a new routing protocol (TIP: Follow the instructions on the site substituting his protocol name "wfrp" for "antnet" and then make the changes below. This should get you antnet compiled and working)

File: queue/drop-tail.h 
```c++
public: int getlength();
````

Added to the DropTail class

File: queue/drop-tail.cc 
```c++
int DropTail::getlength() {
    return q_->length();
}
```

Add this method to queue/drop-tail.cc
```c++
File: queue/priqueue.cc 

case PT_ANT:
    if (ch->direction() == hdr_cmn::UP) {
    recvHighPriority(p, h);
    }
    else {
    Queue::recv(p, h);
    }
    break;
``` 

Added to queue/priqueue.cc instead

File: common/packet.h 
```c++
name_[PT_ANT] = "Ant";
[/Code] Basically anywhere that the WFRP tutorial (The website above) tells you to put PT_WFRP, put PT_ANT instead
```

File:trace/cmu-trace.cc 

```c++
#include <antnet/ant_pkt.h>
```

Remember to include this in cmu-trace.cc
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

The trace format function in cmu-trace.cc is the above taken from the ns2.33 version by Richardson Lima


```c++
case PT_ANT:
    format_antnet(p, offset);
    break;
```

Add this to the function CMUTrace::format(.....

File: tcl/lib/ns-default.tcl 
```tcl
# Defaults for Antnet
Agent/Antnet set num_nodes_x_ 4
Agent/Antnet set num_nodes_y_ 4
Agent/Antnet set num_nodes_ 16
Agent/Antnet set r_factor_ 0.001
Agent/Antnet set timer_ant_ 0.03
```

Add this at the end

File: tcl/lib/ns-default.tcl 
```tcl
Simulator instproc get-drop-queue { n1 n2 } {
    $self instvar link_
    set q [$link_($n1:$n2) queue]
    return $q
}
```

Add this code to tcl/lib/ns-lib.tcl

## 📃 Citation

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
