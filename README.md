AntNet Algorithm  for The Network Simulator - ns-2 (www.isi.edu/nsnam/ns/)
======

AntNet Algorithm - Ant-based routing in networks

Antnet for NS2.34
Antnet implementation for ns2.34 based on the version for ns2.33 by Richardson Lima description available at http://antnet.wordpress.com

Matthew Orlinski at orlinskm@cs.man.ac.uk and Richardson Lima at richardsonlimasurfjiujitsu@gmail.com with suggestions/improvements

REMEMBER, YOU NEED GCC4.0 TO INSTALL NS2.34 * ***********************************************
WARNING
If you have made any alterations to your NS2.34 (e.g. installing newer protocols), installing with the below instructions MAY overright your changes. If you have made alterations and you still want to use this protocol. The changelog below should be helpful to you in getting Antnet into ns2.34 without destroying your additions.

Instructions
copy the files provided into NS2.34 directory as is.

Run: ./configure make

Finally, enjoy

Change log for none destructive install
Additions we're made to ns2.34 to get antnet to work. The below is a guide only to be used in troubleshooting and ARE NOT COMPLETE INSTRUCTIONS. Not all changes are documented below but with these and following the clear advice on http://elmurod.net/wps/?p=157 on how to add a new routing protocol (TIP: Follow the instructions on the site substituting his protocol name "wfrp" for "antnet" and then make the changes below. This should get you antnet compiled and working)

// =========================== // queue/drop-tail.h // =========================== [Code]

public: int getlength();

[/Code] Added to the DropTail class

// =========================== // queue/drop-tail.cc // =========================== [Code]

int DropTail::getlength() {
    return q_->length();
}
[/Code] add this method to queue/drop-tail.cc

// =========================== // queue/priqueue.cc // =========================== [Code]

case PT_ANT:
    if (ch->direction() == hdr_cmn::UP) {
    recvHighPriority(p, h);
    }
    else {
    Queue::recv(p, h);
    }
    break;
[/Code] Added to queue/priqueue.cc instead

// =========================== // common/packet.h // =========================== [Code]

name_[PT_ANT] = "Ant";
[/Code] Basically anywhere that the WFRP tutorial (The website above) tells you to put PT_WFRP, put PT_ANT instead

// =========================== // trace/cmu-trace.cc // =========================== [Code]

#include <antnet/ant_pkt.h>
[/Code] Remember to include this in cmu-trace.cc

[Code]

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
[/Code] The trace format function in cmu-trace.cc is the above taken from the ns2.33 version by Richardson Lima

[Code]

case PT_ANT:
    format_antnet(p, offset);
    break;
[/Code] Add this to the function CMUTrace::format(.....

// =========================== // tcl/lib/ns-default.tcl // =========================== [Code]

# Defaults for Antnet
Agent/Antnet set num_nodes_x_ 4
Agent/Antnet set num_nodes_y_ 4
Agent/Antnet set num_nodes_ 16
Agent/Antnet set r_factor_ 0.001
Agent/Antnet set timer_ant_ 0.03
[/Code] Add this at the end

// =========================== // tcl/lib/ns-default.tcl // =========================== [Code]

Simulator instproc get-drop-queue { n1 n2 } {
    $self instvar link_
    set q [$link_($n1:$n2) queue]
    return $q
}
[/Code] Add this code to tcl/lib/ns-lib.tcl