from ryu.base import app_manager
from ryu.controller import mac_to_port
from ryu.controller import ofp_event
from ryu.controller.handler import CONFIG_DISPATCHER, MAIN_DISPATCHER
from ryu.controller.handler import set_ev_cls
from ryu.ofproto import ofproto_v1_3
from ryu.ofproto import ether
from ryu.lib.mac import haddr_to_bin
from ryu.lib.packet import packet
from ryu.lib.packet import ethernet
from ryu.lib.packet import mpls
from ryu.lib.packet import ether_types
from ryu.lib import mac


from ryu.topology.api import get_switch, get_link
from ryu.app.wsgi import ControllerBase
from ryu.topology import event, switches
import networkx as nx



class SimpleSwitch13(app_manager.RyuApp):

    OFP_VERSIONS = [ofproto_v1_3.OFP_VERSION]



    def __init__(self, *args, **kwargs):

        super(SimpleSwitch13, self).__init__(*args, **kwargs)
        self.mac_to_port = {}
        self.topology_api_app = self
        self.net=nx.DiGraph()
        self.nodes = {}
        self.links = {}
        self.mylabeltable = {}
        self.swaptable = {}
        self.host_to_switch = {}
        self.no_of_nodes = 0
        self.no_of_links = 0
        self.i=0



    # Handy function that lists all attributes in the given object

    def ls(self,obj):
        print("\n".join([x for x in dir(obj) if x[0] != "_"]))


    #-- This is the function which adds a flow entry to the the Controller
    def add_flow(self, datapath, match, actions):

        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser

        inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS, actions)]
        mod = datapath.ofproto_parser.OFPFlowMod(
            datapath=datapath, match=match, cookie=0,
            command=ofproto.OFPFC_ADD, idle_timeout=0, hard_timeout=0,
            priority=ofproto.OFP_DEFAULT_PRIORITY, instructions=inst)
        datapath.send_msg(mod)



    @set_ev_cls(ofp_event.EventOFPSwitchFeatures , CONFIG_DISPATCHER)
    #-- This is the function which gets called when the switches connect to the controller
    def switch_features_handler(self , ev):

        print "switch_features_handler is called"

        datapath = ev.msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        match = parser.OFPMatch()
        #-- This is setting the action as Connect to the Controller
        actions = [parser.OFPActionOutput(ofproto.OFPP_CONTROLLER, ofproto.OFPCML_NO_BUFFER)]

        inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS , actions)]

        mod = datapath.ofproto_parser.OFPFlowMod(
            datapath=datapath, match=match, cookie=0,
            command=ofproto.OFPFC_ADD, idle_timeout=0, hard_timeout=0, priority=0, instructions=inst)
        datapath.send_msg(mod)



    @set_ev_cls(ofp_event.EventOFPPacketIn, MAIN_DISPATCHER)
    #-- This is the MAIN function
    #-- This is the Function where we Define our Logic when a new Packet comes in
    def _packet_in_handler(self, ev):

        msg = ev.msg
        datapath = msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        in_port = msg.match['in_port']     #-- Storing the inport (Port where the node connected to the switch) as it will be used later


        pkt = packet.Packet(msg.data)
        eth = pkt.get_protocols(ethernet.ethernet)[0]

        if eth.ethertype == ether_types.ETH_TYPE_LLDP or eth.ethertype == ether_types.ETH_TYPE_IPV6:
        # ignore lldp packet and the IPv6 Packets which are sent at the beginning of topology setup
        # This gets rid of all the unneccesary packets which flow through in the beginning of topology setup
            return



        dst = eth.dst   #-- Destination of my Ethernet Packet
        src = eth.src   #-- Source of my Ethernet Packet
        dpid = datapath.id  #-- Switch ID
        eth_IP = ether.ETH_TYPE_IP
        eth_MPLS = ether.ETH_TYPE_MPLS
        ethtype = eth.ethertype ## -- This is the variable which will be storing the Type of Packet

        self.mac_to_port.setdefault(dpid, {})

        #print self.mylabeltable[dpid]


        ## In this Piece of code we are trying to seggregate the packet based
        ## On the Type of Packet it is. It can either be an Ethernet Packet (PINGS)
        ## or it could be an IP packet or it could be an MPLS Packet.
        ## If it is an Ethernet packet we call the arpHandler Function.
        ## If it is an IPv4 Packet we call the ipv4Handler Function.
        ## If it is an MPLS Packet we call the mplsHandler Function.


        if ethtype == 2054: ## -- This is the Hexa code Ethernet Packet
            self.arpHandler(msg)

        if ethtype == 2048: ## -- This is the Hexa Code for IPv4 Packet
            self.ipv4Handler(msg)

        if ethtype == 34887: ## -- This is the Hexa Code for MPLS Packet
            self.mplsHandler(msg)



    ## -- This is the function which defines the Logic for the Ethernet Packet
    def arpHandler(self,msg):

        #msg = ev.msg
        datapath = msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        in_port = msg.match['in_port']     #-- Storing the inport (Port where the node connected to the switch) as it will be used later


        pkt = packet.Packet(msg.data)
        eth = pkt.get_protocols(ethernet.ethernet)[0]

        if eth.ethertype == ether_types.ETH_TYPE_LLDP or eth.ethertype == ether_types.ETH_TYPE_IPV6:
        # ignore lldp packet and the IPv6 Packets which are sent at the beginning of topology setup
        # This gets rid of all the unneccesary packets which flow through in the beginning of topology setup
            return



        dst = eth.dst   #-- Destination of my Ethernet Packet
        src = eth.src   #-- Source of my Ethernet Packet
        dpid = datapath.id  #-- Switch ID
        eth_IP = ether.ETH_TYPE_IP
        eth_MPLS = ether.ETH_TYPE_MPLS
        ethtype = eth.ethertype

        ## -- This Piece of code checks if we have encountered the host in our topology before
        ## -- If not then we keep track of the switch it is connected to.
        if src not in self.host_to_switch:
            self.host_to_switch[src] = dpid

        ## This piece of code checks if we have encountered the host before at this switch
        ## IF we haven't the we associate the incoming port to that host.
        if src not in self.mac_to_port[dpid]:
            self.mac_to_port[dpid][src] = in_port

        ## This is piece of code checks if I have received a ethernet packet again by
        ## checking if the Source of the packet has been seen before and if the incoming port
        ## is different than the inport we registered for that port.
        ## We install a flow with MATCH(source,in_port) and we set the ACTION to drop the packet
        ## Hence we stop the flooding of the initial ping packets
        if src in self.mac_to_port[dpid] and in_port != self.mac_to_port[dpid][src]:
            prio = 2
            actions = []
            match = parser.OFPMatch(eth_src=src,in_port = in_port,eth_type = ethtype)
            inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS,actions)]
            if msg.buffer_id != ofproto.OFP_NO_BUFFER:
                mod = parser.OFPFlowMod(datapath=datapath, buffer_id=msg.buffer_id,
                                        priority=prio, match=match,
                                        instructions=inst)

            else:
                mod = parser.OFPFlowMod(datapath=datapath, priority=prio,
                                        match=match, instructions=inst)

            datapath.send_msg(mod)
            return
        #-- END of the piece of code


        ## The following piece of Code is COncerned with our Topology we obtain from the NETWORKX module
        ## Initially the Topology only knows about the Switches. It knows nothing about the Hosts
        ## As and when a host connects to the Switch the host is added to the network topology and
        ## a link is added between the switch and the host in our network topology.

        ## Checking if the Source Host is present in the Topology. If no then it is added to the topology.
        if src not in self.net:

            print "Adding a node to the graph: %s"%(src)
            print "Adding a link to the graph between %s and %s"%(dpid,src)
            self.net.add_node(src)
            self.net.add_edge(dpid,src,{'port':in_port})
            self.net.add_edge(src,dpid)

        ## The following code is used to find the path to the next hop if the destination host
        ## exists in the topology. The Networkx module computes the shortest path to the destination host.
        if dst in self.net:

            print "Destination exists in the graph !"
            ## This is where the path is calculated
            path=nx.shortest_path(self.net,src,dst)

            print "The Path to the destination is:"
            print path
            ## We find the next hop by looking for the next hop in our path determined by the Networkx module.
            next=path[path.index(dpid)+1]
            ## This gives us the output port of the next hop.
            out_port=self.net[dpid][next]['port']

        ## If the Destination host is not present in the Topology yet then we FLOOD our packet till we get to the destination host.
        else:
            print "Destination is not in the graph hence Flooding"
            out_port = ofproto.OFPP_FLOOD


        ##-- We set the action as forwarding the packet on the determined output port of the next hop.
        actions = [datapath.ofproto_parser.OFPActionOutput(out_port)]


        ## We only Establish a FLow Table Entry if we are not Flooding
        if out_port != ofproto.OFPP_FLOOD:
            match = parser.OFPMatch(eth_dst=dst,in_port = in_port,eth_type = ethtype)
            self.add_flow(datapath, match, actions)
            print "Adding flow for Switch:%s for inport:%s and destination:%s"%(dpid,in_port,dst)


        out = datapath.ofproto_parser.OFPPacketOut(

            datapath=datapath, buffer_id=msg.buffer_id, in_port=in_port,

            actions=actions)

        datapath.send_msg(out)


    ## -- This is the Function which is called when we receive an IPv4 Packet
    def ipv4Handler(self,msg):


        #msg = ev.msg
        datapath = msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        in_port = msg.match['in_port']     #-- Storing the inport (Port where the node connected to the switch) as it will be used later


        pkt = packet.Packet(msg.data)
        eth = pkt.get_protocols(ethernet.ethernet)[0]

        if eth.ethertype == ether_types.ETH_TYPE_LLDP or eth.ethertype == ether_types.ETH_TYPE_IPV6:
        # ignore lldp packet and the IPv6 Packets which are sent at the beginning of topology setup
        # This gets rid of all the unneccesary packets which flow through in the beginning of topology setup
            return



        dst = eth.dst   #-- Destination of my Ethernet Packet
        src = eth.src   #-- Source of my Ethernet Packet
        dpid = datapath.id  #-- Switch ID
        eth_IP = ether.ETH_TYPE_IP
        eth_MPLS = ether.ETH_TYPE_MPLS
        ethtype = eth.ethertype



        print "Destination exists in the graph !"
        ## This is where the path is calculated
        path=nx.shortest_path(self.net,self.host_to_switch[src],self.host_to_switch[dst])
        print "The Path to the destination is:"
        print path


        ## -- The Only time this Function will be called is when the Source Host sends the packet to
        ## -- its attached Switched. Here the Switch will Push the MPLS LABEL of its next hop.

        if self.host_to_switch[src] == dpid:
            ## We have to Push a Label to the Current Packet
            dst_switch = self.host_to_switch[dst]
            curr_label = self.mylabeltable[dpid][dst_switch]
            ## We find the next hop by looking for the next hop in our path determined by the Networkx module.
            next=path[path.index(dpid)+1]
            ## This gives us the output port of the next hop.
            out_port=self.net[dpid][next]['port']
            swap_label = self.mylabeltable[next][dst_switch] ## -- Here the Swap Label contains the label which will be pushed onto the Packet.
            print "The Label being placed is:%s"%(swap_label)
            match = parser.OFPMatch(eth_dst = dst,in_port = in_port,eth_type = ethtype)
            actions = [parser.OFPActionPushMpls(ethertype=34887,type_=None,len_=None),parser.OFPActionSetField(mpls_label = swap_label),parser.OFPActionOutput(out_port)]
            self.add_flow(datapath, match, actions)
            print "Adding flow for Switch:%s for inport:%s and destination:%s"%(dpid,in_port,dst)

            out = datapath.ofproto_parser.OFPPacketOut(datapath=datapath, buffer_id=msg.buffer_id, in_port=in_port,actions=actions)

            datapath.send_msg(out)


    ## -- This is the function which is called when we receive an MPLS Packet
    def mplsHandler(self,msg):


        #msg = ev.msg
        datapath = msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        in_port = msg.match['in_port']     #-- Storing the inport (Port where the node connected to the switch) as it will be used later


        pkt = packet.Packet(msg.data)
        eth = pkt.get_protocols(ethernet.ethernet)[0]
        mpls_proto = pkt.get_protocol(mpls.mpls)

        if eth.ethertype == ether_types.ETH_TYPE_LLDP or eth.ethertype == ether_types.ETH_TYPE_IPV6:
        # ignore lldp packet and the IPv6 Packets which are sent at the beginning of topology setup
        # This gets rid of all the unneccesary packets which flow through in the beginning of topology setup
            return



        dst = eth.dst   #-- Destination of my Ethernet Packet
        src = eth.src   #-- Source of my Ethernet Packet
        dpid = datapath.id  #-- Switch ID
        eth_IP = ether.ETH_TYPE_IP
        eth_MPLS = ether.ETH_TYPE_MPLS
        ethtype = eth.ethertype

        path=nx.shortest_path(self.net,self.host_to_switch[src],self.host_to_switch[dst])
        print "The Path to the destination is:"
        print path

        ## -- This function can be called in 2 scenarios.
        ## -- A> When the switch is not the end Switch [Here the Action should be Pop -> Push]
        ## -- B> When the switch is the end switch [Here the Action should be POP Label]


        if self.host_to_switch[dst] == dpid:
                ## We have to pop the label from the current packet

            out_port = self.mac_to_port[dpid][dst]
            dst_switch = self.host_to_switch[dst]
            curr_label = self.mylabeltable[dpid][dst_switch]
            print "The Label being popped is:%s"%(curr_label)
            match = parser.OFPMatch(in_port = in_port,eth_dst = dst,eth_type = ethtype,mpls_label = mpls_proto.label)
            actions = [parser.OFPActionPopMpls(),parser.OFPActionOutput(out_port)]
            self.add_flow(datapath, match, actions)
            print "Adding flow for Switch:%s for inport:%s and destination:%s"%(dpid,in_port,dst)


        else:
                ## We have to replace the current label with the next Label

            dst_switch = self.host_to_switch[dst]
            curr_label = self.mylabeltable[dpid][dst_switch]
            ## We find the next hop by looking for the next hop in our path determined by the Networkx module.
            next=path[path.index(dpid)+1]
            ## This gives us the output port of the next hop.
            out_port=self.net[dpid][next]['port']
            swap_label = self.mylabeltable[next][dst_switch] ## The Swap Label is the Label being placed on the packet. 
            print "The Label being placed is:%s"%(swap_label)
            match = parser.OFPMatch(in_port = in_port,eth_dst = dst,eth_type = ethtype,mpls_label = mpls_proto.label)
            actions = [parser.OFPActionPopMpls(),parser.OFPActionPushMpls(),parser.OFPActionSetField(mpls_label = swap_label),parser.OFPActionOutput(out_port)]
            self.add_flow(datapath, match, actions)
            print "Adding flow for Switch:%s for inport:%s and destination:%s"%(dpid,in_port,dst)



        out = datapath.ofproto_parser.OFPPacketOut(

            datapath=datapath, buffer_id=msg.buffer_id, in_port=in_port,

            actions=actions)

        datapath.send_msg(out)



    ## -- This is the function which gets called to initialize the labels
    def fill_out_labels(self,switches):
        k = 10
        for x in switches:
            self.mylabeltable.setdefault(x,{})
            for y in switches:
                self.mylabeltable[x][y] = k
                k = k+10



    @set_ev_cls(event.EventSwitchEnter)
    ##-- This is the function which gets called initially when the Switches Connect to the Controller
    ##-- This creates the initial Topology for the Networkx Module for all the switches
    def get_topology_data(self, ev):
        print "I am HERE !"
        ## Creating a list of Switches from the topology
        switch_list = get_switch(self.topology_api_app, None)

        switches=[switch.dp.id for switch in switch_list]

        self.net.add_nodes_from(switches)

        print "*List of switches*"

        for switch in switch_list:

          #self.ls(switch)

          print switch

          #self.nodes[self.no_of_nodes] = switch

          #self.no_of_nodes += 1


        ## Creating a list of Links between the switches in the topology
        links_list = get_link(self.topology_api_app, None)



        links=[(link.src.dpid,link.dst.dpid,{'port':link.src.port_no}) for link in links_list]

        self.net.add_edges_from(links)

        links=[(link.dst.dpid,link.src.dpid,{'port':link.dst.port_no}) for link in links_list]

        self.net.add_edges_from(links)

        print "*List of links*"

        print self.net.edges()

        self.fill_out_labels(switches)
