from collections import defaultdict
from heapq import *

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
        self.edge_list = []
        self.path_directory = {}
        self.path_directory_ipv4 = {}
        self.attribute_dir = []
        self.counter = 0
        self.dir_of_switches = []
        self.no_of_switches = 0
        self.link_flow_counter = {}



    # Handy function that lists all attributes in the given object

    def ls(self,obj):
        print("\n".join([x for x in dir(obj) if x[0] != "_"]))


    #-- This is the function which adds a flow entry to the the Controller
    def add_flow(self, msg, datapath, match, actions):

        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser

        inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS, actions)]

        if msg.buffer_id != ofproto.OFP_NO_BUFFER:
            mod = parser.OFPFlowMod(datapath=datapath, buffer_id=msg.buffer_id,
                                    priority=ofproto.OFP_DEFAULT_PRIORITY, match=match,
                                    instructions=inst)

        else:
            mod = parser.OFPFlowMod(datapath=datapath, priority=ofproto.OFP_DEFAULT_PRIORITY,
                                    match=match, instructions=inst)



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



    #-- This is where I am inserting the Dijkstra's Algorithm Implementation (EDITED: This is Dijkstra which gives Equal Paths) --

    def dijkstra(self,edges, f, t):
        g = defaultdict(list)
        m_path = []
        for l,r,c in edges:
            g[l].append((c,r))

        q, seen = [(0,f,())], set()
        while q:
            (cost,v1,path) = heappop(q)
            if v1 not in seen:
                seen.add(v1)
            path = (v1, path)
            if v1 == t:
                if not m_path:
                    m_path.append((cost,path))
                elif cost < m_path[0][0]:
                    del m_path[:]
                    m_path.append((cost,path))
                elif cost == m_path[0][0]:
                    m_path.append((cost,path))

            for c, v2 in g.get(v1, ()):
                if v2 not in seen:
                    heappush(q, (cost+c, v2, path))

        return m_path

    def dijkstra_call(self,edges, f, t):
        out = self.dijkstra(edges, f, t)
        ans = []
        for i in out:
            data = {}
            data['cost']=i[0]
            aux=[]
            while len(i)>1:
                aux.append(i[0])
                i = i[1]
            aux.remove(data['cost'])
            aux.reverse()
            data['path']=aux
            ans.append(aux)
        return ans

    ### -- This is where the Dijkstra's Algorithm Implementation Ends --

    ### -- This is the function which finds the best path from the equal cost paths available
    def find_best_path(self,paths):
        print "The Paths Obtained are: "
        print paths
        bot_neck = []
        cnt = len(paths)
        for x in range(cnt):
            bot_neck.append(0)
        count = 0
        index = 0
        min = 100


        for path in paths:
            max = 0
            nodes = len(path)
            for x in range(nodes-1):
                if path[x] < path[x+1]:
                    if max <= self.link_flow_counter[path[x],path[x+1]]:
                        max = self.link_flow_counter[path[x],path[x+1]]
                        bot_neck[count] = (path[x],path[x+1])
                else:
                    if max <= self.link_flow_counter[path[x+1],path[x]]:
                        max = self.link_flow_counter[path[x+1],path[x]]
                        bot_neck[count] = (path[x+1],path[x])

            count = count + 1

        for x in range(count):
            if min > self.link_flow_counter[bot_neck[x][0],bot_neck[x][1]]:
                min = self.link_flow_counter[bot_neck[x][0],bot_neck[x][1]]
                index = x

        return paths[index]
        ### -- This is where the implementation end for the find_best_path function









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
        port_stat = datapath.ofproto_parser.OFPPort
        #p = msg.body


        pkt = packet.Packet(msg.data)
        eth = pkt.get_protocols(ethernet.ethernet)[0]

        if eth.ethertype == ether_types.ETH_TYPE_LLDP or eth.ethertype == ether_types.ETH_TYPE_IPV6:
        # ignore lldp packet and the IPv6 Packets which are sent at the beginning of topology setup
        # This gets rid of all the unneccesary packets which flow through def dijkstra(self,edges, f, t):




        dst = eth.dst   #-- Destination of my Ethernet Packet
        src = eth.src   #-- Source of my Ethernet Packet
        dpid = datapath.id  #-- Switch ID
        eth_IP = ether.ETH_TYPE_IP
        eth_MPLS = ether.ETH_TYPE_MPLS
        ethtype = eth.ethertype

        #print "The Port being used is: %s and it's max_speed is: %s"%(p.port_no,p.max_speed)

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
            prio = 100
            actions = []
            match = parser.OFPMatch(eth_src=src,eth_type = ethtype)
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
        '''if src not in self.net:

            print "Adding a node to the graph: %s"%(src)
            print "Adding a link to the graph between %s and %s"%(dpid,src)
            self.net.add_node(src)
            self.net.add_edge(dpid,src,{'port':in_port})
            self.net.add_edge(src,dpid)
        '''

        ## The following code is used to find the path to the next hop if the destination host
        ## exists in the topology. The Networkx module computes the shortest path to the destination host.
        if dst in self.host_to_switch:

            print "Destination exists in the graph !"
            ## This is where the path is calculated

            if dpid == self.host_to_switch[dst]:
                out_port = self.mac_to_port[dpid][dst]

            elif dpid == self.host_to_switch[src]:
                paths=self.path_directory[self.host_to_switch[src]][self.host_to_switch[dst]] ##- Getting the path from Pre-computed path directory
                routes = len(paths)
                if routes > 1:
                    path = self.find_best_path(paths)
                    self.path_directory[self.host_to_switch[src]][self.host_to_switch[dst]] = path
                else:
                    path = paths[0]
                    self.path_directory[self.host_to_switch[src]][self.host_to_switch[dst]] = path

                print "The Path [ARP-Handler] (after computation) to the destination is:"
                print path
                ## We find the next hop by looking for the next hop in our path determined by the Networkx module.
                next=path[path.index(dpid)+1]
                ## This gives us the output port of the next hop.
                out_port=self.net[dpid][next]['port']

                ## This Following code introduces the counter
                a = dpid
                b = next
                if a < b:
                    cnt = self.link_flow_counter[a,b]
                    cnt = cnt + 1
                    self.link_flow_counter[a,b] = cnt
                else:
                    cnt = self.link_flow_counter[b,a]
                    cnt = cnt + 1
                    self.link_flow_counter[b,a] = cnt

                print "Change in link flow counter::"
                print self.link_flow_counter

            else:
                path = self.path_directory[self.host_to_switch[src]][self.host_to_switch[dst]]
                print "The Path [ARP-Handler] (No Computation) to the destination is:"
                print path
                ## We find the next hop by looking for the next hop in our path determined by the Networkx module.
                next=path[path.index(dpid)+1]
                ## This gives us the output port of the next hop.
                out_port=self.net[dpid][next]['port']

                ## This Following code introduces the counter
                a = dpid
                b = next
                if a < b:
                    cnt = self.link_flow_counter[a,b]
                    cnt = cnt + 1
                    self.link_flow_counter[a,b] = cnt
                else:
                    cnt = self.link_flow_counter[b,a]
                    cnt = cnt + 1
                    self.link_flow_counter[b,a] = cnt

                print "Change in link flow counter::"
                print self.link_flow_counter




        ## If the Destination host is not present in the Topology yet then we FLOOD our packet till we get to the destination host.
        else:
            print "Destination is not in the graph hence Flooding"
            out_port = ofproto.OFPP_FLOOD


        ##-- We set the action as forwarding the packet on the determined output port of the next hop.
        actions = [datapath.ofproto_parser.OFPActionOutput(out_port)]


        ## We only Establish a FLow Table Entry if we are not Flooding
        if out_port != ofproto.OFPP_FLOOD:
            match = parser.OFPMatch(eth_dst=dst,in_port = in_port,eth_type = ethtype)
            self.add_flow(msg,datapath, match, actions)
            print "Adding flow for Switch:%s for inport:%s and destination:%s"%(dpid,in_port,dst)
            out = datapath.ofproto_parser.OFPPacketOut(datapath=datapath, buffer_id=msg.buffer_id, in_port=in_port,actions=actions)
            datapath.send_msg(out)

        else:
            prio = 1000
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
        #path=nx.shortest_path(self.net,self.host_to_switch[src],self.host_to_switch[dst])
        #path = self.dijkstra_call(self.edge_list,self.host_to_switch[src],self.host_to_switch[dst]) ##- Call to the Dijkstra's Algorithm Function
        #path = self.path_directory[self.host_to_switch[src]][self.host_to_switch[dst]] ##- Getting the path from Pre-computed path directory
        #print "The Dijkstra Path [IPv4 Handler] to the destination is:"
        #print path
        paths=self.path_directory_ipv4[self.host_to_switch[src]][self.host_to_switch[dst]] ##- Getting the path from Pre-computed path directory
        routes = len(paths)
        if routes > 1:
            path = self.find_best_path(paths)
            self.path_directory_ipv4[self.host_to_switch[src]][self.host_to_switch[dst]] = path
        else:
            path = paths[0]
            self.path_directory_ipv4[self.host_to_switch[src]][self.host_to_switch[dst]] = path

        print "The Path [IPv4-Handler] (after computation) to the destination is:"
        print path


        ## -- The Only time this Function will be called is when the Source Host sends the packet to
        ## -- its attached Switch. Here the Switch will Push the MPLS LABEL of its next hop.

        if self.host_to_switch[src] == dpid:
            ## We have to Push a Label to the Current Packet
            dst_switch = self.host_to_switch[dst]
            curr_label = self.mylabeltable[dpid][dst_switch]
            ## We find the next hop by looking for the next hop in our path determined by the Networkx module.
            next=path[path.index(dpid)+1]

            ## This Following code introduces the counter
            a = dpid
            b = next
            if a < b:
                cnt = self.link_flow_counter[a,b]
                cnt = cnt + 1
                self.link_flow_counter[a,b] = cnt
            else:
                cnt = self.link_flow_counter[b,a]
                cnt = cnt + 1
                self.link_flow_counter[b,a] = cnt

            print "Change in link flow counter::"
            print self.link_flow_counter




            ## This gives us the output port of the next hop.
            out_port=self.net[dpid][next]['port']
            swap_label = self.mylabeltable[next][dst_switch] ## -- Here the Swap Label contains the label which will be pushed onto the Packet.
            print "The Label being placed is:%s"%(swap_label)
            match = parser.OFPMatch(eth_dst = dst,in_port = in_port,eth_type = ethtype)
            actions = [parser.OFPActionPushMpls(ethertype=34887,type_=None,len_=None),parser.OFPActionSetField(mpls_label = swap_label),parser.OFPActionOutput(out_port)]
            self.add_flow(msg, datapath, match, actions)
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

        #path=nx.shortest_path(self.net,self.host_to_switch[src],self.host_to_switch[dst])
        #path = self.dijkstra_call(self.edge_list,self.host_to_switch[src],self.host_to_switch[dst])  ## Call to the Dijkstra's Algorithm Function
        path = self.path_directory_ipv4[self.host_to_switch[src]][self.host_to_switch[dst]] ## Getting the path from pre-computed path directory
        print "The Dijkstra Path [MPLS Handler] to the destination is:"
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
            self.add_flow(msg, datapath, match, actions)
            print "Adding flow for Switch:%s for inport:%s and destination:%s"%(dpid,in_port,dst)


        else:
                ## We have to replace the current label with the next Label

            dst_switch = self.host_to_switch[dst]
            curr_label = self.mylabeltable[dpid][dst_switch]
            ## We find the next hop by looking for the next hop in our path determined by the Networkx module.
            next=path[path.index(dpid)+1]

            ## This Following code introduces the counter
            a = dpid
            b = next
            if a < b:
                cnt = self.link_flow_counter[a,b]
                cnt = cnt + 1
                self.link_flow_counter[a,b] = cnt
            else:
                cnt = self.link_flow_counter[b,a]
                cnt = cnt + 1
                self.link_flow_counter[b,a] = cnt

            print "Change in link flow counter::"
            print self.link_flow_counter




            ## This gives us the output port of the next hop.
            out_port=self.net[dpid][next]['port']
            swap_label = self.mylabeltable[next][dst_switch] ## The Swap Label is the Label being placed on the packet.
            print "The Label being placed is:%s"%(swap_label)
            match = parser.OFPMatch(in_port = in_port,eth_dst = dst,eth_type = ethtype,mpls_label = mpls_proto.label)
            actions = [parser.OFPActionPopMpls(),parser.OFPActionPushMpls(),parser.OFPActionSetField(mpls_label = swap_label),parser.OFPActionOutput(out_port)]
            self.add_flow(msg, datapath, match, actions)
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


    ## -- This is the function which gets called to append the Cost Factor in the Edges
    def fill_out_edges(self,edges):
        if self.counter == 0:
            print "Please enter the name in quotes!!"
            filename = input("Enter the name of the file to extract link features: ")
            self.counter = self.counter + 1
            filepath = "/home/avirudh/mininet/examples/"
            filepath = filepath+filename
            f = open(filepath,"r")
            for line in f:
                s = list(line)
                if s[0] != '\n' and s[0] != '*':
                    #print "The List of Features are:"
                    #print s
                    self.attribute_dir.append(s[4])
                    if s[0] not in self.dir_of_switches:
                        self.dir_of_switches.append(s[0])
                    if s[2] not in self.dir_of_switches:
                        self.dir_of_switches.append(s[2])

            f.close()
            self.no_of_switches = len(self.dir_of_switches)

        elif self.counter == self.no_of_switches:
            self.edge_list = []
            temp1 = 0
            for x in edges:
                temp = (int(self.attribute_dir[temp1]),)
                self.edge_list.append(x + temp)
                temp1 = temp1+1
        else:
            self.edge_list = []
            for x in edges:
                temp = (1,)
                self.edge_list.append(x + temp)
            self.counter = self.counter+1

        print "The Edge_List used by Dijkstra is as follows:"

    ## -- This is the function which auto-computes the path from one switch to every other switch
    def fill_out_paths(self,switches):
        for x in switches:
            self.path_directory.setdefault(x,{})
            #print "*The paths for Switch : %d are:"%(x)
            for y in switches:
                self.path_directory[x][y] = self.dijkstra_call(self.edge_list,x,y)
            print "The ARP Paths for Switch:%s are:"%(x)
            print self.path_directory[x]

    def fill_out_paths_ipv4(self,switches):
        for x in switches:
            self.path_directory_ipv4.setdefault(x,{})
            #print "*The paths for Switch : %d are:"%(x)
            for y in switches:
                self.path_directory_ipv4[x][y] = self.dijkstra_call(self.edge_list,x,y)
            print "The IPv4 Paths for Switch:%s are:"%(x)
            print self.path_directory_ipv4[x]


    def init_counter(self,links):
        for x in links:
            a = x[0]
            b = x[1]
            if a < b:
                if (a,b) not in self.link_flow_counter:
                    self.link_flow_counter[a,b] = 0
            else:
                if(b,a) not in self.link_flow_counter:
                    self.link_flow_counter[b,a] = 0



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

        print links

        #print self.net.edges()

        self.init_counter(links)

        self.fill_out_edges(self.net.edges())

        print self.edge_list

        self.fill_out_paths(switches)

        self.fill_out_paths_ipv4(switches)

        self.fill_out_labels(switches)
