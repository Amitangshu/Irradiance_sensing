#include <Timer.h>
#include <TreeRouting.h>
#include <CollectionDebugMsg.h>
/* $Id: CtpRoutingEngineP.nc,v 1.23 2010/02/04 07:31:46 gnawali Exp $ */
/*
 * "Copyright (c) 2005 The Regents of the University  of California.  
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF
 * CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 *
 */

/** 
 *  The TreeRoutingEngine is responsible for computing the routes for
 *  collection.  It builds a set of trees rooted at specific nodes (roots) and
 *  maintains these trees using information provided by the link estimator on
 *  the quality of one hop links.
 * 
 *  <p>Each node is part of only one tree at any given time, but there is no
 *  difference from the node's point of view of which tree it is part. In other
 *  words, a message is sent towards <i>a</i> root, but which one is not
 *  specified. It is assumed that the roots will work together to have all data
 *  aggregated later if need be.  The tree routing engine's responsibility is
 *  for each node to find the path with the least number of transmissions to
 *  any one root.
 *
 *  <p>The tree is proactively maintained by periodic beacons sent by each
 *  node. These beacons are jittered in time to prevent synchronizations in the
 *  network. All nodes maintain the same <i>average</i> beacon sending rate
 *  (defined by BEACON_INTERVAL +- 50%). The beacon contains the node's parent,
 *  the current hopcount, and the cumulative path quality metric. The metric is
 *  defined as the parent's metric plus the bidirectional quality of the link
 *  between the current node and its parent.  The metric represents the
 *  expected number of transmissions along the path to the root, and is 0 by
 *  definition at the root.
 * 
 *  <p>Every time a node receives an update from a neighbor it records the
 *  information if the node is part of the neighbor table. The neighbor table
 *  keeps the best candidates for being parents i.e., the nodes with the best
 *  path metric. The neighbor table does not store the full path metric,
 *  though. It stores the parent's path metric, and the link quality to the
 *  parent is only added when the information is needed: (i) when choosing a
 *  parent and (ii) when choosing a route. The nodes in the neighbor table are
 *  a subset of the nodes in the link estimator table, as a node is not
 *  admitted in the neighbor table with an estimate of infinity.
 * 
 *  <p>There are two uses for the neighbor table, as mentioned above. The first
 *  one is to select a parent. The parent is just the neighbor with the best
 *  path metric. It serves to define the node's own path metric and hopcount,
 *  and the set of child-parent links is what defines the tree. In a sense the
 *  tree is defined to form a coherent propagation substrate for the path
 *  metrics. The parent is (re)-selected periodically, immediately before a
 *  node sends its own beacon, in the updateRouteTask.
 *  
 *  <p>The second use is to actually choose a next hop towards any root at
 *  message forwarding time.  This need not be the current parent, even though
 *  it is currently implemented as such.
 *
 *  <p>The operation of the routing engine has two main tasks and one main
 *  event: updateRouteTask is called periodically and chooses a new parent;
 *  sendBeaconTask broadcasts the current route information to the neighbors.
 *  The main event is the receiving of a neighbor's beacon, which updates the
 *  neighbor table.
 *  
 *  <p> The interface with the ForwardingEngine occurs through the nextHop()
 *  call.
 * 
 *  <p> Any node can become a root, and routed messages from a subset of the
 *  network will be routed towards it. The RootControl interface allows
 *  setting, unsetting, and querying the root state of a node. By convention,
 *  when a node is root its hopcount and metric are 0, and the parent is
 *  itself. A root always has a valid route, to itself.
 *
 *  @author Rodrigo Fonseca
 *  @author Philip Levis (added trickle-like updates)
 *  Acknowledgment: based on MintRoute, MultiHopLQI, BVR tree construction, Berkeley's MTree
 *                           
 *  @date   $Date: 2010/02/04 07:31:46 $
 *  @see Net2-WG
 */

generic module CtpRoutingEngineP(uint8_t routingTableSize, uint32_t minInterval, uint32_t maxInterval) {
    provides {
        interface UnicastNameFreeRouting as Routing;
        interface RootControl;
        interface CtpInfo;
        interface StdControl;
        interface CtpRoutingPacket;
        interface Init;
	interface GetNow<uint16_t> as GetMAX_c;
    } 
    uses {
        interface AMSend as BeaconSend;
        interface Receive as BeaconReceive;
        interface LinkEstimator;
        interface AMPacket;
        interface SplitControl as RadioControl;
        interface Timer<TMilli> as BeaconTimer;
        interface Timer<TMilli> as RouteTimer;
	interface Timer<TMilli> as Timerbackoff;
	interface Timer<TMilli> as Timer1backoff;
	interface Timer<TMilli> as Timer2backoff;
	interface Timer<TMilli> as Timerfinal;
	interface Timer<TMilli> as Timer_if_no_channel;
	//interface Timer<TMilli> as Timer_sendBeacon_on_different_channel;
	interface Timer<TMilli> as Timer_for_lifetime_calc;
        interface Random;
        interface CollectionDebug;
        interface CtpCongestion;
	interface GetNow<uint16_t> as GetVoltageData; //mod
	interface GetNow<uint16_t> as GetRate;
	interface Leds;
	interface CC2420Config;

	interface CompareBit;

    }
}



implementation {

    bool ECNOff = TRUE;

    /* Keeps track of whether the radio is on. No sense updating or sending
     * beacons if radio is off */
    bool radioOn = FALSE;
    /* Controls whether the node's periodic timer will fire. The node will not
     * send any beacon, and will not update the route. Start and stop control this. */
    bool running = FALSE;
    /* Guards the beacon buffer: only one beacon being sent at a time */
    bool sending = FALSE;

    /* Tells updateNeighbor that the parent was just evicted.*/ 
    bool justEvicted = FALSE;

    route_info_t routeInfo;
    bool state_is_root;
    am_addr_t my_ll_addr;

    message_t beaconMsgBuffer;
    ctp_routing_header_t* beaconMsg;

    /* routing table -- routing info about neighbors */
    routing_table_entry routingTable[routingTableSize];
    uint8_t routingTableActive;

    /* statistics */
    uint32_t parentChanges;
    /* end statistics */

    // forward declarations
    void routingTableInit();
    uint8_t routingTableFind(am_addr_t);
    error_t routingTableUpdateEntry(am_addr_t, am_addr_t , uint16_t);
    error_t routingTableEvict(am_addr_t neighbor);
    //async command uint16_t GetMAX_c();

    uint32_t currentInterval = minInterval;
    uint32_t t; 
    bool tHasPassed;
    int INTV=30000;

    int ch, volt, in, num_node=7;
    int voltage_adc, MAX_c, set=0, MAX_tc, MAX_tp;
    int min_bl[8], MAX1=-10, chan_node[8], min_node_bl[8];
    int MAX_p=10; //Maximum number of parents
    int parent_a[10], parent_c[10], parent_v[10];
    int MAX_bl=0, par, find_parent, ETX, can_start=0, HC=999, HC_m=999, HC_array[100];
    int chan_record[50], voltage_record[50];
    double sum_bl[10], sum_min_bl[10];
    int final_channel_assigned=0, received_packet=0, NP=0, Beacon_channel=0, sendBeacon_on_different_channel=0, channel_index=0;
    int HC_m_next=999, count_min_hp=0, count_next_min_hp=0;
    int percent_cap;
    int HC_m_text=99;
    //for (in=0;in<num_chan;in++) { min_bl[in]=999; }
    ctp_routing_header_t* getHeader(message_t* ONE m);

    void chooseAdvertiseTime() {
       t = currentInterval;
       t /= 2;
       t += call Random.rand32() % t;
       tHasPassed = FALSE;
       call BeaconTimer.stop();
       if(final_channel_assigned==1) { t=INTV;/*minInterval=5000;*/ /*maxInterval=5000;*/ }//Added on Tuesday
       call BeaconTimer.startOneShot(t);
    }

    void resetInterval() {
      currentInterval = minInterval;
      if(final_channel_assigned==1) currentInterval = INTV;//Added on Tuesday
      chooseAdvertiseTime();
    }

    void decayInterval() {
        currentInterval *= 2;
        if (currentInterval > maxInterval) {
          currentInterval = maxInterval;
        }
        if(final_channel_assigned==1) currentInterval = INTV;//Added on Tuesday
      chooseAdvertiseTime();
    }

    void remainingInterval() {
       uint32_t remaining = currentInterval;
       remaining -= t;
       tHasPassed = TRUE;
       call BeaconTimer.startOneShot(remaining);
    }

    command error_t Init.init() {
        uint8_t maxLength;
        radioOn = FALSE;
        running = FALSE;
        parentChanges = 0;
        state_is_root = 0;
        routeInfoInit(&routeInfo);
        routingTableInit();
        my_ll_addr = call AMPacket.address();
        beaconMsg = call BeaconSend.getPayload(&beaconMsgBuffer, call BeaconSend.maxPayloadLength());
        maxLength = call BeaconSend.maxPayloadLength();
        dbg("TreeRoutingCtl","TreeRouting initialized. (used payload:%d max payload:%d!\n", 
              sizeof(beaconMsg), maxLength);
        return SUCCESS;
    }

    command error_t StdControl.start() {
      //start will (re)start the sending of messages
	for (in=0;in<num_chan;in++) { min_bl[in]=999; chan_node[in]=9999;	 min_node_bl[in]=999;}  //Added
	for (in=0;in<num_node;in++) { HC_array[in]=999; }  //Added
	for (in=0;in<MAX_p;in++) { parent_a[in]=999; parent_c[in]=999; parent_v[in]=999;	sum_bl[in]=0;	sum_min_bl[in]=0;}  //Added

      if (!running) {
	running = TRUE;
	resetInterval();
	call RouteTimer.startPeriodic(BEACON_INTERVAL);
	//if(state_is_root!=1){
		call Timer2backoff.startOneShot(8000); //Added
		call Timerfinal.startOneShot(180000); //Added 180000
		call Timer_if_no_channel.startPeriodic(190000); //Added
		//call Timer_sendBeacon_on_different_channel.startPeriodic(200000); //Added
		//call Timer_for_lifetime_calc.startPeriodic(60000); //Added (1 minute timer)
		call Timer_for_lifetime_calc.startPeriodic(60000); //Added (1 minute timer)
	//}
	Life=call GetVoltageData.getNow();
	Life=100; //Should be removed for actual code
	if(TOS_NODE_ID==3)	Life=50;  //Should be removed for actual code
	dbg("TreeRoutingCtl","%s running: %d radioOn: %d\n", __FUNCTION__, running, radioOn);
      }     
      return SUCCESS;
    }




	int battery_life_cal(){
	int v, pc;
	int32_t II;
	int soc;
        int Life_prev;

	Life_prev=Life;
	/*I_Rt=2; T_Rt=14;
	I_Dt=2; T_Dt=14;
	I_p=8;   T_p=3;
	I_Rr=2; T_Rr=14;
	I_Dr=2; T_Dr=14;
	I_s=7.5; T_s=112;*/

	//rate_RUI_p=5; rate_FWD_p=5; rate_RUI_RECV_p=5; rate_OVERHEAR_p=5;
	//I = (rate_RUI_p*I_Rt*T_Rt + rate_FWD_p*I_Dt*T_Dt + rate_RUI_RECV_p*I_Rt*T_Rt + rate_OVERHEAR_p*I_Dt*T_Dt + I_s*T_s/(double)T_Dt + 8*I_p*T_p)/(double)(10.0);
	I = (0.001*rate_RUI_p*I_Rt*T_Rt + 0.001*rate_FWD_p*I_Dt*T_Dt + 0.001*rate_RUI_RECV_p*I_Rt*T_Rt + 0.001*rate_OVERHEAR_p*I_Dt*T_Dt + 0.001*I_s*T_s/(double)T_Dt 		+ 0.001*8*I_p*T_p);
	//I = (rate_RUI_p*I_Rt*T_Rt + rate_FWD_p*I_Dt*T_Dt + rate_RUI_RECV_p*I_Rt*T_Rt + rate_OVERHEAR_p*I_Dt*T_Dt + I_s*T_s/(double)T_Dt +8*I_p*T_p)/1000;
	v = call GetVoltageData.getNow();
	if(v>=482)	v=481;
	//cap=(482-v)/((double)0.65);
	//cap=100*v/(double)482;
	//cap=5000;
	//Life=cap/I;
	//pc = 100*(482- call GetVoltageData.getNow())/65; //Added for lifetime calc
	pc = 100*(482- v)/65; //Added for lifetime calc
	//Life=pc/I;
	//soc=55000/I;
        //pc=100;
	pc=100;
	soc=(550*pc)/I;
	//Life=(1000*1000)/(pc*I);
	//Life=1000*soc/pc;
	//Life=pc*soc;
	Life=soc;

	if(TOS_NODE_ID==3)	Life=Life/2;
	//Life=I;
	//Life=I*60/24.0;

	//Life=I;

	//Life = cap*60*1000/(I*24.0);
	//return Life;
	//II = rate_RUI_p*I_Rt*T_Rt/10 + rate_FWD_p*I_Dt*T_Dt/10 + rate_RUI_RECV_p*I_Rt*T_Rt/10 + rate_OVERHEAR_p*I_Dt*T_Dt/10 + (I_s*T_s/(double)T_Dt +8*I_p*T_p)/1000;
	//percent_cap=soc;

	Life=0.5*Life_prev + 0.5*Life;
	percent_cap=Life;
	
	if(final_channel_assigned==0)		Life = call GetVoltageData.getNow();
	
	if(final_channel_assigned==0) {
		Life=100; //Should be removed for actual code
		if(TOS_NODE_ID==3)	Life=50;
	}
	return Life; //Working

	}

	event void Timer_for_lifetime_calc.fired()
	{

		rate_RUI_p = rate_RUI;
		rate_FWD_p = rate_FWD;
		rate_RUI_RECV_p = rate_RUI_RECV;
		rate_OVERHEAR_p = rate_OVERHEAR;

		rate_RUI = 0;
		rate_FWD = 0;
		rate_RUI_RECV = 0;
		rate_OVERHEAR = 0;
		Life=battery_life_cal();
	}


	

	int current_consumed(){

	C = 0.001*event_RUI*I_Rt*T_Rt + 0.001*event_FWD*I_Dt*T_Dt + 0.001*event_RUI_RECV*I_Rt*T_Rt + 0.001*event_OVERHEAR*I_Dt*T_Dt + 0.001*I_s*T_s/(double)T_Dt + 		0.001*8*I_p*T_p;
	return C;

	}

	event void Timer_if_no_channel.fired()
  	{
		int j, choose_random_channel;
		double random1;  
		if(received_packet==0 && TOS_NODE_ID!=0 && HC_m!=1){
			random1=call Random.rand16();
			random1=random1/(double)65535.0;
			choose_random_channel=(int)(num_chan*random1);
			call Leds.set(choose_random_channel+1);
			choose_random_channel=2*choose_random_channel+11;
			//choose_random_channel=11;
			MAX_c=choose_random_channel;
			if(choose_random_channel!=call CC2420Config.getChannel() && SINGLE_CHAN==0){
				call CC2420Config.setChannel(choose_random_channel);
				call CC2420Config.sync();
			}
		}
		received_packet=0;
		call Timer_if_no_channel.stop();
    		call Timer_if_no_channel.startPeriodic(60000);    
    
	
  	}


	event void Timerbackoff.fired()
  	{
		int j;  
		//if(count_min_hp<30)	HC_m = HC_m_next; //Added on Monday
		//call Leds.set(4);  		
		dbg("BlinkC", "Timer backoff fired @ %s.\n", sim_time_string());
    		printf("\n\rTimer backoff fired");
    		printfflush();
		if(set==0){    	
			for(j=0;j<num_chan;j++){  
				if(min_bl[j]==999){
    					MAX_c=j*2+11;
					break;
				}
				if(min_bl[j]!=999 && min_bl[j]>MAX1){
					MAX1=min_bl[j];	MAX_c=j*2+11;
				}
    			} 
   			if(MAX_c<11 || MAX_c>25)	MAX_c=22;
                	call Leds.set(0.5*(MAX_c-11)+1);
			set=1;
		}
		//call Leds.set(5);
		//call Leds.set(HC_m);	
		call Timerbackoff.stop();	
  	}


	event void Timerfinal.fired()
  	{
		//MAX_c=11;
    		if(HC_m!=0 && MAX_c!=call CC2420Config.getChannel() && SINGLE_CHAN==0){
				final_channel_assigned=1;	
				//INTV=5000;		
				//event_RUI_RECV=0; //Added on Wednesday
				if(TOS_NODE_ID==1 || TOS_NODE_ID==3 || TOS_NODE_ID==5)	MAX_c=11; //Added on Wednesday
				if(TOS_NODE_ID==2 || TOS_NODE_ID==4 || TOS_NODE_ID==5)	MAX_c=13; //Added on Wednesday
				call CC2420Config.setChannel(MAX_c);
				call CC2420Config.sync();
				call Leds.set(7);
		}	
		call Timerfinal.stop();	
  	}


	event void Timer2backoff.fired()
  	{
    		can_start=1;
		//if(count_min_hp<30 && HC_m_next!=999)	HC_m = HC_m_next; //Added on Monday
		//if(1.5*count_min_hp<count_next_min_hp)	HC_m = HC_m_next; //Added on Monday
		if(count_min_hp<20 && count_next_min_hp>20)	HC_m = HC_m_next; //Added on Monday
		HC_m_text=count_min_hp;
		HC_m_text=HC_m;
		if(can_start==1){	
			voltage_adc=call GetVoltageData.getNow();	//voltage_adc=5;
			if(HC_m!=0){
				//if(routeInfo.parent==0)	call Timerbackoff.startOneShot( 10*voltage_adc+call Random.rand16()); //Added
				//if(routeInfo.parent==0)	call Timerbackoff.startOneShot( 0.25*(call Random.rand16())); //Added
				if(HC_m==1)	call Timerbackoff.startOneShot( 0.25*(call Random.rand16())); //Added
				//call Leds.set(routeInfo.parent);	
				else {//ETX=getHeader(msg)->etx;
					/*if(HC_m==999) call Leds.set(7);	
					else call Leds.set(HC_m);*/			
					//call Timer1backoff.startOneShot(100*HC_m + 1000*voltage_adc); //Added
					call Timer1backoff.startOneShot(5000*HC_m + 0.25*(call Random.rand16())); //Added
				}
			}
		}
		call Timer2backoff.stop();		
  	}

	int chan_exist(int k){
		int got_chan=0, j;		
		for(j=0;j<MAX_p;j++){
			//if(parent_c[j]!=999)	//call Leds.set(parent_a[j]+1);		
				//call Leds.set(0.5*(parent_c[j]-11)+1);	
				//call Leds.set(k);	
			//if(k==1)	call Leds.set(6);
			if(parent_c[j]==k*2+11){  
				got_chan=1;
				break;
			}
		}
		return got_chan;
	}

	int choose_channel(){

		int j, sum_min_bat=0, sum_bat=0;
		double random1;    		
		
		for(j=0;j<num_chan;j++){
				sum_min_bl[j]=0;
				if(min_bl[j]!=999)	sum_min_bat=sum_min_bat+min_bl[j];
				sum_min_bl[j]=sum_min_bat;
		}

		for(j=0;j<num_chan;j++){
			sum_min_bl[j]=sum_min_bl[j]/(double)sum_min_bat;
		}

		random1=call Random.rand16();
		random1=random1/(double)65535.0;
		for(j=0;j<num_chan;j++){
			if(random1<=sum_min_bl[j]){
				MAX_tc=j*2+11;
				break;
			}
		}

		if(MAX_tc<11 || MAX_tc>25)	{ 	
			MAX_tc=22;
                	call Leds.set(0.5*(MAX_tc-11)+1);
		}
		call Leds.set(0.5*(MAX_tc-11)+1);

		find_parent=0;		
		for(j=0;j<MAX_p;j++){
			if(parent_c[j]==MAX_tc ){
				sum_bl[j]=0;
				sum_bat=sum_bat+parent_v[j];
				sum_bl[j]=sum_bat;
			}	
		}


		for(j=0;j<MAX_p;j++){
			sum_bl[j]=sum_bl[j]/(double)sum_bat;
		}

		random1=call Random.rand16();
		random1=random1/(double)65535.0;
		for(j=0;j<MAX_p;j++){
			if(random1<=sum_bl[j]){
				find_parent=1;
				par=parent_a[j];
				break;
			}
		}
		MAX_tp=par;
		//par=MAX_tc;
		//par=100*random1;
		//par=100*sum_min_bl[1];
		//par=min_bl[0];
	}
		

	event void Timer1backoff.fired()
  	{
		int j, sum_bat=0;
		double random1; 
		//if(count_min_hp<2)	HC_m = HC_m_next; //Added on Monday   		
		if(set==0){    		
		A:		
		for(j=0;j<num_chan;j++){
			if(chan_exist(j)==1){ 
				/*if(min_bl[j]==999){	call Leds.set(2);	
    					MAX_c=j*2+11;
					break;
				}*/
				if(min_bl[j]!=999 && min_bl[j]>MAX1){		
					MAX1=min_bl[j];	MAX_c=j*2+11;
					
				}
    			} 
		}

		

   		if(MAX_c<11 || MAX_c>25)	{ MAX_c=22;
                call Leds.set(0.5*(MAX_c-11)+1);
		}
		call Leds.set(0.5*(MAX_c-11)+1);
			

		find_parent=0;		
		for(j=0;j<MAX_p;j++){
			if(parent_c[j]==MAX_c /*&& parent_v[j]>MAX_bl*/){
				//find_parent=1;				
				//MAX_bl=parent_v[in];	par=parent_a[in];
				sum_bat=sum_bat+parent_v[j];
				sum_bl[j]=sum_bat;
			}	
		}


		for(j=0;j<MAX_p;j++){
			sum_bl[j]=sum_bl[j]/(double)sum_bat;
		}
		//random1=rand()/(double)RAND_MAX;
		random1=call Random.rand16();
		random1=random1/(double)65535.0;
		for(j=0;j<MAX_p;j++){
			if(random1<=sum_bl[j]){
				find_parent=1;
				par=parent_a[j];
				break;
			}
		}

		//if(find_parent==0){	min_bl[(int)(0.5*(MAX_c-11))]=999;	call Leds.set(5); goto A;}
		set=1;
		}	
		call Timer1backoff.stop();
  	}

    command error_t StdControl.stop() {
        running = FALSE;
        dbg("TreeRoutingCtl","%s running: %d radioOn: %d\n", __FUNCTION__, running, radioOn);
        return SUCCESS;
    } 

    event void RadioControl.startDone(error_t error) {
        radioOn = TRUE;
        dbg("TreeRoutingCtl","%s running: %d radioOn: %d\n", __FUNCTION__, running, radioOn);
        if (running) {
            uint16_t nextInt;
            nextInt = call Random.rand16() % BEACON_INTERVAL;
            nextInt += BEACON_INTERVAL >> 1;
            call BeaconTimer.startOneShot(nextInt);
        }
    } 

    event void RadioControl.stopDone(error_t error) {
        radioOn = FALSE;
        dbg("TreeRoutingCtl","%s running: %d radioOn: %d\n", __FUNCTION__, running, radioOn);
    }

    /* Is this quality measure better than the minimum threshold? */
    // Implemented assuming quality is EETX
    bool passLinkEtxThreshold(uint16_t etx) {
        return (etx < ETX_THRESHOLD);
    }


    /* updates the routing information, using the info that has been received
     * from neighbor beacons. Two things can cause this info to change: 
     * neighbor beacons, changes in link estimates, including neighbor eviction */
    task void updateRouteTask() {
        uint8_t i;
        routing_table_entry* entry;
        routing_table_entry* best;
        uint16_t minEtx;
        uint16_t currentEtx;
        uint16_t linkEtx, pathEtx;
	int j;


        if (state_is_root)
            return;
       
        best = NULL;
        /* Minimum etx found among neighbors, initially infinity */
        minEtx = MAX_METRIC;
        /* Metric through current parent, initially infinity */
        currentEtx = MAX_METRIC;

        dbg("TreeRouting","%s\n",__FUNCTION__);

        /* Find best path in table, other than our current */
        for (i = 0; i < routingTableActive; i++) {
            entry = &routingTable[i];


	// Avoid bad entries and 1-hop loops
            if (entry->info.parent == INVALID_ADDR || entry->info.parent == my_ll_addr) {
              dbg("TreeRouting", 
                  "routingTable[%d]: neighbor: [id: %d parent: %d  etx: NO ROUTE]\n",  
                  i, entry->neighbor, entry->info.parent);
              continue;
            }

            linkEtx = call LinkEstimator.getLinkQuality(entry->neighbor);
            dbg("TreeRouting", 
                "routingTable[%d]: neighbor: [id: %d parent: %d etx: %d retx: %d]\n",  
                i, entry->neighbor, entry->info.parent, linkEtx, entry->info.etx);
            pathEtx = linkEtx + entry->info.etx;

		if(HC_m==1 && entry->info.parent==0)	pathEtx=-100; //Added on Sat
		if(HC_m==1 && entry->neighbor==0){	pathEtx=-100; //Added on Sat
					entry->info.parent=0;	entry->info.etx=-100;	entry->info.congested=FALSE;}	
		if(HC_m==1 && entry->neighbor!=0)	entry->info.congested=TRUE;
	    /* Operations specific to the current parent */
            if (entry->neighbor == routeInfo.parent) {
                dbg("TreeRouting", "   already parent.\n");
                currentEtx = pathEtx;
                /* update routeInfo with parent's current info */
                atomic {
                    routeInfo.etx = entry->info.etx;
                    routeInfo.congested = entry->info.congested;
                }
                continue;
            }
            /* Ignore links that are congested */
            if (entry->info.congested)
                continue;
            /* Ignore links that are bad */
            if (!passLinkEtxThreshold(linkEtx)) {
              dbg("TreeRouting", "   did not pass threshold.\n");
              continue;
            }
            
            if (pathEtx < minEtx) {
	      dbg("TreeRouting", "   best is %d, setting to %d\n", pathEtx, entry->neighbor);
                minEtx = pathEtx;
                best = entry;
            }  


		//Should be removed
		/*MAX_p=2;
		if(TOS_NODE_ID==3)	{	parent_a[0]=1;	parent_a[1]=2;	parent_c[0]=chan_record[parent_a[0]]; parent_v[1]=voltage_record[parent_a[1]]; }
		if(TOS_NODE_ID==4)	{	parent_a[0]=1;	parent_a[1]=2;	parent_c[0]=chan_record[parent_a[0]]; parent_v[1]=voltage_record[parent_a[1]]; }
		if(TOS_NODE_ID==5)	{	parent_a[0]=3;	parent_a[1]=4;	parent_c[0]=chan_record[parent_a[0]]; parent_v[1]=voltage_record[parent_a[1]]; }
		if(TOS_NODE_ID==6)	{	parent_a[0]=3;	parent_a[1]=4;	parent_c[0]=chan_record[parent_a[0]]; parent_v[1]=voltage_record[parent_a[1]]; }*/

	   //Included
		for(j=0;j<MAX_p;j++){
			//if(entry->info.parent==parent_a[in])	break;
			if(entry->neighbor==parent_a[j] || HC_m <= HC_array[entry->neighbor])	break;
			else if(parent_a[j]==999){
				//parent_a[in]=entry->info.parent; call Leds.set(parent_a[in]+4);
				parent_a[j]=entry->neighbor; //call Leds.set(parent_a[in]+3);
				parent_c[j]=chan_record[parent_a[j]]; //call Leds.set(0.5*(parent_c[in]-11)+1);
				parent_v[j]=voltage_record[parent_a[j]];
				break;
			}
		}

		
        }

        //call CollectionDebug.logEventDbg(NET_C_DBG_3, routeInfo.parent, currentEtx, minEtx);  

        /* Now choose between the current parent and the best neighbor */
        /* Requires that: 
            1. at least another neighbor was found with ok quality and not congested
            2. the current parent is congested and the other best route is at least as good
            3. or the current parent is not congested and the neighbor quality is better by 
               the PARENT_SWITCH_THRESHOLD.
          Note: if our parent is congested, in order to avoid forming loops, we try to select
                a node which is not a descendent of our parent. routeInfo.ext is our parent's
                etx. Any descendent will be at least that + 10 (1 hop), so we restrict the 
                selection to be less than that.
        */

	if (minEtx != MAX_METRIC) {
            if (currentEtx == MAX_METRIC ||
                (routeInfo.congested && (minEtx < (routeInfo.etx + 10))) ||
                minEtx + PARENT_SWITCH_THRESHOLD < currentEtx) {
                // routeInfo.metric will not store the composed metric.
                // since the linkMetric may change, we will compose whenever
                // we need it: i. when choosing a parent (here); 
                //            ii. when choosing a next hop
                parentChanges++;

		dbg("TreeRouting","Changed parent. from %d to %d\n", routeInfo.parent, best->neighbor);
                call CollectionDebug.logEventDbg(NET_C_TREE_NEW_PARENT, best->neighbor, best->info.etx, minEtx);
                call LinkEstimator.unpinNeighbor(routeInfo.parent);
                call LinkEstimator.pinNeighbor(best->neighbor);
                call LinkEstimator.clearDLQ(best->neighbor);

		atomic {
                    routeInfo.parent = best->neighbor;
                    routeInfo.etx = best->info.etx;
                    routeInfo.congested = best->info.congested;
                }
		if (currentEtx - minEtx > 20) {
		  call CtpInfo.triggerRouteUpdate();
		}
            }
        }    

        /* Finally, tell people what happened:  */
        /* We can only loose a route to a parent if it has been evicted. If it hasn't 
         * been just evicted then we already did not have a route */
        if (justEvicted && routeInfo.parent == INVALID_ADDR) 
            signal Routing.noRoute();
        /* On the other hand, if we didn't have a parent (no currentEtx) and now we
         * do, then we signal route found. The exception is if we just evicted the 
         * parent and immediately found a replacement route: we don't signal in this 
         * case */
        else if (!justEvicted && 
                  currentEtx == MAX_METRIC &&
                  minEtx != MAX_METRIC)
            signal Routing.routeFound();
        justEvicted = FALSE;

   }

    
    

    /* send a beacon advertising this node's routeInfo */
    // only posted if running and radioOn
    task void sendBeaconTask() {
        error_t eval;

	/*if (TOS_NODE_ID!=0 && MAX_c!=0 && call CtpInfo.get_chan_assigned()==1 && MAX_c!=call CC2420Config.getChannel())		{ 
			call CC2420Config.setChannel(MAX_c);	
			call CC2420Config.sync();
			//call Leds.set(7);
		}*/

        if (sending) {
            return;
        }


	rate_RUI++; //Added for lifetime calc
	event_RUI++; //Added for lifetime calc

	/*if (TOS_NODE_ID!=0 && MAX_c!=0 && call CtpInfo.get_chan_assigned()==1 && MAX_c!=call CC2420Config.getChannel() && SINGLE_CHAN==0)		{ 
			call CC2420Config.setChannel(MAX_c);	
			call CC2420Config.sync();
			//call Leds.set(7);
		}
	if (TOS_NODE_ID!=0 && sendBeacon_on_different_channel==1 && Beacon_channel!=call CC2420Config.getChannel() && final_channel_assigned==1 && SINGLE_CHAN==0)		{ 
			call CC2420Config.setChannel(Beacon_channel);	
			call CC2420Config.sync();
			//beaconMsg -> voltage = 290;
			sendBeacon_on_different_channel=0;
			//call Leds.set(7);
	}*/

	if (TOS_NODE_ID!=0 && /*sendBeacon_on_different_channel==1 && */Beacon_channel!=call CC2420Config.getChannel() && final_channel_assigned==1 && SINGLE_CHAN==0)		{ 
			sendBeacon_on_different_channel=1;
			Beacon_channel=2*channel_index+11;
			if(channel_index==num_chan-1)	channel_index=0;
			channel_index++;
			call CC2420Config.setChannel(Beacon_channel);	
			call CC2420Config.sync();
	}
  	
        beaconMsg->options = 0;

        /* Congestion notification: am I congested? */
        if (call CtpCongestion.isCongested()) {
            beaconMsg->options |= CTP_OPT_ECN;
        }

	beaconMsg->parent = routeInfo.parent;
        if (state_is_root) {
            beaconMsg->etx = routeInfo.etx;
        }
        else if (routeInfo.parent == INVALID_ADDR) {
            beaconMsg->etx = routeInfo.etx;
            beaconMsg->options |= CTP_OPT_PULL;
        } else {
            beaconMsg->etx = routeInfo.etx + call LinkEstimator.getLinkQuality(routeInfo.parent);
        }
        beaconMsg->chan = MAX_c; //Added
	beaconMsg->hop_2_sink = HC_m;
	beaconMsg->rt_p = 1;
	//beaconMsg->voltage = call GetVoltageData.getNow(); //Added for lifetime calc
	//beaconMsg->voltage = battery_life_cal();
	beaconMsg->voltage = Life;
	//beaconMsg->voltage = 100*(482- call GetVoltageData.getNow())/65; //Added for lifetime calc
	//beaconMsg->voltage = 100;
	//percent_cap = (482 - call GetVoltageData.getNow());   // equation of the form y = mx + c i.e. y = -0.65x + 482
          //  	percent_cap = (percent_cap *65)/100;   //capacity on the scale of 100
           
	//percent_cap=beaconMsg->voltage;
	//percent_cap=call GetVoltageData.getNow();
	//if(beaconMsg->voltage==0)	call Leds.set(4);
	//percent_cap=battery_life_cal();


        dbg("TreeRouting", "%s parent: %d etx: %d\n",
                  __FUNCTION__,
                  beaconMsg->parent, 
                  beaconMsg->etx);
        call CollectionDebug.logEventRoute(NET_C_TREE_SENT_BEACON, beaconMsg->parent, 0, beaconMsg->etx);

        eval = call BeaconSend.send(AM_BROADCAST_ADDR, 
                                    &beaconMsgBuffer, 
                                    sizeof(ctp_routing_header_t));
        if (eval == SUCCESS) {
            sending = TRUE;
        } else if (eval == EOFF) {
            radioOn = FALSE;
            dbg("TreeRoutingCtl","%s running: %d radioOn: %d\n", __FUNCTION__, running, radioOn);
        }
    }

    event void BeaconSend.sendDone(message_t* msg, error_t error) {


	ETX=getHeader(msg)->etx;	
	
        if ((msg != &beaconMsgBuffer) || !sending) {
            //something smells bad around here
            return;
        }
        sending = FALSE;
    }

    event void RouteTimer.fired() {
      if (radioOn && running) {
         post updateRouteTask();
      }
    }
      
    event void BeaconTimer.fired() {
      if (radioOn && running) {
        if (!tHasPassed) {
          post updateRouteTask(); //always send the most up to date info
          post sendBeaconTask();
          dbg("RoutingTimer", "Beacon timer fired at %s\n", sim_time_string());
          remainingInterval();
        }
        else {
          decayInterval();
        }
      }
    }


	/*event void Timer_sendBeacon_on_different_channel.fired()
  	{
		int j, choose_random_channel;
		double random1;  
		sendBeacon_on_different_channel=1;
		Beacon_channel=2*channel_index+11;
		if(channel_index==num_chan-1)	channel_index=0;
		channel_index++;
		post sendBeaconTask();
		call Timer_sendBeacon_on_different_channel.stop();
    		call Timer_sendBeacon_on_different_channel.startPeriodic(800);    
    
	
  	}*/

    ctp_routing_header_t* getHeader(message_t* ONE m) {
      return (ctp_routing_header_t*)call BeaconSend.getPayload(m, call BeaconSend.maxPayloadLength());
    }
    
    
    /* Handle the receiving of beacon messages from the neighbors. We update the
     * table, but wait for the next route update to choose a new parent */
    event message_t* BeaconReceive.receive(message_t* msg, void* payload, uint8_t len) {
        am_addr_t from;
        ctp_routing_header_t* rcvBeacon;
        bool congested;
	int i,j;

	received_packet++;

	rate_RUI_RECV++; //Added for lifetime calc
	event_RUI_RECV++; //Added for lifetime calc

	//call Leds.led2Toggle();
        
	//ch=call CtpPacket.getchan(msg);
	ch=getHeader(msg)->chan;
	ch=0.5*(ch-11);
	//volt=call CtpPacket.getvoltage(msg);
	volt=getHeader(msg)->voltage;
	HC = 1 + getHeader(msg) -> hop_2_sink;
	if(HC < HC_m && HC!=HC_m_next)	HC_m=HC;

	if(HC < HC_m_next && HC != HC_m)	HC_m_next=HC; //Added on Monday
	if(HC_m == HC)	count_min_hp++;
	else if(HC_m_next ==HC)	count_next_min_hp++;

	//Should be removed in original code	
	if(TOS_NODE_ID==1 || TOS_NODE_ID==2)	HC_m=1;
	if(TOS_NODE_ID==3 || TOS_NODE_ID==4)	HC_m=2;
	if(TOS_NODE_ID==5 || TOS_NODE_ID==6)	HC_m=3;

	//call GetVoltageData.getNow();
	/*if(call CtpPacket.getOrigin(msg) == 1) call Leds.set(10);
	call Leds.set(call CtpPacket.getOrigin(msg));
	call Leds.set(10);*/
	//if(volt >= 482) 			//ADC = 482 = 2.60 volts
        //    		volt = 481;	
	if((volt<min_bl[ch] || min_bl[ch]==999) && volt!=0)	{ 
		min_bl[ch]=volt; 
		min_node_bl[ch]= call AMPacket.source(msg);
		/*call Leds.set(7);*/ 
	} //Commented on Thrusday
	//chan_chosen[getHeader(msg)->origin]=getHeader(msg)->chan;

	if(min_node_bl[ch]==call AMPacket.source(msg))		min_bl[ch]=volt; 

	// Received a beacon, but it's not from us.
        if (len != sizeof(ctp_routing_header_t)) {
          dbg("LITest", "%s, received beacon of size %hhu, expected %i\n",
                     __FUNCTION__, 
                     len,
                     (int)sizeof(ctp_routing_header_t));
              
          return msg;
        }
        
        //need to get the am_addr_t of the source
        from = call AMPacket.source(msg);
        rcvBeacon = (ctp_routing_header_t*)payload;

	//Included
	chan_record[from]=getHeader(msg)->chan;
	voltage_record[from]=getHeader(msg)->voltage;
	HC_array[from]=getHeader(msg)->hop_2_sink;

	//Should be removed in original code	
	HC_array[3]=2;	HC_array[4]=2;	HC_array[5]=3;	HC_array[6]=3;

	/*chan_node[from]=ch;
	chan_node[0]=9999;
	//Added on Thrusday
	for(i=0;i<num_chan;i++)		min_bl[i]=999;
	for(i=0;i<num_chan;i++){
		for(j=1;j<num_node;j++){
			if(i==chan_node[j] && j!=TOS_NODE_ID){
				if(min_bl[i]==999 || voltage_record[j]<min_bl[i]){
					min_bl[i]=voltage_record[j];
				}
			}
		}
	}*/

        congested = call CtpRoutingPacket.getOption(msg, CTP_OPT_ECN);

        dbg("TreeRouting","%s from: %d  [ parent: %d etx: %d]\n",
            __FUNCTION__, from, 
            rcvBeacon->parent, rcvBeacon->etx);

        //update neighbor table
        if (rcvBeacon->parent != INVALID_ADDR) {

            /* If this node is a root, request a forced insert in the link
             * estimator table and pin the node. */
            if (rcvBeacon->etx == 0) {
                dbg("TreeRouting","from a root, inserting if not in table\n");
                call LinkEstimator.insertNeighbor(from);
                call LinkEstimator.pinNeighbor(from);
            }
            //TODO: also, if better than my current parent's path etx, insert

            routingTableUpdateEntry(from, rcvBeacon->parent, rcvBeacon->etx);
            call CtpInfo.setNeighborCongested(from, congested);
        }

        if (call CtpRoutingPacket.getOption(msg, CTP_OPT_PULL)) {
              resetInterval();
        }
        return msg;
    }


    /* Signals that a neighbor is no longer reachable. need special care if
     * that neighbor is our parent */
    event void LinkEstimator.evicted(am_addr_t neighbor) {
	if((neighbor!=0 && HC_m==1) || HC_m>0){	//Added on Sat        	
		routingTableEvict(neighbor);
        	dbg("TreeRouting","%s\n",__FUNCTION__);
        	if (routeInfo.parent == neighbor) {
            	routeInfoInit(&routeInfo);
            	justEvicted = TRUE;
            	post updateRouteTask();
        	}
	}
    }

    /* Interface UnicastNameFreeRouting */
    /* Simple implementation: return the current routeInfo */
    command am_addr_t Routing.nextHop() {
	return routeInfo.parent;    
    }
    command bool Routing.hasRoute() {
	return (routeInfo.parent != INVALID_ADDR);
    }
   
    /* CtpInfo interface */
    command error_t CtpInfo.getParent(am_addr_t* parent) {
        if (parent == NULL) 
            return FAIL;
        if (routeInfo.parent == INVALID_ADDR)    
            return FAIL;
        *parent = routeInfo.parent;
        return SUCCESS;
    }

    command error_t CtpInfo.getEtx(uint16_t* etx) {
        if (etx == NULL) 
            return FAIL;
        if (routeInfo.parent == INVALID_ADDR)    
            return FAIL;
	if (state_is_root == 1) {
	  *etx = 0;
	} else {
	  *etx = routeInfo.etx + call LinkEstimator.getLinkQuality(routeInfo.parent);
	}

	return SUCCESS;
    }

    command void CtpInfo.recomputeRoutes() {
      post updateRouteTask();
    }

    command void CtpInfo.triggerRouteUpdate() {
      resetInterval();
     }

    command void CtpInfo.triggerImmediateRouteUpdate() {
      resetInterval();
    }

    command void CtpInfo.setNeighborCongested(am_addr_t n, bool congested) {
        uint8_t idx;    
        if (ECNOff)
            return;
        idx = routingTableFind(n);
        if (idx < routingTableActive) {
            routingTable[idx].info.congested = congested;
        }
        if (routeInfo.congested && !congested) 
            post updateRouteTask();
        else if (routeInfo.parent == n && congested) 
            post updateRouteTask();
    }

    command bool CtpInfo.isNeighborCongested(am_addr_t n) {
        uint8_t idx;    

        if (ECNOff) 
            return FALSE;

        idx = routingTableFind(n);
        if (idx < routingTableActive) {
            return routingTable[idx].info.congested;
        }
        return FALSE;
    }
    
    /* RootControl interface */
    /** sets the current node as a root, if not already a root */
    /*  returns FAIL if it's not possible for some reason      */
    command error_t RootControl.setRoot() {
        bool route_found = FALSE;
        route_found = (routeInfo.parent == INVALID_ADDR);
        atomic {
            state_is_root = 1;
            routeInfo.parent = my_ll_addr; //myself
            routeInfo.etx = 0;
	    HC_m=0;
	}
        if (route_found) 
            signal Routing.routeFound();
        dbg("TreeRouting","%s I'm a root now!\n",__FUNCTION__);
        call CollectionDebug.logEventRoute(NET_C_TREE_NEW_PARENT, routeInfo.parent, 0, routeInfo.etx);
        return SUCCESS;
    }

    command error_t RootControl.unsetRoot() {
        atomic {
            state_is_root = 0;
            routeInfoInit(&routeInfo);
        }
        dbg("TreeRouting","%s I'm not a root now!\n",__FUNCTION__);
        post updateRouteTask();
        return SUCCESS;
    }

    command bool RootControl.isRoot() {
        return state_is_root;
    }

    default event void Routing.noRoute() {
    }
    
    default event void Routing.routeFound() {
    }


  /* The link will be recommended for insertion if it is better* than some
   * link in the routing table that is not our parent.
   * We are comparing the path quality up to the node, and ignoring the link
   * quality from us to the node. This is because of a couple of things:
   *   1. we expect this call only for links with white bit set
   *   2. we are being optimistic to the nodes in the table, by ignoring the
   *      1-hop quality to them (which means we are assuming it's 1 as well)
   *      This actually sets the bar a little higher for replacement
   *   3. this is faster
   */
    event bool CompareBit.shouldInsert(message_t *msg, void* payload, uint8_t len) {
        
        bool found = FALSE;
        uint16_t pathEtx;
        uint16_t neighEtx;
        int i;
        routing_table_entry* entry;
        ctp_routing_header_t* rcvBeacon;

        if ((call AMPacket.type(msg) != AM_CTP_ROUTING) ||
            (len != sizeof(ctp_routing_header_t))) 
            return FALSE;

        /* 1.determine this packet's path quality */
        rcvBeacon = (ctp_routing_header_t*)payload;

        if (rcvBeacon->parent == INVALID_ADDR)
            return FALSE;
        /* the node is a root, recommend insertion! */
        if (rcvBeacon->etx == 0) {
            return TRUE;
        }
    
        pathEtx = rcvBeacon->etx; // + linkEtx;

        /* 2. see if we find some neighbor that is worse */
        for (i = 0; i < routingTableActive && !found; i++) {
            entry = &routingTable[i];
            //ignore parent, since we can't replace it
            if (entry->neighbor == routeInfo.parent)
                continue;
            neighEtx = entry->info.etx;
            found |= (pathEtx < neighEtx); 
        }
        return found;
    }


    /************************************************************/
    /* Routing Table Functions                                  */

    /* The routing table keeps info about neighbor's route_info,
     * and is used when choosing a parent.
     * The table is simple: 
     *   - not fragmented (all entries in 0..routingTableActive)
     *   - not ordered
     *   - no replacement: eviction follows the LinkEstimator table
     */

    void routingTableInit() {
        routingTableActive = 0;
    }

    /* Returns the index of parent in the table or
     * routingTableActive if not found */
    uint8_t routingTableFind(am_addr_t neighbor) {
        uint8_t i;
        if (neighbor == INVALID_ADDR)
            return routingTableActive;
        for (i = 0; i < routingTableActive; i++) {
            if (routingTable[i].neighbor == neighbor)
                break;
        }
        return i;
    }


    error_t routingTableUpdateEntry(am_addr_t from, am_addr_t parent, uint16_t etx)    {
        uint8_t idx;
        uint16_t  linkEtx;
        linkEtx = call LinkEstimator.getLinkQuality(from);

	idx = routingTableFind(from);
        if (idx == routingTableSize) {
            //not found and table is full
            //if (passLinkEtxThreshold(linkEtx))
                //TODO: add replacement here, replace the worst
            //}
            dbg("TreeRouting", "%s FAIL, table full\n", __FUNCTION__);
            return FAIL;
        }
        else if (idx == routingTableActive) {
            //not found and there is space
            if (passLinkEtxThreshold(linkEtx)) {
                atomic {
                    routingTable[idx].neighbor = from;
                    routingTable[idx].info.parent = parent;
                    routingTable[idx].info.etx = etx;
		    routingTable[idx].info.haveHeard = 1;
                    routingTable[idx].info.congested = FALSE;
                    routingTableActive++;
                }
                dbg("TreeRouting", "%s OK, new entry\n", __FUNCTION__);
            } else {
                dbg("TreeRouting", "%s Fail, link quality (%hu) below threshold\n", __FUNCTION__, linkEtx);
            }
        } else {
            //found, just update
            atomic {
                routingTable[idx].neighbor = from;
                routingTable[idx].info.parent = parent;
                routingTable[idx].info.etx = etx;
		        routingTable[idx].info.haveHeard = 1;
            }
            dbg("TreeRouting", "%s OK, updated entry\n", __FUNCTION__);
        }
        return SUCCESS;
    }

    /* if this gets expensive, introduce indirection through an array of pointers */
    error_t routingTableEvict(am_addr_t neighbor) {
        uint8_t idx,i;
        idx = routingTableFind(neighbor);
        if (idx == routingTableActive) 
            return FAIL;
        routingTableActive--;
        for (i = idx; i < routingTableActive; i++) {
            routingTable[i] = routingTable[i+1];    
        } 
        return SUCCESS; 
    }
    /*********** end routing table functions ***************/

    /* Default implementations for CollectionDebug calls.
     * These allow CollectionDebug not to be wired to anything if debugging
     * is not desired. */

  event void CC2420Config.syncDone(error_t error) {
  
  }

    default command error_t CollectionDebug.logEvent(uint8_t type) {
        return SUCCESS;
    }
    default command error_t CollectionDebug.logEventSimple(uint8_t type, uint16_t arg) {
        return SUCCESS;
    }
    default command error_t CollectionDebug.logEventDbg(uint8_t type, uint16_t arg1, uint16_t arg2, uint16_t arg3) {
        return SUCCESS;
    }
    default command error_t CollectionDebug.logEventMsg(uint8_t type, uint16_t msg, am_addr_t origin, am_addr_t node) {
        return SUCCESS;
    }
    default command error_t CollectionDebug.logEventRoute(uint8_t type, am_addr_t parent, uint8_t hopcount, uint16_t etx) {
        return SUCCESS;
    }

    command bool CtpRoutingPacket.getOption(message_t* msg, ctp_options_t opt) {
      return ((getHeader(msg)->options & opt) == opt) ? TRUE : FALSE;
    }

    command void CtpRoutingPacket.setOption(message_t* msg, ctp_options_t opt) {
      getHeader(msg)->options |= opt;
    }

    command void CtpRoutingPacket.clearOption(message_t* msg, ctp_options_t opt) {
      getHeader(msg)->options &= ~opt;
    }

    command void CtpRoutingPacket.clearOptions(message_t* msg) {
      getHeader(msg)->options = 0;
    }

    
    command am_addr_t     CtpRoutingPacket.getParent(message_t* msg) {
      return getHeader(msg)->parent;
    }
    command void          CtpRoutingPacket.setParent(message_t* msg, am_addr_t addr) {
      getHeader(msg)->parent = addr;
    }
    
    command uint16_t      CtpRoutingPacket.getEtx(message_t* msg) {
      return getHeader(msg)->etx;
    }
    command void          CtpRoutingPacket.setEtx(message_t* msg, uint16_t etx) {
      getHeader(msg)->etx = etx;
    }
    
    command void	CtpRoutingPacket.setchan(message_t* msg, uint16_t chan) {
    	getHeader(msg)->chan = chan;
    }

   command void	CtpRoutingPacket.setvoltage(message_t* msg, uint16_t voltage) {
    	getHeader(msg)->voltage = voltage;
    }

    command uint8_t CtpInfo.numNeighbors() {
      return routingTableActive;
    }
    command uint16_t CtpInfo.getNeighborLinkQuality(uint8_t n) {
      return (n < routingTableActive)? call LinkEstimator.getLinkQuality(routingTable[n].neighbor):0xffff;
    }
    command uint16_t CtpInfo.getNeighborRouteQuality(uint8_t n) {
      return (n < routingTableActive)? call LinkEstimator.getLinkQuality(routingTable[n].neighbor) + routingTable[n].info.etx:0xfffff;
    }
    command am_addr_t CtpInfo.getNeighborAddr(uint8_t n) {
      return (n < routingTableActive)? routingTable[n].neighbor:AM_BROADCAST_ADDR;
    }
    
   async command uint16_t GetMAX_c.getNow(){
  	return MAX_c;
  }

  command uint16_t CtpInfo.getchan(){
  	return MAX_c;
  }

  command uint16_t CtpInfo.getchan_random(){
	choose_channel();
  	return MAX_tc;
  }

  command uint16_t CtpInfo.getparent_random(){
	//return battery_life_cal();
	//return event_OVERHEAR;
	//return Life;
	return percent_cap;
	//return current_consumed();
	//return HC_m;
  }

  command uint16_t CtpInfo.getparent_channel_random(int *m){
	choose_channel();
	*m=MAX_tc;
  	return MAX_tp;
  }

  command uint16_t CtpInfo.current_consumed(){
	current_consumed();
	return C;
  }

  command uint16_t CtpInfo.hop_cnt(){
  	return HC_m;
  }

  command uint16_t CtpInfo.hop_cnt_nxt(){
  	//return HC_m;
	return event_RUI_RECV; //Was Uncommented
	//return pkt_trans_in_chan[11];
	//return min_bl[0];
	//return 999;
	//return event_FWD;
	//return HC_m_text;
  }

  command uint16_t CtpInfo.ov_nxt(){
  	//return HC_m;
	//return percent_cap;
	//return pkt_trans_in_chan[13];
	//return min_bl[1];
	return event_OVERHEAR; //Was Uncommented
	//return HC_m_text;
  }

    command uint16_t CtpInfo.forwd_num(){
  	//return HC_m;
	return event_FWD;
	//return HC_m_text;
  }

  command uint16_t CtpInfo.get_chan_assigned(){
  	return final_channel_assigned;
  }


} 
