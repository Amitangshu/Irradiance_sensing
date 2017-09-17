/*
 * Copyright (c) 2006 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE     
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA, 
 * 94704.  Attention:  Intel License Inquiry.
 */

/**
 * @author David Gay
 * @author Kyle Jamieson
 * @author Phil Levis
 */

#ifndef MVIZ_H
#define MVIZ_H

#include "AM.h"

enum {
  /* Default sampling period. */
  //DEFAULT_INTERVAL = 1024*5*60,
  DEFAULT_INTERVAL = 3072,
  AM_MVIZ_MSG = 0x93
};

typedef nx_struct mviz_msg {
  nx_uint16_t version; /* Version of the interval. */
  nx_uint16_t interval; /* Samping period. */
  nx_uint16_t origin; /* Mote id of sending mote. */
  nx_uint16_t count; /* The readings are samples count * NREADINGS onwards */
  nx_uint16_t voltage_reading;
  nx_uint16_t irradiance_reading;
  nx_uint16_t temp_reading;
  nx_uint16_t etx;
  nx_uint16_t link_route_value;
  nx_am_addr_t link_route_addr;
  nx_uint16_t 	Beacontrs;
  nx_uint16_t	Beaconrcv;
  nx_uint16_t	Datafwd;
  nx_uint16_t	Datarcv;
  nx_uint16_t	Dataov;
  nx_uint16_t	Percent_cap;
  nx_uint16_t	Remain_Life;
} mviz_msg_t;

#endif
