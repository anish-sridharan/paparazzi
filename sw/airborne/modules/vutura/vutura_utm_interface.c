/*
 * Copyright (C) Dennis van Wijngaarden
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/vutura/vutura_utm_interface.c"
 * @author Dennis van Wijngaarden
 * Sends over position and speed data to vutura utm module
 */

#include "modules/vutura/vutura_utm_interface.h"
#include "firmwares/fixedwing/nav.h"
#include "generated/flight_plan.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <string.h>


#define VUTURA_UTM_INTERFACE_VERBOSE 1

#define PRINT(string,...) fprintf(stderr, "[vutura_utm->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if VUTURA_UTM_INTERFACE_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

struct vutura_utm_interface_t {
	int fd;
	struct sockaddr_in ext_addr;
	struct sockaddr_in loc_addr;
} vutura_utm_data;

AvoidanceParameters avoidance;
FlightplanParameters flightplan;

void init_vutura_utm_interface(void)
{
	// Initialise flightplan
	InitFlightplan();

	// initialise constants
	avoidance.avoid = false;
	avoidance.vn = 0;
	avoidance.ve = 0;
	avoidance.vd = 0;

	// Make a UDP connection
	vutura_utm_data.fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	// Listen to Vutura UTM
	memset(&vutura_utm_data.loc_addr, 0, sizeof(vutura_utm_data.loc_addr));
	vutura_utm_data.loc_addr.sin_family = AF_INET;
	vutura_utm_data.loc_addr.sin_addr.s_addr = INADDR_ANY;
	vutura_utm_data.loc_addr.sin_port = htons(8200);

	if (bind(vutura_utm_data.fd, (struct sockaddr *)&vutura_utm_data.loc_addr, sizeof(struct sockaddr)) == -1)
	{
		VERBOSE_PRINT("UDP bind failed");
		close(vutura_utm_data.fd);
		exit(EXIT_FAILURE);
	}

	// Configure socket for sending from UAV to Vutura_utm
	memset(&vutura_utm_data.ext_addr, 0, sizeof(vutura_utm_data.ext_addr));
	vutura_utm_data.ext_addr.sin_family = AF_INET;
	vutura_utm_data.ext_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
	vutura_utm_data.ext_addr.sin_port = htons(14551);

}

void parse_gps(void)
{
	struct LlaCoor_i *lonlatalt = stateGetPositionLla_i();
	struct NedCoor_f *speed_ned = stateGetSpeedNed_f();

	// Construct message
	PaparazziToVuturaMsg msg;

	msg.lon = lonlatalt->lon; // [deg e7]
	msg.lat = lonlatalt->lat; // [deg e7]]
	msg.alt = lonlatalt->alt; // [mm]
	msg.Vn  = speed_ned->x * 1000.; // [mm/s]
	msg.Ve  = speed_ned->y * 1000.; // [mm/s]
	msg.Vd  = speed_ned->z * 1000.; // [mm/s]

	// Put it in the buffer

	// Send over UDP

	ssize_t bytes_sent = sendto(vutura_utm_data.fd, &msg, sizeof(msg), 0, (struct sockaddr*)&vutura_utm_data.ext_addr, sizeof(vutura_utm_data.ext_addr));
	(void) bytes_sent;
	return;
}

void avoid_check(void)
{

	VuturaToPaparazziMsg msg;

	socklen_t addr_len = sizeof(vutura_utm_data.loc_addr);
	ssize_t count = recvfrom(vutura_utm_data.fd, &msg, sizeof(msg), MSG_DONTWAIT, (struct sockaddr*)&vutura_utm_data.loc_addr, &addr_len);
	if (count == -1)
		{
			//VERBOSE_PRINT("No UDP data\n");
		}
		else if (count == sizeof(msg))
		{
			avoidance.avoid = msg.avoid;
			avoidance.vn = msg.vn;
			avoidance.ve = msg.ve;
			avoidance.vd = msg.vd;
			avoidance.lat = msg.lat;
			avoidance.lon = msg.lon;
			//VERBOSE_PRINT("received avoidance msg: avoid->%i, vn->%i, ve->%i, vd->%i, lat->%i, lon->%i\n", avoidance.avoid, avoidance.vn, avoidance.ve, avoidance.vd, avoidance.lat, avoidance.lon);
		}
		else
		{
			//VERBOSE_PRINT("msg not of correct size\n");
		}

	return;
}

bool GetAvoid(void)
{
	return avoidance.avoid;
}

void InitFlightplan(void)
{
	flightplan.start_index_ROUTE = WP_ROUTE_00;
	flightplan.start_index_AVOID = WP_AVOID_00;
	flightplan.lla_ref_i.lat = NAV_LAT0;
	flightplan.lla_ref_i.lon = NAV_LON0;
	flightplan.lla_ref_i.alt = NAV_ALT0;
	ltp_def_from_lla_i(&flightplan.ltp_ref_i, &flightplan.lla_ref_i);
}

void RunAvoidance(void)
{
	// First check if LEG_xx block is activated
	char* fp_blocks[NB_BLOCK] = FP_BLOCKS;
	uint8_t leg_number;

	// Check if LEG and/or AVOID block is activated and move avoidance waypoints
	if (strstr(fp_blocks[nav_block], "LEG_") != NULL)
	{
		char leg_number_c[2];

		if (strstr(fp_blocks[nav_block], "_AVOID_") != NULL)
		{
			// Check LEG number if in AVOID LEG
			leg_number_c[0] = fp_blocks[nav_block][10];
			leg_number_c[1] = fp_blocks[nav_block][11];
			leg_number = atoi(leg_number_c);
		}
		else
		{
			// Check LEG number if in regular LEG
			leg_number_c[0] = fp_blocks[nav_block][4];
			leg_number_c[1] = fp_blocks[nav_block][5];
			leg_number = atoi(leg_number_c);
		}

		// If in LEG or AVOID, move waypoints in case of avoidance
		if (avoidance.avoid)
		{
			uint8_t wp_id_from = flightplan.start_index_AVOID + leg_number;
			uint8_t wp_id_to = wp_id_from + 1;

			NavSetWaypointHere(wp_id_from);

			set_wp_at_latlon(wp_id_to, avoidance.lat, avoidance.lon);



		}
	}
}

void set_wp_at_latlon(uint8_t wp_id, int32_t lat, int32_t lon)
{
	struct LlaCoor_i wp_lla_i;
	struct EnuCoor_i wp_enu_i;
	struct EnuCoor_f wp_enu_f;
	wp_lla_i.lat = lat;
	wp_lla_i.lon = lon;
	wp_lla_i.alt = WaypointAlt(wp_id) / 1000.;

	enu_of_lla_pos_i(&wp_enu_i, &flightplan.ltp_ref_i, &wp_lla_i);

	ENU_FLOAT_OF_BFP(wp_enu_f, wp_enu_i);

	waypoints[wp_id].x = wp_enu_f.x;
	waypoints[wp_id].y = wp_enu_f.y;
}
