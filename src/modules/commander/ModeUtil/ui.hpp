/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <uORB/topics/vehicle_status.h>

#include <stdint.h>

namespace mode_util
{

/**
 * @return Bitmask with all valid modes
 */
static inline uint32_t getValidNavStates()
{
	return (1u << vehicle_status_s::NAVIGATION_STATE_MANUAL) |
	       (1u << vehicle_status_s::NAVIGATION_STATE_ALTCTL) |
	       (1u << vehicle_status_s::NAVIGATION_STATE_POSCTL) |
	       (1u << vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) |
	       (1u << vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER) |
	       (1u << vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) |
	       (1u << vehicle_status_s::NAVIGATION_STATE_ACRO) |
	       (1u << vehicle_status_s::NAVIGATION_STATE_TERMINATION) |
	       (1u << vehicle_status_s::NAVIGATION_STATE_OFFBOARD) |
	       (1u << vehicle_status_s::NAVIGATION_STATE_STAB) |
	       (1u << vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF) |
	       (1u << vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) |
	       (1u << vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET) |
	       (1u << vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND) |
	       (1u << vehicle_status_s::NAVIGATION_STATE_ORBIT) |
	       (1u << vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF);

	static_assert(vehicle_status_s::NAVIGATION_STATE_MAX  == 31, "code requires update");
}

/**
 * @return Bitmask with all selectable modes
 */
static inline uint32_t getSelectableNavStates()
{
	return (1u << vehicle_status_s::NAVIGATION_STATE_ALTCTL) |
	       (1u << vehicle_status_s::NAVIGATION_STATE_POSCTL);

	static_assert(vehicle_status_s::NAVIGATION_STATE_MAX  == 31, "code requires update");
}

const char *const nav_state_names[vehicle_status_s::NAVIGATION_STATE_MAX] = {
	"Manual",
	"Altitude",
	"Position",
	"Mission",
	"Hold",
	"Return",
	"6: unallocated",
	"7: unallocated",
	"8: unallocated",
	"9: unallocated",
	"Acro",
	"11: UNUSED",
	"Descend",
	"Termination",
	"Offboard",
	"Stabilized",
	"16: UNUSED2",
	"Takeoff",
	"Land",
	"Follow Target",
	"Precision Landing",
	"Orbit",
	"VTOL Takeoff",
	"External 1",
	"External 2",
	"External 3",
	"External 4",
	"External 5",
	"External 6",
	"External 7",
	"External 8",
};

} // namespace mode_util