/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
/**
 * @file land.cpp
 *
 * Helper class to land at the current position
 *
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>

#include <geo/geo.h>

#include "land.h"
#include "navigator.h"

void set_waypoint(struct mission_item_s *item,
  struct position_setpoint_triplet_s *pos_sp_triplet);

Land::Land(Navigator *navigator, const char *name) :
    MissionBlock(navigator, name),
    _current_pose_setted(false),
    _param_smart_land_en(this, "NAV_EN_SLAND", false),
    _landing_target{},
    _current_landing_target{},
    _landing_state(ACQUIRE_LAND_SPOT)
{
    /* load initial params */
    updateParams();
}

Land::~Land()
{
}

void
Land::on_inactive()
{
}

void
Land::on_activation()
{
    _navigator->get_mission_result()->reached = false;
    _navigator->get_mission_result()->finished = false;
    _navigator->set_mission_result_updated();
    reset_mission_item_reached();
    _smart_landing_en = _param_smart_land_en.get();
    if (_smart_landing_en) {
      /* handle smart landing */
      // TODO: intialisation
      // send request for landing spot
      // set_current_position_item(&_mission_item);
    	// struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
    	// mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	    _current_pose_setted = false;
      _landing_state = ACQUIRE_LAND_SPOT;
      PX4_WARN("Smart Landing enabled");
    } else {
      _landing_state = LANDING_COMPLETED;
      /* set current mission item to Land */
      set_land_item(&_mission_item, true);
      /* convert mission item to current setpoint */
      struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
      pos_sp_triplet->previous.valid = false;
      mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
      pos_sp_triplet->next.valid = false;

      _navigator->set_can_loiter_at_sp(false);

      _navigator->set_position_setpoint_triplet_updated();
  }
}

void
Land::on_active()
{
    /* handle landing */
    // check if new postion msg are sent for smart landing
    bool landing_target_updated = false;
    orb_check(_navigator->get_landing_target_sub(), &landing_target_updated);
    if (landing_target_updated) {
      update_landing_target();
    }
    // handle manouver
    if (is_mission_item_reached() || _landing_state==ACQUIRE_LAND_SPOT) {
      smart_landing_state_machine();
    }
}

void
Land::update_landing_target()
{
  if (orb_copy(ORB_ID(landing_target), _navigator->get_landing_target_sub(), &_landing_target) == OK){
    // select best landing spot
    if (_landing_target.target_num == 0) {
      PX4_WARN("landing_target msg recieved");
      _current_landing_target = _landing_target;
      // get global position
      _current_global_position = _navigator->get_global_position();
      _current_pose_setted = true;
    }
  }
}

void Land::add_landing_target_delta(float &x_curr, float &y_curr,float theta,
                                    float phi, float distance){
  x_curr += distance*sinf(theta)*cosf(phi);
  y_curr += distance*sinf(theta)*sinf(phi);
}

void Land::smart_landing_state_machine() {
  switch (_landing_state) {
    case ACQUIRE_LAND_SPOT:
      if (_current_pose_setted) {
        // get land position
        get_current_landing_target();
        _current_pose_setted = false;
        PX4_INFO("Landing spot acquired");
        set_land_waypoint();
        PX4_INFO("Waypoint setted");
        _landing_state = LANDING_SPOT_CHECK;
      }
      break;
    case LANDING_SPOT_CHECK:
      // check if current position still good to land
      PX4_INFO("Checking if landing spot is still the best...");
      get_current_landing_target();
      _current_pose_setted = false;
      if (_setted_landing_target.angle_x < 0.09f) {
        PX4_INFO("Okay.");
        _landing_state = FINAL_TOUCH_DOWN;
        PX4_INFO("Starting descent...");
      }else{
        PX4_INFO("Landing spot is not the best anymore.");
        _landing_state = ACQUIRE_LAND_SPOT; // recon
        PX4_INFO("Relocating...");
      }
      break;

    case START_DECENT:
      // reduce altitude to nearly 7 m
      break;

    case LANDING_SPOT_DECENT_CHECK:
      // check goodness again of the landing spot
      break;

    case FINAL_TOUCH_DOWN:
      PX4_INFO("Waypoint reached");
      final_touch_down();
      PX4_INFO("Landing...");
      _landing_state = LANDING_COMPLETED;
      break;

      case LANDING_COMPLETED:
      landing_completed();
      PX4_INFO("Landing completed");
      break;
  }

  reset_mission_item_reached();
}

inline void Land::get_current_landing_target() {
  // copy the best landing target
  _setted_landing_target =_current_landing_target;
}

void Land::set_land_waypoint(){
  struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
  float x,y;
  double lat,lon;
  // convert ot local coordinates
  map_projection_global_project(_current_global_position->lat, _current_global_position->lon, &x, &y);
  PX4_INFO("local x: %f and y %f", (double)x, (double)y);
  // add landing spot
  add_landing_target_delta(x, y, _current_landing_target.angle_x, _current_landing_target.angle_y,
                           _current_landing_target.distance);
  PX4_INFO("new x: %f and y: %f", (double)x,(double)y);
  // convert in lat lon
  map_projection_global_reproject(x, y, &lat, &lon);
  PX4_INFO("New lat: %f and lon: %f", lat, lon);
  mission_item_s landing_waypoint;
  // set mission item
  landing_waypoint.lat = lat;
  landing_waypoint.lon = lon;
  landing_waypoint.altitude = _current_global_position->alt;
  landing_waypoint.loiter_radius = _navigator->get_loiter_radius();
	landing_waypoint.loiter_direction = 1;
	landing_waypoint.nav_cmd = NAV_CMD_WAYPOINT;
	landing_waypoint.acceptance_radius = 1;
	landing_waypoint.time_inside = 0.0f;
	landing_waypoint.pitch_min = 0.0f;
	landing_waypoint.autocontinue = false; // ?????? TODO: review
  landing_waypoint.origin = ORIGIN_ONBOARD;
  _mission_item = landing_waypoint;
  pos_sp_triplet->previous.valid = false;
  /* convert mission item to current position setpoint and make it valid */
  mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
  pos_sp_triplet->next.valid = false;

  _navigator->set_position_setpoint_triplet_updated();
}

void Land::final_touch_down() {
  struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
  set_land_item(&_mission_item, true);
  mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
  pos_sp_triplet->next.valid = false;

  _navigator->set_position_setpoint_triplet_updated();
}

void Land::landing_completed() {
  _navigator->get_mission_result()->finished = true;
  _navigator->set_mission_result_updated();
  set_idle_item(&_mission_item);

  struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
  mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
  _navigator->set_position_setpoint_triplet_updated();
}
