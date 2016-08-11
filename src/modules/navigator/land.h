/***************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file land.h
 *
 * Helper class to land at the current position
 *
 * @author Andreas Antener <andreas@uaventure.com>
 */

#ifndef NAVIGATOR_LAND_H
#define NAVIGATOR_LAND_H

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include <uORB/topics/landing_target.h>

#include "navigator_mode.h"
#include "mission_block.h"

enum smart_landing_states {
  ACQUIRE_LAND_SPOT,
  ACQUIRE_LAND_SPOT_REACHED,
  LANDING_SPOT_CHECK,
  START_DESCENT,
  START_DESCENT_REACHED,
  LANDING_SPOT_DESCENT_CHECK,
  ABORT_MANOUVER,
  ABORT_MANOUVER_REACHED,
  FINAL_TOUCH_DOWN,
  FINAL_TOUCH_DOWN_REACHED,
  LANDING_COMPLETED
};

class Land : public MissionBlock {
public:
  Land(Navigator *navigator, const char *name);

  ~Land();

  virtual void on_inactive();

  virtual void on_activation();

  virtual void on_active();

private:
  int _smart_landing_en;
  bool _current_pose_setted;
  bool _current_global_position_setted;
  control::BlockParamInt _param_smart_land_en;
  landing_target_s _landing_target;
  landing_target_s _current_landing_targets[10];
  landing_target_s *_current_landing_targets_ptr;
  landing_target_s _setted_landing_target;
  smart_landing_states _landing_state;
  struct vehicle_global_position_s *_current_global_position;

  void update_landing_target();
  void add_landing_target_delta(float &x_curr, float &y_curr, float theta,
                                float phi, float distance);
  void get_best_landing_target();
  void get_current_landing_target();
  void smart_landing_state_machine();
  void set_land_waypoint();
  void set_descent_waypoint();
  void set_abort_waypoint();
  void final_touch_down();
  void landing_completed();
};

#endif
