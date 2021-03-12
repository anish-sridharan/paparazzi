/* This file has been generated from /home/denesh/paparazzi/conf/modules/imu_common.xml /home/denesh/paparazzi/conf/modules/gps.xml /home/denesh/paparazzi/conf/modules/stabilization_indi_simple.xml /home/denesh/paparazzi/conf/modules/nav_basic_rotorcraft.xml /home/denesh/paparazzi/conf/modules/guidance_rotorcraft.xml /home/denesh/paparazzi/conf/modules/ahrs_int_cmpl_quat.xml /home/denesh/paparazzi/conf/modules/ins_extended.xml /home/denesh/paparazzi/conf/modules/bebop_cam.xml /home/denesh/paparazzi/conf/modules/video_capture.xml /home/denesh/paparazzi/conf/modules/cv_detect_color_object.xml /home/denesh/paparazzi/conf/modules/orange_avoider.xml /home/denesh/paparazzi/conf/modules/video_rtp_stream.xml /home/denesh/paparazzi/conf/settings/rotorcraft_basic.xml */
/* Version v5.17_devel-99-gdbb1ea851-dirty */
/* Please DO NOT EDIT */

#ifndef SETTINGS_H
#define SETTINGS_H


#include "generated/periodic_telemetry.h"
#include "autopilot.h"
#include "computer_vision/viewvideo.h"
#include "computer_vision/video_capture.h"
#include "boards/bebop/mt9f002.h"
#include "subsystems/ins/vf_extended_float.h"
#include "subsystems/ahrs/ahrs_int_cmpl_quat.h"
#include "guidance/guidance_h.h"
#include "guidance/guidance_v.h"
#include "navigation.h"
#include "stabilization/stabilization_indi_simple.h"
#include "subsystems/gps.h"
#include "subsystems/imu.h"
#include "generated/modules.h"

#define SETTINGS_NAMES { \
 { "telemetry_mode_Main" }, \
 { "video_thread_video_thread_periodic_status" }, \
 { "logger_file_file_logger_periodic_status" }, \
 { "gps_nps_gps_nps_periodic_check_status" }, \
 { "autopilot_mode_auto2" }, \
 { "autopilot.kill_throttle" }, \
 { "autopilot.power_switch" }, \
 { "autopilot.mode" }, \
 { "viewvideo.use_rtp" }, \
 { "oa_color_count_frac" }, \
 { "cod_lum_min1" }, \
 { "cod_lum_max1" }, \
 { "cod_cb_min1" }, \
 { "cod_cb_max1" }, \
 { "cod_cr_min1" }, \
 { "cod_cr_max1" }, \
 { "cod_draw1" }, \
 { "cod_lum_min2" }, \
 { "cod_lum_max2" }, \
 { "cod_cb_min2" }, \
 { "cod_cb_max2" }, \
 { "cod_cr_min2" }, \
 { "cod_cr_max2" }, \
 { "cod_draw2" }, \
 { "video_capture_record_video" }, \
 { "video_capture_take_shot" }, \
 { "mt9f002.set_zoom" }, \
 { "mt9f002.set_offset_x" }, \
 { "mt9f002.set_offset_y" }, \
 { "mt9f002_send_resolution" }, \
 { "mt9f002.gain_green1" }, \
 { "mt9f002.gain_green2" }, \
 { "mt9f002.gain_blue" }, \
 { "mt9f002.gain_red" }, \
 { "mt9f002_send_color" }, \
 { "mt9f002.target_exposure " }, \
 { "mt9f002_send_exposure" }, \
 { "vff.accel_noise" }, \
 { "vff.r_baro" }, \
 { "vff.r_alt" }, \
 { "vff.r_obs_height" }, \
 { "ahrs_icq.gravity_heuristic_factor" }, \
 { "ahrs_icq.accel_omega" }, \
 { "ahrs_icq.accel_zeta" }, \
 { "ahrs_icq.mag_omega" }, \
 { "ahrs_icq.mag_zeta" }, \
 { "guidance_h.use_ref" }, \
 { "gh_ref.max_speed" }, \
 { "guidance_h.approx_force_by_thrust" }, \
 { "gh_ref.tau" }, \
 { "gh_ref.omega" }, \
 { "gh_ref.zeta" }, \
 { "guidance_h.gains.p" }, \
 { "guidance_h.gains.d" }, \
 { "guidance_h.gains.i" }, \
 { "guidance_h.gains.v" }, \
 { "guidance_h.gains.a" }, \
 { "guidance_h.sp.pos.x" }, \
 { "guidance_h.sp.pos.y" }, \
 { "guidance_v_kp" }, \
 { "guidance_v_kd" }, \
 { "guidance_v_ki" }, \
 { "guidance_v_nominal_throttle" }, \
 { "guidance_v_adapt_throttle_enabled" }, \
 { "guidance_v_z_sp" }, \
 { "flight_altitude" }, \
 { "nav_heading" }, \
 { "nav_radius" }, \
 { "nav_climb_vspeed" }, \
 { "nav_descend_vspeed" }, \
 { "indi.gains.att.p" }, \
 { "indi.gains.rate.p" }, \
 { "indi.g1.p" }, \
 { "indi.gains.att.q" }, \
 { "indi.gains.rate.q" }, \
 { "indi.g1.q" }, \
 { "indi.gains.att.r" }, \
 { "indi.gains.rate.r" }, \
 { "indi.g1.r" }, \
 { "indi.g2" }, \
 { "indi.adaptive" }, \
 { "indi.max_rate" }, \
 { "indi.attitude_max_yaw_rate" }, \
 { "multi_gps_mode" }, \
 { "imu.body_to_imu.eulers_f.phi" }, \
 { "imu.body_to_imu.eulers_f.theta" }, \
 { "imu.body_to_imu.eulers_f.psi" }, \
 { "imu.b2i_set_current" }, \
};
#define SETTINGS_NAMES_SHORT { \
 "tel_mod_Mai" , \
 "vid_thr_vid_thr_" , \
 "log_fil_fil_log_" , \
 "gps_nps_gps_nps_" , \
 "aut_mod_aut" , \
 "aut_kil_thr" , \
 "aut_pow_swi" , \
 "aut_mod" , \
 "vie_use_rtp" , \
 "oa_col_cou_fra" , \
 "cod_lum_min" , \
 "cod_lum_max" , \
 "cod_cb_min" , \
 "cod_cb_max" , \
 "cod_cr_min" , \
 "cod_cr_max" , \
 "cod_dra" , \
 "cod_lum_min" , \
 "cod_lum_max" , \
 "cod_cb_min" , \
 "cod_cb_max" , \
 "cod_cr_min" , \
 "cod_cr_max" , \
 "cod_dra" , \
 "vid_cap_rec_vid" , \
 "vid_cap_tak_sho" , \
 "mt9_set_zoo" , \
 "mt9_set_off_x" , \
 "mt9_set_off_y" , \
 "mt9_sen_res" , \
 "mt9_gai_gre" , \
 "mt9_gai_gre" , \
 "mt9_gai_blu" , \
 "mt9_gai_red" , \
 "mt9_sen_col" , \
 "mt9_tar_exp" , \
 "mt9_sen_exp" , \
 "vff_acc_noi" , \
 "vff_r_bar" , \
 "vff_r_alt" , \
 "vff_r_obs_hei" , \
 "ahr_icq_gra_heu_" , \
 "ahr_icq_acc_ome" , \
 "ahr_icq_acc_zet" , \
 "ahr_icq_mag_ome" , \
 "ahr_icq_mag_zet" , \
 "gui_h_use_ref" , \
 "gh_ref_max_spe" , \
 "gui_h_app_for_by" , \
 "gh_ref_tau" , \
 "gh_ref_ome" , \
 "gh_ref_zet" , \
 "gui_h_gai_p" , \
 "gui_h_gai_d" , \
 "gui_h_gai_i" , \
 "gui_h_gai_v" , \
 "gui_h_gai_a" , \
 "gui_h_sp_pos_x" , \
 "gui_h_sp_pos_y" , \
 "gui_v_kp" , \
 "gui_v_kd" , \
 "gui_v_ki" , \
 "gui_v_nom_thr" , \
 "gui_v_ada_thr_en" , \
 "gui_v_z_sp" , \
 "fli_alt" , \
 "nav_hea" , \
 "nav_rad" , \
 "nav_cli_vsp" , \
 "nav_des_vsp" , \
 "ind_gai_att_p" , \
 "ind_gai_rat_p" , \
 "ind_g1_p" , \
 "ind_gai_att_q" , \
 "ind_gai_rat_q" , \
 "ind_g1_q" , \
 "ind_gai_att_r" , \
 "ind_gai_rat_r" , \
 "ind_g1_r" , \
 "ind_g2" , \
 "ind_ada" , \
 "ind_max_rat" , \
 "ind_att_max_yaw_" , \
 "mul_gps_mod" , \
 "imu_bod_to_imu_e" , \
 "imu_bod_to_imu_e" , \
 "imu_bod_to_imu_e" , \
 "imu_b2i_set_cur" , \
};
#define NB_SETTING 88
#define DlSetting(_idx, _value) { \
  switch (_idx) { \
    case 0: telemetry_mode_Main = _value; break;\
    case 1: video_thread_video_thread_periodic_status = _value; break;\
    case 2: logger_file_file_logger_periodic_status = _value; break;\
    case 3: gps_nps_gps_nps_periodic_check_status = _value; break;\
    case 4: autopilot_mode_auto2 = _value; break;\
    case 5: autopilot_KillThrottle( _value ); _value = autopilot.kill_throttle; break;\
    case 6: autopilot_SetPowerSwitch( _value ); _value = autopilot.power_switch; break;\
    case 7: autopilot_SetModeHandler( _value ); _value = autopilot.mode; break;\
    case 8: viewvideo.use_rtp = _value; break;\
    case 9: oa_color_count_frac = _value; break;\
    case 10: cod_lum_min1 = _value; break;\
    case 11: cod_lum_max1 = _value; break;\
    case 12: cod_cb_min1 = _value; break;\
    case 13: cod_cb_max1 = _value; break;\
    case 14: cod_cr_min1 = _value; break;\
    case 15: cod_cr_max1 = _value; break;\
    case 16: cod_draw1 = _value; break;\
    case 17: cod_lum_min2 = _value; break;\
    case 18: cod_lum_max2 = _value; break;\
    case 19: cod_cb_min2 = _value; break;\
    case 20: cod_cb_max2 = _value; break;\
    case 21: cod_cr_min2 = _value; break;\
    case 22: cod_cr_max2 = _value; break;\
    case 23: cod_draw2 = _value; break;\
    case 24: video_capture_record_video = _value; break;\
    case 25: video_capture_take_shot = _value; break;\
    case 26: mt9f002.set_zoom = _value; break;\
    case 27: mt9f002.set_offset_x = _value; break;\
    case 28: mt9f002.set_offset_y = _value; break;\
    case 29: mt9f002_setting_update_resolution( _value ); _value = mt9f002_send_resolution; break;\
    case 30: mt9f002.gain_green1 = _value; break;\
    case 31: mt9f002.gain_green2 = _value; break;\
    case 32: mt9f002.gain_blue = _value; break;\
    case 33: mt9f002.gain_red = _value; break;\
    case 34: mt9f002_setting_update_color( _value ); _value = mt9f002_send_color; break;\
    case 35: mt9f002.target_exposure  = _value; break;\
    case 36: mt9f002_setting_update_exposure( _value ); _value = mt9f002_send_exposure; break;\
    case 37: vff.accel_noise = _value; break;\
    case 38: vff.r_baro = _value; break;\
    case 39: vff.r_alt = _value; break;\
    case 40: vff.r_obs_height = _value; break;\
    case 41: ahrs_icq.gravity_heuristic_factor = _value; break;\
    case 42: ahrs_int_cmpl_quat_SetAccelOmega( _value ); _value = ahrs_icq.accel_omega; break;\
    case 43: ahrs_int_cmpl_quat_SetAccelZeta( _value ); _value = ahrs_icq.accel_zeta; break;\
    case 44: ahrs_int_cmpl_quat_SetMagOmega( _value ); _value = ahrs_icq.mag_omega; break;\
    case 45: ahrs_int_cmpl_quat_SetMagZeta( _value ); _value = ahrs_icq.mag_zeta; break;\
    case 46: guidance_h_SetUseRef( _value ); _value = guidance_h.use_ref; break;\
    case 47: guidance_h_SetMaxSpeed( _value ); _value = gh_ref.max_speed; break;\
    case 48: guidance_h.approx_force_by_thrust = _value; break;\
    case 49: guidance_h_SetTau( _value ); _value = gh_ref.tau; break;\
    case 50: guidance_h_SetOmega( _value ); _value = gh_ref.omega; break;\
    case 51: guidance_h_SetZeta( _value ); _value = gh_ref.zeta; break;\
    case 52: guidance_h.gains.p = _value; break;\
    case 53: guidance_h.gains.d = _value; break;\
    case 54: guidance_h_set_igain( _value ); _value = guidance_h.gains.i; break;\
    case 55: guidance_h.gains.v = _value; break;\
    case 56: guidance_h.gains.a = _value; break;\
    case 57: guidance_h.sp.pos.x = _value; break;\
    case 58: guidance_h.sp.pos.y = _value; break;\
    case 59: guidance_v_kp = _value; break;\
    case 60: guidance_v_kd = _value; break;\
    case 61: guidance_v_SetKi( _value ); _value = guidance_v_ki; break;\
    case 62: guidance_v_nominal_throttle = _value; break;\
    case 63: guidance_v_adapt_throttle_enabled = _value; break;\
    case 64: guidance_v_z_sp = _value; break;\
    case 65: navigation_SetFlightAltitude( _value ); _value = flight_altitude; break;\
    case 66: nav_heading = _value; break;\
    case 67: nav_radius = _value; break;\
    case 68: nav_climb_vspeed = _value; break;\
    case 69: nav_descend_vspeed = _value; break;\
    case 70: indi.gains.att.p = _value; break;\
    case 71: indi.gains.rate.p = _value; break;\
    case 72: indi.g1.p = _value; break;\
    case 73: indi.gains.att.q = _value; break;\
    case 74: indi.gains.rate.q = _value; break;\
    case 75: indi.g1.q = _value; break;\
    case 76: indi.gains.att.r = _value; break;\
    case 77: indi.gains.rate.r = _value; break;\
    case 78: indi.g1.r = _value; break;\
    case 79: indi.g2 = _value; break;\
    case 80: indi.adaptive = _value; break;\
    case 81: indi.max_rate = _value; break;\
    case 82: indi.attitude_max_yaw_rate = _value; break;\
    case 83: multi_gps_mode = _value; break;\
    case 84: imu_SetBodyToImuPhi( _value ); _value = imu.body_to_imu.eulers_f.phi; break;\
    case 85: imu_SetBodyToImuTheta( _value ); _value = imu.body_to_imu.eulers_f.theta; break;\
    case 86: imu_SetBodyToImuPsi( _value ); _value = imu.body_to_imu.eulers_f.psi; break;\
    case 87: imu_SetBodyToImuCurrent( _value ); _value = imu.b2i_set_current; break;\
    default: break;\
  }\
}
#define PeriodicSendDlValue(_trans, _dev) { \
  static uint8_t i;\
  float var;\
  if (i >= 88) i = 0;\
  switch (i) { \
    case 0: var = telemetry_mode_Main; break;\
    case 1: var = video_thread_video_thread_periodic_status; break;\
    case 2: var = logger_file_file_logger_periodic_status; break;\
    case 3: var = gps_nps_gps_nps_periodic_check_status; break;\
    case 4: var = autopilot_mode_auto2; break;\
    case 5: var = autopilot.kill_throttle; break;\
    case 6: var = autopilot.power_switch; break;\
    case 7: var = autopilot.mode; break;\
    case 8: var = viewvideo.use_rtp; break;\
    case 9: var = oa_color_count_frac; break;\
    case 10: var = cod_lum_min1; break;\
    case 11: var = cod_lum_max1; break;\
    case 12: var = cod_cb_min1; break;\
    case 13: var = cod_cb_max1; break;\
    case 14: var = cod_cr_min1; break;\
    case 15: var = cod_cr_max1; break;\
    case 16: var = cod_draw1; break;\
    case 17: var = cod_lum_min2; break;\
    case 18: var = cod_lum_max2; break;\
    case 19: var = cod_cb_min2; break;\
    case 20: var = cod_cb_max2; break;\
    case 21: var = cod_cr_min2; break;\
    case 22: var = cod_cr_max2; break;\
    case 23: var = cod_draw2; break;\
    case 24: var = video_capture_record_video; break;\
    case 25: var = video_capture_take_shot; break;\
    case 26: var = mt9f002.set_zoom; break;\
    case 27: var = mt9f002.set_offset_x; break;\
    case 28: var = mt9f002.set_offset_y; break;\
    case 29: var = mt9f002_send_resolution; break;\
    case 30: var = mt9f002.gain_green1; break;\
    case 31: var = mt9f002.gain_green2; break;\
    case 32: var = mt9f002.gain_blue; break;\
    case 33: var = mt9f002.gain_red; break;\
    case 34: var = mt9f002_send_color; break;\
    case 35: var = mt9f002.target_exposure ; break;\
    case 36: var = mt9f002_send_exposure; break;\
    case 37: var = vff.accel_noise; break;\
    case 38: var = vff.r_baro; break;\
    case 39: var = vff.r_alt; break;\
    case 40: var = vff.r_obs_height; break;\
    case 41: var = ahrs_icq.gravity_heuristic_factor; break;\
    case 42: var = ahrs_icq.accel_omega; break;\
    case 43: var = ahrs_icq.accel_zeta; break;\
    case 44: var = ahrs_icq.mag_omega; break;\
    case 45: var = ahrs_icq.mag_zeta; break;\
    case 46: var = guidance_h.use_ref; break;\
    case 47: var = gh_ref.max_speed; break;\
    case 48: var = guidance_h.approx_force_by_thrust; break;\
    case 49: var = gh_ref.tau; break;\
    case 50: var = gh_ref.omega; break;\
    case 51: var = gh_ref.zeta; break;\
    case 52: var = guidance_h.gains.p; break;\
    case 53: var = guidance_h.gains.d; break;\
    case 54: var = guidance_h.gains.i; break;\
    case 55: var = guidance_h.gains.v; break;\
    case 56: var = guidance_h.gains.a; break;\
    case 57: var = guidance_h.sp.pos.x; break;\
    case 58: var = guidance_h.sp.pos.y; break;\
    case 59: var = guidance_v_kp; break;\
    case 60: var = guidance_v_kd; break;\
    case 61: var = guidance_v_ki; break;\
    case 62: var = guidance_v_nominal_throttle; break;\
    case 63: var = guidance_v_adapt_throttle_enabled; break;\
    case 64: var = guidance_v_z_sp; break;\
    case 65: var = flight_altitude; break;\
    case 66: var = nav_heading; break;\
    case 67: var = nav_radius; break;\
    case 68: var = nav_climb_vspeed; break;\
    case 69: var = nav_descend_vspeed; break;\
    case 70: var = indi.gains.att.p; break;\
    case 71: var = indi.gains.rate.p; break;\
    case 72: var = indi.g1.p; break;\
    case 73: var = indi.gains.att.q; break;\
    case 74: var = indi.gains.rate.q; break;\
    case 75: var = indi.g1.q; break;\
    case 76: var = indi.gains.att.r; break;\
    case 77: var = indi.gains.rate.r; break;\
    case 78: var = indi.g1.r; break;\
    case 79: var = indi.g2; break;\
    case 80: var = indi.adaptive; break;\
    case 81: var = indi.max_rate; break;\
    case 82: var = indi.attitude_max_yaw_rate; break;\
    case 83: var = multi_gps_mode; break;\
    case 84: var = imu.body_to_imu.eulers_f.phi; break;\
    case 85: var = imu.body_to_imu.eulers_f.theta; break;\
    case 86: var = imu.body_to_imu.eulers_f.psi; break;\
    case 87: var = imu.b2i_set_current; break;\
    default: var = 0.; break;\
  }\
  pprz_msg_send_DL_VALUE(_trans, _dev, AC_ID, &i, &var);\
  i++;\
}
static inline float settings_get_value(uint8_t i) {
  switch (i) {
    case 0: return telemetry_mode_Main;
    case 1: return video_thread_video_thread_periodic_status;
    case 2: return logger_file_file_logger_periodic_status;
    case 3: return gps_nps_gps_nps_periodic_check_status;
    case 4: return autopilot_mode_auto2;
    case 5: return autopilot.kill_throttle;
    case 6: return autopilot.power_switch;
    case 7: return autopilot.mode;
    case 8: return viewvideo.use_rtp;
    case 9: return oa_color_count_frac;
    case 10: return cod_lum_min1;
    case 11: return cod_lum_max1;
    case 12: return cod_cb_min1;
    case 13: return cod_cb_max1;
    case 14: return cod_cr_min1;
    case 15: return cod_cr_max1;
    case 16: return cod_draw1;
    case 17: return cod_lum_min2;
    case 18: return cod_lum_max2;
    case 19: return cod_cb_min2;
    case 20: return cod_cb_max2;
    case 21: return cod_cr_min2;
    case 22: return cod_cr_max2;
    case 23: return cod_draw2;
    case 24: return video_capture_record_video;
    case 25: return video_capture_take_shot;
    case 26: return mt9f002.set_zoom;
    case 27: return mt9f002.set_offset_x;
    case 28: return mt9f002.set_offset_y;
    case 29: return mt9f002_send_resolution;
    case 30: return mt9f002.gain_green1;
    case 31: return mt9f002.gain_green2;
    case 32: return mt9f002.gain_blue;
    case 33: return mt9f002.gain_red;
    case 34: return mt9f002_send_color;
    case 35: return mt9f002.target_exposure ;
    case 36: return mt9f002_send_exposure;
    case 37: return vff.accel_noise;
    case 38: return vff.r_baro;
    case 39: return vff.r_alt;
    case 40: return vff.r_obs_height;
    case 41: return ahrs_icq.gravity_heuristic_factor;
    case 42: return ahrs_icq.accel_omega;
    case 43: return ahrs_icq.accel_zeta;
    case 44: return ahrs_icq.mag_omega;
    case 45: return ahrs_icq.mag_zeta;
    case 46: return guidance_h.use_ref;
    case 47: return gh_ref.max_speed;
    case 48: return guidance_h.approx_force_by_thrust;
    case 49: return gh_ref.tau;
    case 50: return gh_ref.omega;
    case 51: return gh_ref.zeta;
    case 52: return guidance_h.gains.p;
    case 53: return guidance_h.gains.d;
    case 54: return guidance_h.gains.i;
    case 55: return guidance_h.gains.v;
    case 56: return guidance_h.gains.a;
    case 57: return guidance_h.sp.pos.x;
    case 58: return guidance_h.sp.pos.y;
    case 59: return guidance_v_kp;
    case 60: return guidance_v_kd;
    case 61: return guidance_v_ki;
    case 62: return guidance_v_nominal_throttle;
    case 63: return guidance_v_adapt_throttle_enabled;
    case 64: return guidance_v_z_sp;
    case 65: return flight_altitude;
    case 66: return nav_heading;
    case 67: return nav_radius;
    case 68: return nav_climb_vspeed;
    case 69: return nav_descend_vspeed;
    case 70: return indi.gains.att.p;
    case 71: return indi.gains.rate.p;
    case 72: return indi.g1.p;
    case 73: return indi.gains.att.q;
    case 74: return indi.gains.rate.q;
    case 75: return indi.g1.q;
    case 76: return indi.gains.att.r;
    case 77: return indi.gains.rate.r;
    case 78: return indi.g1.r;
    case 79: return indi.g2;
    case 80: return indi.adaptive;
    case 81: return indi.max_rate;
    case 82: return indi.attitude_max_yaw_rate;
    case 83: return multi_gps_mode;
    case 84: return imu.body_to_imu.eulers_f.phi;
    case 85: return imu.body_to_imu.eulers_f.theta;
    case 86: return imu.body_to_imu.eulers_f.psi;
    case 87: return imu.b2i_set_current;
    default: return 0.;
  }
}

/* Persistent Settings */
struct PersistentSettings {
  uint8_t s_0; /* ahrs_icq.gravity_heuristic_factor */
  float s_1; /* ahrs_icq.accel_omega */
  float s_2; /* ahrs_icq.accel_zeta */
  float s_3; /* ahrs_icq.mag_omega */
  float s_4; /* ahrs_icq.mag_zeta */
  float s_5; /* guidance_h.use_ref */
  float s_6; /* gh_ref.max_speed */
  uint8_t s_7; /* guidance_h.approx_force_by_thrust */
  float s_8; /* gh_ref.tau */
  float s_9; /* gh_ref.omega */
  float s_10; /* gh_ref.zeta */
  int32_t s_11; /* guidance_h.gains.p */
  int32_t s_12; /* guidance_h.gains.d */
  int32_t s_13; /* guidance_h.gains.i */
  int32_t s_14; /* guidance_h.gains.v */
  int32_t s_15; /* guidance_h.gains.a */
  float s_16; /* guidance_v_kp */
  float s_17; /* guidance_v_kd */
  float s_18; /* guidance_v_ki */
  float s_19; /* guidance_v_nominal_throttle */
  float s_20; /* guidance_v_adapt_throttle_enabled */
  float s_21; /* indi.gains.att.p */
  float s_22; /* indi.gains.rate.p */
  float s_23; /* indi.g1.p */
  float s_24; /* indi.gains.att.q */
  float s_25; /* indi.gains.rate.q */
  float s_26; /* indi.g1.q */
  float s_27; /* indi.gains.att.r */
  float s_28; /* indi.gains.rate.r */
  float s_29; /* indi.g1.r */
  float s_30; /* indi.g2 */
  uint8_t s_31; /* indi.adaptive */
  float s_32; /* imu.body_to_imu.eulers_f.phi */
  float s_33; /* imu.body_to_imu.eulers_f.theta */
  float s_34; /* imu.body_to_imu.eulers_f.psi */
};

extern struct PersistentSettings pers_settings;

static inline void persistent_settings_store( void ) {
  pers_settings.s_0 = ahrs_icq.gravity_heuristic_factor;
  pers_settings.s_1 = ahrs_icq.accel_omega;
  pers_settings.s_2 = ahrs_icq.accel_zeta;
  pers_settings.s_3 = ahrs_icq.mag_omega;
  pers_settings.s_4 = ahrs_icq.mag_zeta;
  pers_settings.s_5 = guidance_h.use_ref;
  pers_settings.s_6 = gh_ref.max_speed;
  pers_settings.s_7 = guidance_h.approx_force_by_thrust;
  pers_settings.s_8 = gh_ref.tau;
  pers_settings.s_9 = gh_ref.omega;
  pers_settings.s_10 = gh_ref.zeta;
  pers_settings.s_11 = guidance_h.gains.p;
  pers_settings.s_12 = guidance_h.gains.d;
  pers_settings.s_13 = guidance_h.gains.i;
  pers_settings.s_14 = guidance_h.gains.v;
  pers_settings.s_15 = guidance_h.gains.a;
  pers_settings.s_16 = guidance_v_kp;
  pers_settings.s_17 = guidance_v_kd;
  pers_settings.s_18 = guidance_v_ki;
  pers_settings.s_19 = guidance_v_nominal_throttle;
  pers_settings.s_20 = guidance_v_adapt_throttle_enabled;
  pers_settings.s_21 = indi.gains.att.p;
  pers_settings.s_22 = indi.gains.rate.p;
  pers_settings.s_23 = indi.g1.p;
  pers_settings.s_24 = indi.gains.att.q;
  pers_settings.s_25 = indi.gains.rate.q;
  pers_settings.s_26 = indi.g1.q;
  pers_settings.s_27 = indi.gains.att.r;
  pers_settings.s_28 = indi.gains.rate.r;
  pers_settings.s_29 = indi.g1.r;
  pers_settings.s_30 = indi.g2;
  pers_settings.s_31 = indi.adaptive;
  pers_settings.s_32 = imu.body_to_imu.eulers_f.phi;
  pers_settings.s_33 = imu.body_to_imu.eulers_f.theta;
  pers_settings.s_34 = imu.body_to_imu.eulers_f.psi;
}

static inline void persistent_settings_load( void ) {
  ahrs_icq.gravity_heuristic_factor = pers_settings.s_0;
  ahrs_int_cmpl_quat_SetAccelOmega( pers_settings.s_1 );
  ahrs_int_cmpl_quat_SetAccelZeta( pers_settings.s_2 );
  ahrs_int_cmpl_quat_SetMagOmega( pers_settings.s_3 );
  ahrs_int_cmpl_quat_SetMagZeta( pers_settings.s_4 );
  guidance_h_SetUseRef( pers_settings.s_5 );
  guidance_h_SetMaxSpeed( pers_settings.s_6 );
  guidance_h.approx_force_by_thrust = pers_settings.s_7;
  guidance_h_SetTau( pers_settings.s_8 );
  guidance_h_SetOmega( pers_settings.s_9 );
  guidance_h_SetZeta( pers_settings.s_10 );
  guidance_h.gains.p = pers_settings.s_11;
  guidance_h.gains.d = pers_settings.s_12;
  guidance_h_set_igain( pers_settings.s_13 );
  guidance_h.gains.v = pers_settings.s_14;
  guidance_h.gains.a = pers_settings.s_15;
  guidance_v_kp = pers_settings.s_16;
  guidance_v_kd = pers_settings.s_17;
  guidance_v_SetKi( pers_settings.s_18 );
  guidance_v_nominal_throttle = pers_settings.s_19;
  guidance_v_adapt_throttle_enabled = pers_settings.s_20;
  indi.gains.att.p = pers_settings.s_21;
  indi.gains.rate.p = pers_settings.s_22;
  indi.g1.p = pers_settings.s_23;
  indi.gains.att.q = pers_settings.s_24;
  indi.gains.rate.q = pers_settings.s_25;
  indi.g1.q = pers_settings.s_26;
  indi.gains.att.r = pers_settings.s_27;
  indi.gains.rate.r = pers_settings.s_28;
  indi.g1.r = pers_settings.s_29;
  indi.g2 = pers_settings.s_30;
  indi.adaptive = pers_settings.s_31;
  imu_SetBodyToImuPhi( pers_settings.s_32 );
  imu_SetBodyToImuTheta( pers_settings.s_33 );
  imu_SetBodyToImuPsi( pers_settings.s_34 );
}

#endif // SETTINGS_H
