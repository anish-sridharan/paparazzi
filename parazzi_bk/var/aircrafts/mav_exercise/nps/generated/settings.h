/* This file has been generated from /home/denesh/paparazzi/conf/modules/imu_common.xml /home/denesh/paparazzi/conf/modules/gps.xml /home/denesh/paparazzi/conf/modules/stabilization_indi_simple.xml /home/denesh/paparazzi/conf/modules/nav_basic_rotorcraft.xml /home/denesh/paparazzi/conf/modules/guidance_rotorcraft.xml /home/denesh/paparazzi/conf/modules/ahrs_int_cmpl_quat.xml /home/denesh/paparazzi/conf/modules/ins_extended.xml /home/denesh/paparazzi/conf/modules/mav_course_exercise.xml /home/denesh/paparazzi/conf/modules/bebop_cam.xml /home/denesh/paparazzi/conf/modules/video_capture.xml /home/denesh/paparazzi/conf/modules/cv_detect_color_object.xml /home/denesh/paparazzi/conf/modules/cv_opticflow.xml /home/denesh/paparazzi/conf/modules/video_rtp_stream.xml /home/denesh/paparazzi/conf/settings/rotorcraft_basic.xml */
/* Version v5.17_devel-99-gdbb1ea851-dirty */
/* Please DO NOT EDIT */

#ifndef SETTINGS_H
#define SETTINGS_H


#include "generated/periodic_telemetry.h"
#include "autopilot.h"
#include "computer_vision/viewvideo.h"
#include "computer_vision/opticflow_module.h"
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
 { "pose_history_pose_periodic_status" }, \
 { "video_thread_video_thread_periodic_status" }, \
 { "cv_opticflow_opticflow_module_run_status" }, \
 { "logger_file_file_logger_periodic_status" }, \
 { "gps_nps_gps_nps_periodic_check_status" }, \
 { "autopilot_mode_auto2" }, \
 { "autopilot.kill_throttle" }, \
 { "autopilot.power_switch" }, \
 { "autopilot.mode" }, \
 { "viewvideo.use_rtp" }, \
 { "opticflow.method" }, \
 { "opticflow.corner_method" }, \
 { "opticflow.window_size" }, \
 { "opticflow.search_distance" }, \
 { "opticflow.subpixel_factor" }, \
 { "opticflow.resolution_factor" }, \
 { "opticflow.derotation" }, \
 { "opticflow.median_filter" }, \
 { "opticflow.feature_management" }, \
 { "opticflow.track_back" }, \
 { "opticflow.show_flow" }, \
 { "opticflow.max_track_corners" }, \
 { "opticflow.max_iterations" }, \
 { "opticflow.threshold_vec" }, \
 { "opticflow.fast9_adaptive" }, \
 { "opticflow.fast9_threshold" }, \
 { "opticflow.fast9_min_distance" }, \
 { "opticflow.fast9_padding" }, \
 { "opticflow.fast9_region_detect" }, \
 { "opticflow.fast9_num_regions" }, \
 { "opticflow.actfast_long_step" }, \
 { "opticflow.actfast_short_step" }, \
 { "opticflow.actfast_min_gradient" }, \
 { "opticflow.actfast_gradient_method" }, \
 { "opticflow.pyramid_level" }, \
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
 { "div_thresh" }, \
 { "heading_increment" }, \
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
 "pos_his_pos_per_" , \
 "vid_thr_vid_thr_" , \
 "cv_opt_opt_mod_r" , \
 "log_fil_fil_log_" , \
 "gps_nps_gps_nps_" , \
 "aut_mod_aut" , \
 "aut_kil_thr" , \
 "aut_pow_swi" , \
 "aut_mod" , \
 "vie_use_rtp" , \
 "opt_met" , \
 "opt_cor_met" , \
 "opt_win_siz" , \
 "opt_sea_dis" , \
 "opt_sub_fac" , \
 "opt_res_fac" , \
 "opt_der" , \
 "opt_med_fil" , \
 "opt_fea_man" , \
 "opt_tra_bac" , \
 "opt_sho_flo" , \
 "opt_max_tra_cor" , \
 "opt_max_ite" , \
 "opt_thr_vec" , \
 "opt_fas_ada" , \
 "opt_fas_thr" , \
 "opt_fas_min_dis" , \
 "opt_fas_pad" , \
 "opt_fas_reg_det" , \
 "opt_fas_num_reg" , \
 "opt_act_lon_ste" , \
 "opt_act_sho_ste" , \
 "opt_act_min_gra" , \
 "opt_act_gra_met" , \
 "opt_pyr_lev" , \
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
 "div_thr" , \
 "hea_inc" , \
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
#define NB_SETTING 116
#define DlSetting(_idx, _value) { \
  switch (_idx) { \
    case 0: telemetry_mode_Main = _value; break;\
    case 1: pose_history_pose_periodic_status = _value; break;\
    case 2: video_thread_video_thread_periodic_status = _value; break;\
    case 3: cv_opticflow_opticflow_module_run_status = _value; break;\
    case 4: logger_file_file_logger_periodic_status = _value; break;\
    case 5: gps_nps_gps_nps_periodic_check_status = _value; break;\
    case 6: autopilot_mode_auto2 = _value; break;\
    case 7: autopilot_KillThrottle( _value ); _value = autopilot.kill_throttle; break;\
    case 8: autopilot_SetPowerSwitch( _value ); _value = autopilot.power_switch; break;\
    case 9: autopilot_SetModeHandler( _value ); _value = autopilot.mode; break;\
    case 10: viewvideo.use_rtp = _value; break;\
    case 11: opticflow.method = _value; break;\
    case 12: opticflow.corner_method = _value; break;\
    case 13: opticflow.window_size = _value; break;\
    case 14: opticflow.search_distance = _value; break;\
    case 15: opticflow.subpixel_factor = _value; break;\
    case 16: opticflow.resolution_factor = _value; break;\
    case 17: opticflow.derotation = _value; break;\
    case 18: opticflow.median_filter = _value; break;\
    case 19: opticflow.feature_management = _value; break;\
    case 20: opticflow.track_back = _value; break;\
    case 21: opticflow.show_flow = _value; break;\
    case 22: opticflow.max_track_corners = _value; break;\
    case 23: opticflow.max_iterations = _value; break;\
    case 24: opticflow.threshold_vec = _value; break;\
    case 25: opticflow.fast9_adaptive = _value; break;\
    case 26: opticflow.fast9_threshold = _value; break;\
    case 27: opticflow.fast9_min_distance = _value; break;\
    case 28: opticflow.fast9_padding = _value; break;\
    case 29: opticflow.fast9_region_detect = _value; break;\
    case 30: opticflow.fast9_num_regions = _value; break;\
    case 31: opticflow.actfast_long_step = _value; break;\
    case 32: opticflow.actfast_short_step = _value; break;\
    case 33: opticflow.actfast_min_gradient = _value; break;\
    case 34: opticflow.actfast_gradient_method = _value; break;\
    case 35: opticflow.pyramid_level = _value; break;\
    case 36: cod_lum_min1 = _value; break;\
    case 37: cod_lum_max1 = _value; break;\
    case 38: cod_cb_min1 = _value; break;\
    case 39: cod_cb_max1 = _value; break;\
    case 40: cod_cr_min1 = _value; break;\
    case 41: cod_cr_max1 = _value; break;\
    case 42: cod_draw1 = _value; break;\
    case 43: cod_lum_min2 = _value; break;\
    case 44: cod_lum_max2 = _value; break;\
    case 45: cod_cb_min2 = _value; break;\
    case 46: cod_cb_max2 = _value; break;\
    case 47: cod_cr_min2 = _value; break;\
    case 48: cod_cr_max2 = _value; break;\
    case 49: cod_draw2 = _value; break;\
    case 50: video_capture_record_video = _value; break;\
    case 51: video_capture_take_shot = _value; break;\
    case 52: mt9f002.set_zoom = _value; break;\
    case 53: mt9f002.set_offset_x = _value; break;\
    case 54: mt9f002.set_offset_y = _value; break;\
    case 55: mt9f002_setting_update_resolution( _value ); _value = mt9f002_send_resolution; break;\
    case 56: mt9f002.gain_green1 = _value; break;\
    case 57: mt9f002.gain_green2 = _value; break;\
    case 58: mt9f002.gain_blue = _value; break;\
    case 59: mt9f002.gain_red = _value; break;\
    case 60: mt9f002_setting_update_color( _value ); _value = mt9f002_send_color; break;\
    case 61: mt9f002.target_exposure  = _value; break;\
    case 62: mt9f002_setting_update_exposure( _value ); _value = mt9f002_send_exposure; break;\
    case 63: div_thresh = _value; break;\
    case 64: heading_increment = _value; break;\
    case 65: vff.accel_noise = _value; break;\
    case 66: vff.r_baro = _value; break;\
    case 67: vff.r_alt = _value; break;\
    case 68: vff.r_obs_height = _value; break;\
    case 69: ahrs_icq.gravity_heuristic_factor = _value; break;\
    case 70: ahrs_int_cmpl_quat_SetAccelOmega( _value ); _value = ahrs_icq.accel_omega; break;\
    case 71: ahrs_int_cmpl_quat_SetAccelZeta( _value ); _value = ahrs_icq.accel_zeta; break;\
    case 72: ahrs_int_cmpl_quat_SetMagOmega( _value ); _value = ahrs_icq.mag_omega; break;\
    case 73: ahrs_int_cmpl_quat_SetMagZeta( _value ); _value = ahrs_icq.mag_zeta; break;\
    case 74: guidance_h_SetUseRef( _value ); _value = guidance_h.use_ref; break;\
    case 75: guidance_h_SetMaxSpeed( _value ); _value = gh_ref.max_speed; break;\
    case 76: guidance_h.approx_force_by_thrust = _value; break;\
    case 77: guidance_h_SetTau( _value ); _value = gh_ref.tau; break;\
    case 78: guidance_h_SetOmega( _value ); _value = gh_ref.omega; break;\
    case 79: guidance_h_SetZeta( _value ); _value = gh_ref.zeta; break;\
    case 80: guidance_h.gains.p = _value; break;\
    case 81: guidance_h.gains.d = _value; break;\
    case 82: guidance_h_set_igain( _value ); _value = guidance_h.gains.i; break;\
    case 83: guidance_h.gains.v = _value; break;\
    case 84: guidance_h.gains.a = _value; break;\
    case 85: guidance_h.sp.pos.x = _value; break;\
    case 86: guidance_h.sp.pos.y = _value; break;\
    case 87: guidance_v_kp = _value; break;\
    case 88: guidance_v_kd = _value; break;\
    case 89: guidance_v_SetKi( _value ); _value = guidance_v_ki; break;\
    case 90: guidance_v_nominal_throttle = _value; break;\
    case 91: guidance_v_adapt_throttle_enabled = _value; break;\
    case 92: guidance_v_z_sp = _value; break;\
    case 93: navigation_SetFlightAltitude( _value ); _value = flight_altitude; break;\
    case 94: nav_heading = _value; break;\
    case 95: nav_radius = _value; break;\
    case 96: nav_climb_vspeed = _value; break;\
    case 97: nav_descend_vspeed = _value; break;\
    case 98: indi.gains.att.p = _value; break;\
    case 99: indi.gains.rate.p = _value; break;\
    case 100: indi.g1.p = _value; break;\
    case 101: indi.gains.att.q = _value; break;\
    case 102: indi.gains.rate.q = _value; break;\
    case 103: indi.g1.q = _value; break;\
    case 104: indi.gains.att.r = _value; break;\
    case 105: indi.gains.rate.r = _value; break;\
    case 106: indi.g1.r = _value; break;\
    case 107: indi.g2 = _value; break;\
    case 108: indi.adaptive = _value; break;\
    case 109: indi.max_rate = _value; break;\
    case 110: indi.attitude_max_yaw_rate = _value; break;\
    case 111: multi_gps_mode = _value; break;\
    case 112: imu_SetBodyToImuPhi( _value ); _value = imu.body_to_imu.eulers_f.phi; break;\
    case 113: imu_SetBodyToImuTheta( _value ); _value = imu.body_to_imu.eulers_f.theta; break;\
    case 114: imu_SetBodyToImuPsi( _value ); _value = imu.body_to_imu.eulers_f.psi; break;\
    case 115: imu_SetBodyToImuCurrent( _value ); _value = imu.b2i_set_current; break;\
    default: break;\
  }\
}
#define PeriodicSendDlValue(_trans, _dev) { \
  static uint8_t i;\
  float var;\
  if (i >= 116) i = 0;\
  switch (i) { \
    case 0: var = telemetry_mode_Main; break;\
    case 1: var = pose_history_pose_periodic_status; break;\
    case 2: var = video_thread_video_thread_periodic_status; break;\
    case 3: var = cv_opticflow_opticflow_module_run_status; break;\
    case 4: var = logger_file_file_logger_periodic_status; break;\
    case 5: var = gps_nps_gps_nps_periodic_check_status; break;\
    case 6: var = autopilot_mode_auto2; break;\
    case 7: var = autopilot.kill_throttle; break;\
    case 8: var = autopilot.power_switch; break;\
    case 9: var = autopilot.mode; break;\
    case 10: var = viewvideo.use_rtp; break;\
    case 11: var = opticflow.method; break;\
    case 12: var = opticflow.corner_method; break;\
    case 13: var = opticflow.window_size; break;\
    case 14: var = opticflow.search_distance; break;\
    case 15: var = opticflow.subpixel_factor; break;\
    case 16: var = opticflow.resolution_factor; break;\
    case 17: var = opticflow.derotation; break;\
    case 18: var = opticflow.median_filter; break;\
    case 19: var = opticflow.feature_management; break;\
    case 20: var = opticflow.track_back; break;\
    case 21: var = opticflow.show_flow; break;\
    case 22: var = opticflow.max_track_corners; break;\
    case 23: var = opticflow.max_iterations; break;\
    case 24: var = opticflow.threshold_vec; break;\
    case 25: var = opticflow.fast9_adaptive; break;\
    case 26: var = opticflow.fast9_threshold; break;\
    case 27: var = opticflow.fast9_min_distance; break;\
    case 28: var = opticflow.fast9_padding; break;\
    case 29: var = opticflow.fast9_region_detect; break;\
    case 30: var = opticflow.fast9_num_regions; break;\
    case 31: var = opticflow.actfast_long_step; break;\
    case 32: var = opticflow.actfast_short_step; break;\
    case 33: var = opticflow.actfast_min_gradient; break;\
    case 34: var = opticflow.actfast_gradient_method; break;\
    case 35: var = opticflow.pyramid_level; break;\
    case 36: var = cod_lum_min1; break;\
    case 37: var = cod_lum_max1; break;\
    case 38: var = cod_cb_min1; break;\
    case 39: var = cod_cb_max1; break;\
    case 40: var = cod_cr_min1; break;\
    case 41: var = cod_cr_max1; break;\
    case 42: var = cod_draw1; break;\
    case 43: var = cod_lum_min2; break;\
    case 44: var = cod_lum_max2; break;\
    case 45: var = cod_cb_min2; break;\
    case 46: var = cod_cb_max2; break;\
    case 47: var = cod_cr_min2; break;\
    case 48: var = cod_cr_max2; break;\
    case 49: var = cod_draw2; break;\
    case 50: var = video_capture_record_video; break;\
    case 51: var = video_capture_take_shot; break;\
    case 52: var = mt9f002.set_zoom; break;\
    case 53: var = mt9f002.set_offset_x; break;\
    case 54: var = mt9f002.set_offset_y; break;\
    case 55: var = mt9f002_send_resolution; break;\
    case 56: var = mt9f002.gain_green1; break;\
    case 57: var = mt9f002.gain_green2; break;\
    case 58: var = mt9f002.gain_blue; break;\
    case 59: var = mt9f002.gain_red; break;\
    case 60: var = mt9f002_send_color; break;\
    case 61: var = mt9f002.target_exposure ; break;\
    case 62: var = mt9f002_send_exposure; break;\
    case 63: var = div_thresh; break;\
    case 64: var = heading_increment; break;\
    case 65: var = vff.accel_noise; break;\
    case 66: var = vff.r_baro; break;\
    case 67: var = vff.r_alt; break;\
    case 68: var = vff.r_obs_height; break;\
    case 69: var = ahrs_icq.gravity_heuristic_factor; break;\
    case 70: var = ahrs_icq.accel_omega; break;\
    case 71: var = ahrs_icq.accel_zeta; break;\
    case 72: var = ahrs_icq.mag_omega; break;\
    case 73: var = ahrs_icq.mag_zeta; break;\
    case 74: var = guidance_h.use_ref; break;\
    case 75: var = gh_ref.max_speed; break;\
    case 76: var = guidance_h.approx_force_by_thrust; break;\
    case 77: var = gh_ref.tau; break;\
    case 78: var = gh_ref.omega; break;\
    case 79: var = gh_ref.zeta; break;\
    case 80: var = guidance_h.gains.p; break;\
    case 81: var = guidance_h.gains.d; break;\
    case 82: var = guidance_h.gains.i; break;\
    case 83: var = guidance_h.gains.v; break;\
    case 84: var = guidance_h.gains.a; break;\
    case 85: var = guidance_h.sp.pos.x; break;\
    case 86: var = guidance_h.sp.pos.y; break;\
    case 87: var = guidance_v_kp; break;\
    case 88: var = guidance_v_kd; break;\
    case 89: var = guidance_v_ki; break;\
    case 90: var = guidance_v_nominal_throttle; break;\
    case 91: var = guidance_v_adapt_throttle_enabled; break;\
    case 92: var = guidance_v_z_sp; break;\
    case 93: var = flight_altitude; break;\
    case 94: var = nav_heading; break;\
    case 95: var = nav_radius; break;\
    case 96: var = nav_climb_vspeed; break;\
    case 97: var = nav_descend_vspeed; break;\
    case 98: var = indi.gains.att.p; break;\
    case 99: var = indi.gains.rate.p; break;\
    case 100: var = indi.g1.p; break;\
    case 101: var = indi.gains.att.q; break;\
    case 102: var = indi.gains.rate.q; break;\
    case 103: var = indi.g1.q; break;\
    case 104: var = indi.gains.att.r; break;\
    case 105: var = indi.gains.rate.r; break;\
    case 106: var = indi.g1.r; break;\
    case 107: var = indi.g2; break;\
    case 108: var = indi.adaptive; break;\
    case 109: var = indi.max_rate; break;\
    case 110: var = indi.attitude_max_yaw_rate; break;\
    case 111: var = multi_gps_mode; break;\
    case 112: var = imu.body_to_imu.eulers_f.phi; break;\
    case 113: var = imu.body_to_imu.eulers_f.theta; break;\
    case 114: var = imu.body_to_imu.eulers_f.psi; break;\
    case 115: var = imu.b2i_set_current; break;\
    default: var = 0.; break;\
  }\
  pprz_msg_send_DL_VALUE(_trans, _dev, AC_ID, &i, &var);\
  i++;\
}
static inline float settings_get_value(uint8_t i) {
  switch (i) {
    case 0: return telemetry_mode_Main;
    case 1: return pose_history_pose_periodic_status;
    case 2: return video_thread_video_thread_periodic_status;
    case 3: return cv_opticflow_opticflow_module_run_status;
    case 4: return logger_file_file_logger_periodic_status;
    case 5: return gps_nps_gps_nps_periodic_check_status;
    case 6: return autopilot_mode_auto2;
    case 7: return autopilot.kill_throttle;
    case 8: return autopilot.power_switch;
    case 9: return autopilot.mode;
    case 10: return viewvideo.use_rtp;
    case 11: return opticflow.method;
    case 12: return opticflow.corner_method;
    case 13: return opticflow.window_size;
    case 14: return opticflow.search_distance;
    case 15: return opticflow.subpixel_factor;
    case 16: return opticflow.resolution_factor;
    case 17: return opticflow.derotation;
    case 18: return opticflow.median_filter;
    case 19: return opticflow.feature_management;
    case 20: return opticflow.track_back;
    case 21: return opticflow.show_flow;
    case 22: return opticflow.max_track_corners;
    case 23: return opticflow.max_iterations;
    case 24: return opticflow.threshold_vec;
    case 25: return opticflow.fast9_adaptive;
    case 26: return opticflow.fast9_threshold;
    case 27: return opticflow.fast9_min_distance;
    case 28: return opticflow.fast9_padding;
    case 29: return opticflow.fast9_region_detect;
    case 30: return opticflow.fast9_num_regions;
    case 31: return opticflow.actfast_long_step;
    case 32: return opticflow.actfast_short_step;
    case 33: return opticflow.actfast_min_gradient;
    case 34: return opticflow.actfast_gradient_method;
    case 35: return opticflow.pyramid_level;
    case 36: return cod_lum_min1;
    case 37: return cod_lum_max1;
    case 38: return cod_cb_min1;
    case 39: return cod_cb_max1;
    case 40: return cod_cr_min1;
    case 41: return cod_cr_max1;
    case 42: return cod_draw1;
    case 43: return cod_lum_min2;
    case 44: return cod_lum_max2;
    case 45: return cod_cb_min2;
    case 46: return cod_cb_max2;
    case 47: return cod_cr_min2;
    case 48: return cod_cr_max2;
    case 49: return cod_draw2;
    case 50: return video_capture_record_video;
    case 51: return video_capture_take_shot;
    case 52: return mt9f002.set_zoom;
    case 53: return mt9f002.set_offset_x;
    case 54: return mt9f002.set_offset_y;
    case 55: return mt9f002_send_resolution;
    case 56: return mt9f002.gain_green1;
    case 57: return mt9f002.gain_green2;
    case 58: return mt9f002.gain_blue;
    case 59: return mt9f002.gain_red;
    case 60: return mt9f002_send_color;
    case 61: return mt9f002.target_exposure ;
    case 62: return mt9f002_send_exposure;
    case 63: return div_thresh;
    case 64: return heading_increment;
    case 65: return vff.accel_noise;
    case 66: return vff.r_baro;
    case 67: return vff.r_alt;
    case 68: return vff.r_obs_height;
    case 69: return ahrs_icq.gravity_heuristic_factor;
    case 70: return ahrs_icq.accel_omega;
    case 71: return ahrs_icq.accel_zeta;
    case 72: return ahrs_icq.mag_omega;
    case 73: return ahrs_icq.mag_zeta;
    case 74: return guidance_h.use_ref;
    case 75: return gh_ref.max_speed;
    case 76: return guidance_h.approx_force_by_thrust;
    case 77: return gh_ref.tau;
    case 78: return gh_ref.omega;
    case 79: return gh_ref.zeta;
    case 80: return guidance_h.gains.p;
    case 81: return guidance_h.gains.d;
    case 82: return guidance_h.gains.i;
    case 83: return guidance_h.gains.v;
    case 84: return guidance_h.gains.a;
    case 85: return guidance_h.sp.pos.x;
    case 86: return guidance_h.sp.pos.y;
    case 87: return guidance_v_kp;
    case 88: return guidance_v_kd;
    case 89: return guidance_v_ki;
    case 90: return guidance_v_nominal_throttle;
    case 91: return guidance_v_adapt_throttle_enabled;
    case 92: return guidance_v_z_sp;
    case 93: return flight_altitude;
    case 94: return nav_heading;
    case 95: return nav_radius;
    case 96: return nav_climb_vspeed;
    case 97: return nav_descend_vspeed;
    case 98: return indi.gains.att.p;
    case 99: return indi.gains.rate.p;
    case 100: return indi.g1.p;
    case 101: return indi.gains.att.q;
    case 102: return indi.gains.rate.q;
    case 103: return indi.g1.q;
    case 104: return indi.gains.att.r;
    case 105: return indi.gains.rate.r;
    case 106: return indi.g1.r;
    case 107: return indi.g2;
    case 108: return indi.adaptive;
    case 109: return indi.max_rate;
    case 110: return indi.attitude_max_yaw_rate;
    case 111: return multi_gps_mode;
    case 112: return imu.body_to_imu.eulers_f.phi;
    case 113: return imu.body_to_imu.eulers_f.theta;
    case 114: return imu.body_to_imu.eulers_f.psi;
    case 115: return imu.b2i_set_current;
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
