/* This file has been generated from  */
/* Version v5.17_devel-99-gdbb1ea851-dirty */
/* Please DO NOT EDIT */

#ifndef MODULES_H
#define MODULES_H

#define MODULES_IDLE  0
#define MODULES_RUN   1
#define MODULES_START 2
#define MODULES_STOP  3

#ifndef MODULES_FREQUENCY
#ifdef PERIODIC_FREQUENCY
#define MODULES_FREQUENCY PERIODIC_FREQUENCY
#else
#error "neither MODULES_FREQUENCY or PERIODIC_FREQUENCY are defined"
#endif
#endif

#ifdef MODULES_C
#define EXTERN_MODULES
#else
#define EXTERN_MODULES extern
#endif
#include "std.h"
#include "computer_vision/viewvideo.h"
#include "pose_history/pose_history.h"
#include "computer_vision/video_thread.h"
#include "sonar/agl_dist.h"
#include "computer_vision/opticflow_module.h"
#include "computer_vision/cv_detect_color_object.h"
#include "computer_vision/video_capture.h"
#include "../boards/bebop/mt9v117.h"
#include "../boards/bebop/mt9f002.h"
#include "mav_course_exercise/mav_exercise.h"
#include "loggers/file_logger.h"
#include "subsystems/ins/ins_int.h"
#include "subsystems/ahrs.h"
#include "guidance/guidance_h.h"
#include "guidance/guidance_v.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "stabilization/stabilization_none.h"
#include "stabilization/stabilization_rate_indi.h"
#include "stabilization/stabilization_indi_simple.h"
#include "subsystems/gps.h"
#include "subsystems/gps.h"
#include "subsystems/imu.h"
#include "subsystems/imu/imu_nps.h"
#include "subsystems/actuators/motor_mixing.h"
#include "subsystems/radio_control/rc_datalink.h"
#include "datalink/pprz_dl.h"

#define POSE_PERIODIC_PERIOD (1.f / MODULES_FREQUENCY)
#define POSE_PERIODIC_FREQ (MODULES_FREQUENCY)
#define VIDEO_THREAD_PERIODIC_PERIOD (1. / (1))
#define VIDEO_THREAD_PERIODIC_FREQ (1)
#define OPTICFLOW_MODULE_RUN_PERIOD (1.f / MODULES_FREQUENCY)
#define OPTICFLOW_MODULE_RUN_FREQ (MODULES_FREQUENCY)
#define COLOR_OBJECT_DETECTOR_PERIODIC_PERIOD (1. / (50))
#define COLOR_OBJECT_DETECTOR_PERIODIC_FREQ (50)
#define MAV_EXERCISE_PERIODIC_PERIOD (1. / (4))
#define MAV_EXERCISE_PERIODIC_FREQ (4)
#define FILE_LOGGER_PERIODIC_PERIOD (0.01)
#define FILE_LOGGER_PERIODIC_FREQ (1. / (0.01))
#define GPS_NPS_PERIODIC_CHECK_PERIOD (1. / (1.))
#define GPS_NPS_PERIODIC_CHECK_FREQ (1.)

#define PRESCALER_1 (uint32_t)(MODULES_FREQUENCY * (1.f / MODULES_FREQUENCY))
#define PRESCALER_2 (uint32_t)(MODULES_FREQUENCY * (1. / (1)))
#define PRESCALER_1 (uint32_t)(MODULES_FREQUENCY * (1.f / MODULES_FREQUENCY))
#define PRESCALER_3 (uint32_t)(MODULES_FREQUENCY * (1. / (50)))
#define PRESCALER_4 (uint32_t)(MODULES_FREQUENCY * (1. / (4)))
#define PRESCALER_5 (uint32_t)(MODULES_FREQUENCY * (0.01))
#define PRESCALER_6 (uint32_t)(MODULES_FREQUENCY * (1. / (1.)))

EXTERN_MODULES uint8_t pose_history_pose_periodic_status;
EXTERN_MODULES uint8_t video_thread_video_thread_periodic_status;
EXTERN_MODULES uint8_t cv_opticflow_opticflow_module_run_status;
EXTERN_MODULES uint8_t logger_file_file_logger_periodic_status;
EXTERN_MODULES uint8_t gps_nps_gps_nps_periodic_check_status;


static inline void modules_datalink_init(void) {
  pprz_dl_init();
}

static inline void modules_default_init(void) {
  imu_nps_init();
  imu_init();
  gps_nps_init();
  gps_nps_gps_nps_periodic_check_status = MODULES_START;
  gps_init();
  stabilization_indi_init();
  stabilization_rate_init();
  stabilization_init();
  stabilization_none_init();
  nav_init();
  guidance_h_init();
  guidance_v_init();
  ins_int_init();
  logger_file_file_logger_periodic_status = MODULES_IDLE;
  mav_exercise_init();
  mt9v117_init(&mt9v117);
  mt9f002_init(&mt9f002);
  video_capture_init();
  color_object_detector_init();
  opticflow_module_init();
  cv_opticflow_opticflow_module_run_status = MODULES_START;
  agl_dist_init();
  video_thread_init();
  video_thread_video_thread_periodic_status = MODULES_START;
  pose_init();
  pose_history_pose_periodic_status = MODULES_START;
  viewvideo_init();
}

static inline void modules_actuators_init(void) {
  motor_mixing_init();
}

static inline void modules_init(void) {
  modules_datalink_init();
  modules_default_init();
  modules_actuators_init();
}

static inline void modules_datalink_periodic_task(void) {

}

static inline void modules_default_periodic_task(void) {
  static uint32_t i1; i1++; if (i1>=PRESCALER_1) i1=0;
  static uint32_t i2; i2++; if (i2>=PRESCALER_2) i2=0;
  static uint32_t i3; i3++; if (i3>=PRESCALER_3) i3=0;
  static uint32_t i4; i4++; if (i4>=PRESCALER_4) i4=0;
  static uint32_t i5; i5++; if (i5>=PRESCALER_5) i5=0;
  static uint32_t i6; i6++; if (i6>=PRESCALER_6) i6=0;

  if (gps_nps_gps_nps_periodic_check_status == MODULES_START) {
    gps_nps_gps_nps_periodic_check_status = MODULES_RUN;
  }
  if (gps_nps_gps_nps_periodic_check_status == MODULES_STOP) {
    gps_nps_gps_nps_periodic_check_status = MODULES_IDLE;
  }

  if (logger_file_file_logger_periodic_status == MODULES_START) {
    file_logger_start();
    logger_file_file_logger_periodic_status = MODULES_RUN;
  }
  if (logger_file_file_logger_periodic_status == MODULES_STOP) {
    file_logger_stop();
    logger_file_file_logger_periodic_status = MODULES_IDLE;
  }

  if (cv_opticflow_opticflow_module_run_status == MODULES_START) {
    cv_opticflow_opticflow_module_run_status = MODULES_RUN;
  }
  if (cv_opticflow_opticflow_module_run_status == MODULES_STOP) {
    cv_opticflow_opticflow_module_run_status = MODULES_IDLE;
  }

  if (video_thread_video_thread_periodic_status == MODULES_START) {
    video_thread_start();
    video_thread_video_thread_periodic_status = MODULES_RUN;
  }
  if (video_thread_video_thread_periodic_status == MODULES_STOP) {
    video_thread_stop();
    video_thread_video_thread_periodic_status = MODULES_IDLE;
  }

  if (pose_history_pose_periodic_status == MODULES_START) {
    pose_history_pose_periodic_status = MODULES_RUN;
  }
  if (pose_history_pose_periodic_status == MODULES_STOP) {
    pose_history_pose_periodic_status = MODULES_IDLE;
  }

  if (i5 == (uint32_t)(0.600000f * PRESCALER_5) && logger_file_file_logger_periodic_status == MODULES_RUN) {
    file_logger_periodic();
  }
  if (i2 == (uint32_t)(0.200000f * PRESCALER_2) && video_thread_video_thread_periodic_status == MODULES_RUN) {
    video_thread_periodic();
  }
  if (i6 == (uint32_t)(0.700000f * PRESCALER_6) && gps_nps_gps_nps_periodic_check_status == MODULES_RUN) {
    gps_nps_periodic_check();
  }
  if (i4 == (uint32_t)(0.500000f * PRESCALER_4)) {
    mav_exercise_periodic();
  }
  if (i3 == (uint32_t)(0.400000f * PRESCALER_3)) {
    color_object_detector_periodic();
  }
  if (pose_history_pose_periodic_status == MODULES_RUN) {
    pose_periodic();
  }
  if (cv_opticflow_opticflow_module_run_status == MODULES_RUN) {
    opticflow_module_run();
  }
}

static inline void modules_actuators_periodic_task(void) {

}

static inline void modules_periodic_task(void) {
  modules_datalink_periodic_task();
  modules_default_periodic_task();
  modules_actuators_periodic_task();
}

static inline void modules_datalink_event_task(void) {
  pprz_dl_event();
}

static inline void modules_default_event_task(void) {
  imu_nps_event();
}

static inline void modules_actuators_event_task(void) {
}

static inline void modules_event_task(void) {
  modules_datalink_event_task();
  modules_default_event_task();
  modules_actuators_event_task();
}

#ifdef MODULES_DATALINK_C

#include "pprzlink/messages.h"
#include "generated/airframe.h"
static inline void modules_parse_datalink(uint8_t msg_id __attribute__ ((unused)),
                                          struct link_device *dev __attribute__((unused)),
                                          struct transport_tx *trans __attribute__((unused)),
                                          uint8_t *buf __attribute__((unused))) {
}

#endif // MODULES_DATALINK_C

#endif // MODULES_H
