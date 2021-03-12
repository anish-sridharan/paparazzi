/** @file
 *  @brief PPRZLink message header for FORMATION_STATUS_TM in class telemetry
 *
 *  
 *  @see http://paparazziuav.org
 */

#ifndef _VAR_MESSAGES_telemetry_FORMATION_STATUS_TM_H_
#define _VAR_MESSAGES_telemetry_FORMATION_STATUS_TM_H_


#include "pprzlink/pprzlink_device.h"
#include "pprzlink/pprzlink_transport.h"
#include "pprzlink/pprzlink_utils.h"
#include "pprzlink/pprzlink_message.h"


#ifdef __cplusplus
extern "C" {
#endif

#if DOWNLINK

#define DL_FORMATION_STATUS_TM 83
#define PPRZ_MSG_ID_FORMATION_STATUS_TM 83

/**
 * Macro that redirect calls to the default version of pprzlink API
 * Used for compatibility between versions.
 */
#define pprzlink_msg_send_FORMATION_STATUS_TM _send_msg(FORMATION_STATUS_TM,PPRZLINK_DEFAULT_VER)

/**
 * Sends a FORMATION_STATUS_TM message (API V2.0 version)
 *
 * @param msg the pprzlink_msg structure for this message
 * @param _ac_id 
 * @param _leader_id 
 * @param _status 
 */
static inline void pprzlink_msg_v2_send_FORMATION_STATUS_TM(struct pprzlink_msg * msg, uint8_t *_ac_id, uint8_t *_leader_id, uint8_t *_status) {
#if PPRZLINK_ENABLE_FD
  long _FD = 0; /* can be an address, an index, a file descriptor, ... */
#endif
  const uint8_t size = msg->trans->size_of(msg, /* msg header overhead */4+1+1+1);
  if (msg->trans->check_available_space(msg, _FD_ADDR, size)) {
    msg->trans->count_bytes(msg, size);
    msg->trans->start_message(msg, _FD, /* msg header overhead */4+1+1+1);
    msg->trans->put_bytes(msg, _FD, DL_TYPE_UINT8, DL_FORMAT_SCALAR, &(msg->sender_id), 1);
    msg->trans->put_named_byte(msg, _FD, DL_TYPE_UINT8, DL_FORMAT_SCALAR, msg->receiver_id, NULL);
    uint8_t comp_class = (msg->component_id & 0x0F) << 4 | (1 & 0x0F);
    msg->trans->put_named_byte(msg, _FD, DL_TYPE_UINT8, DL_FORMAT_SCALAR, comp_class, NULL);
    msg->trans->put_named_byte(msg, _FD, DL_TYPE_UINT8, DL_FORMAT_SCALAR, DL_FORMATION_STATUS_TM, "FORMATION_STATUS_TM");
    msg->trans->put_bytes(msg, _FD, DL_TYPE_UINT8, DL_FORMAT_SCALAR, (void *) _ac_id, 1);
    msg->trans->put_bytes(msg, _FD, DL_TYPE_UINT8, DL_FORMAT_SCALAR, (void *) _leader_id, 1);
    msg->trans->put_bytes(msg, _FD, DL_TYPE_UINT8, DL_FORMAT_SCALAR, (void *) _status, 1);
    msg->trans->end_message(msg, _FD);
  } else
        msg->trans->overrun(msg);
}

// Compatibility with the protocol v1.0 API
#define pprzlink_msg_v1_send_FORMATION_STATUS_TM pprz_msg_send_FORMATION_STATUS_TM
#define DOWNLINK_SEND_FORMATION_STATUS_TM(_trans, _dev, ac_id, leader_id, status) pprz_msg_send_FORMATION_STATUS_TM(&((_trans).trans_tx), &((_dev).device), AC_ID, ac_id, leader_id, status)
/**
 * Sends a FORMATION_STATUS_TM message (API V1.0 version)
 *
 * @param trans A pointer to the transport_tx structure used for sending the message
 * @param dev A pointer to the link_device structure through which the message will be sent
 * @param ac_id The id of the sender of the message
 * @param _ac_id 
 * @param _leader_id 
 * @param _status 
 */
static inline void pprz_msg_send_FORMATION_STATUS_TM(struct transport_tx *trans, struct link_device *dev, uint8_t ac_id, uint8_t *_ac_id, uint8_t *_leader_id, uint8_t *_status) {
    struct pprzlink_msg msg;
    msg.trans = trans;
    msg.dev = dev;
    msg.sender_id = ac_id;
    msg.receiver_id = 0;
    msg.component_id = 0;
    pprzlink_msg_v2_send_FORMATION_STATUS_TM(&msg,_ac_id,_leader_id,_status);
}


#else // DOWNLINK

#define DOWNLINK_SEND_FORMATION_STATUS_TM(_trans, _dev, ac_id, leader_id, status) {}
static inline void pprz_send_msg_FORMATION_STATUS_TM(struct transport_tx *trans __attribute__((unused)), struct link_device *dev __attribute__((unused)), uint8_t ac_id __attribute__((unused)), uint8_t *_ac_id __attribute__((unused)), uint8_t *_leader_id __attribute__((unused)), uint8_t *_status __attribute__((unused))) {}

#endif // DOWNLINK


/** Getter for field ac_id in message FORMATION_STATUS_TM
  *
  * @param _payload : a pointer to the FORMATION_STATUS_TM message
  * @return 
  */
static inline uint8_t pprzlink_get_DL_FORMATION_STATUS_TM_ac_id(uint8_t * _payload __attribute__((unused)))
{
    return _PPRZ_VAL_uint8_t(_payload, 4);
}


/** Getter for field leader_id in message FORMATION_STATUS_TM
  *
  * @param _payload : a pointer to the FORMATION_STATUS_TM message
  * @return 
  */
static inline uint8_t pprzlink_get_DL_FORMATION_STATUS_TM_leader_id(uint8_t * _payload __attribute__((unused)))
{
    return _PPRZ_VAL_uint8_t(_payload, 5);
}


/** Getter for field status in message FORMATION_STATUS_TM
  *
  * @param _payload : a pointer to the FORMATION_STATUS_TM message
  * @return 
  */
static inline uint8_t pprzlink_get_DL_FORMATION_STATUS_TM_status(uint8_t * _payload __attribute__((unused)))
{
    return _PPRZ_VAL_uint8_t(_payload, 6);
}


/* Compatibility macros */
#define DL_FORMATION_STATUS_TM_ac_id(_payload) pprzlink_get_DL_FORMATION_STATUS_TM_ac_id(_payload)
#define DL_FORMATION_STATUS_TM_leader_id(_payload) pprzlink_get_DL_FORMATION_STATUS_TM_leader_id(_payload)
#define DL_FORMATION_STATUS_TM_status(_payload) pprzlink_get_DL_FORMATION_STATUS_TM_status(_payload)



#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _VAR_MESSAGES_telemetry_FORMATION_STATUS_TM_H_

