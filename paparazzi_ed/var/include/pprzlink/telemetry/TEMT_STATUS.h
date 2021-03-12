/** @file
 *  @brief PPRZLink message header for TEMT_STATUS in class telemetry
 *
 *  
 *  @see http://paparazziuav.org
 */

#ifndef _VAR_MESSAGES_telemetry_TEMT_STATUS_H_
#define _VAR_MESSAGES_telemetry_TEMT_STATUS_H_


#include "pprzlink/pprzlink_device.h"
#include "pprzlink/pprzlink_transport.h"
#include "pprzlink/pprzlink_utils.h"
#include "pprzlink/pprzlink_message.h"


#ifdef __cplusplus
extern "C" {
#endif

#if DOWNLINK

#define DL_TEMT_STATUS 97
#define PPRZ_MSG_ID_TEMT_STATUS 97

/**
 * Macro that redirect calls to the default version of pprzlink API
 * Used for compatibility between versions.
 */
#define pprzlink_msg_send_TEMT_STATUS _send_msg(TEMT_STATUS,PPRZLINK_DEFAULT_VER)

/**
 * Sends a TEMT_STATUS message (API V2.0 version)
 *
 * @param msg the pprzlink_msg structure for this message
 * @param _light 
 * @param _f_light 
 */
static inline void pprzlink_msg_v2_send_TEMT_STATUS(struct pprzlink_msg * msg, uint16_t *_light, float *_f_light) {
#if PPRZLINK_ENABLE_FD
  long _FD = 0; /* can be an address, an index, a file descriptor, ... */
#endif
  const uint8_t size = msg->trans->size_of(msg, /* msg header overhead */4+2+4);
  if (msg->trans->check_available_space(msg, _FD_ADDR, size)) {
    msg->trans->count_bytes(msg, size);
    msg->trans->start_message(msg, _FD, /* msg header overhead */4+2+4);
    msg->trans->put_bytes(msg, _FD, DL_TYPE_UINT8, DL_FORMAT_SCALAR, &(msg->sender_id), 1);
    msg->trans->put_named_byte(msg, _FD, DL_TYPE_UINT8, DL_FORMAT_SCALAR, msg->receiver_id, NULL);
    uint8_t comp_class = (msg->component_id & 0x0F) << 4 | (1 & 0x0F);
    msg->trans->put_named_byte(msg, _FD, DL_TYPE_UINT8, DL_FORMAT_SCALAR, comp_class, NULL);
    msg->trans->put_named_byte(msg, _FD, DL_TYPE_UINT8, DL_FORMAT_SCALAR, DL_TEMT_STATUS, "TEMT_STATUS");
    msg->trans->put_bytes(msg, _FD, DL_TYPE_UINT16, DL_FORMAT_SCALAR, (void *) _light, 2);
    msg->trans->put_bytes(msg, _FD, DL_TYPE_FLOAT, DL_FORMAT_SCALAR, (void *) _f_light, 4);
    msg->trans->end_message(msg, _FD);
  } else
        msg->trans->overrun(msg);
}

// Compatibility with the protocol v1.0 API
#define pprzlink_msg_v1_send_TEMT_STATUS pprz_msg_send_TEMT_STATUS
#define DOWNLINK_SEND_TEMT_STATUS(_trans, _dev, light, f_light) pprz_msg_send_TEMT_STATUS(&((_trans).trans_tx), &((_dev).device), AC_ID, light, f_light)
/**
 * Sends a TEMT_STATUS message (API V1.0 version)
 *
 * @param trans A pointer to the transport_tx structure used for sending the message
 * @param dev A pointer to the link_device structure through which the message will be sent
 * @param ac_id The id of the sender of the message
 * @param _light 
 * @param _f_light 
 */
static inline void pprz_msg_send_TEMT_STATUS(struct transport_tx *trans, struct link_device *dev, uint8_t ac_id, uint16_t *_light, float *_f_light) {
    struct pprzlink_msg msg;
    msg.trans = trans;
    msg.dev = dev;
    msg.sender_id = ac_id;
    msg.receiver_id = 0;
    msg.component_id = 0;
    pprzlink_msg_v2_send_TEMT_STATUS(&msg,_light,_f_light);
}


#else // DOWNLINK

#define DOWNLINK_SEND_TEMT_STATUS(_trans, _dev, light, f_light) {}
static inline void pprz_send_msg_TEMT_STATUS(struct transport_tx *trans __attribute__((unused)), struct link_device *dev __attribute__((unused)), uint8_t ac_id __attribute__((unused)), uint16_t *_light __attribute__((unused)), float *_f_light __attribute__((unused))) {}

#endif // DOWNLINK


/** Getter for field light in message TEMT_STATUS
  *
  * @param _payload : a pointer to the TEMT_STATUS message
  * @return 
  */
static inline uint16_t pprzlink_get_DL_TEMT_STATUS_light(uint8_t * _payload __attribute__((unused)))
{
    return _PPRZ_VAL_uint16_t(_payload, 4);
}


/** Getter for field f_light in message TEMT_STATUS
  *
  * @param _payload : a pointer to the TEMT_STATUS message
  * @return 
  */
static inline float pprzlink_get_DL_TEMT_STATUS_f_light(uint8_t * _payload __attribute__((unused)))
{
    return _PPRZ_VAL_float(_payload, 6);
}


/* Compatibility macros */
#define DL_TEMT_STATUS_light(_payload) pprzlink_get_DL_TEMT_STATUS_light(_payload)
#define DL_TEMT_STATUS_f_light(_payload) pprzlink_get_DL_TEMT_STATUS_f_light(_payload)



#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _VAR_MESSAGES_telemetry_TEMT_STATUS_H_

