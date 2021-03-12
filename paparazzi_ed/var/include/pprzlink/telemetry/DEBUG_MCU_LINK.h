/** @file
 *  @brief PPRZLink message header for DEBUG_MCU_LINK in class telemetry
 *
 *  
 *  @see http://paparazziuav.org
 */

#ifndef _VAR_MESSAGES_telemetry_DEBUG_MCU_LINK_H_
#define _VAR_MESSAGES_telemetry_DEBUG_MCU_LINK_H_


#include "pprzlink/pprzlink_device.h"
#include "pprzlink/pprzlink_transport.h"
#include "pprzlink/pprzlink_utils.h"
#include "pprzlink/pprzlink_message.h"


#ifdef __cplusplus
extern "C" {
#endif

#if DOWNLINK

#define DL_DEBUG_MCU_LINK 13
#define PPRZ_MSG_ID_DEBUG_MCU_LINK 13

/**
 * Macro that redirect calls to the default version of pprzlink API
 * Used for compatibility between versions.
 */
#define pprzlink_msg_send_DEBUG_MCU_LINK _send_msg(DEBUG_MCU_LINK,PPRZLINK_DEFAULT_VER)

/**
 * Sends a DEBUG_MCU_LINK message (API V2.0 version)
 *
 * @param msg the pprzlink_msg structure for this message
 * @param _i2c_nb_err 
 * @param _i2c_mcu1_nb_err 
 * @param _ppm_rate 
 */
static inline void pprzlink_msg_v2_send_DEBUG_MCU_LINK(struct pprzlink_msg * msg, uint8_t *_i2c_nb_err, uint8_t *_i2c_mcu1_nb_err, uint8_t *_ppm_rate) {
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
    msg->trans->put_named_byte(msg, _FD, DL_TYPE_UINT8, DL_FORMAT_SCALAR, DL_DEBUG_MCU_LINK, "DEBUG_MCU_LINK");
    msg->trans->put_bytes(msg, _FD, DL_TYPE_UINT8, DL_FORMAT_SCALAR, (void *) _i2c_nb_err, 1);
    msg->trans->put_bytes(msg, _FD, DL_TYPE_UINT8, DL_FORMAT_SCALAR, (void *) _i2c_mcu1_nb_err, 1);
    msg->trans->put_bytes(msg, _FD, DL_TYPE_UINT8, DL_FORMAT_SCALAR, (void *) _ppm_rate, 1);
    msg->trans->end_message(msg, _FD);
  } else
        msg->trans->overrun(msg);
}

// Compatibility with the protocol v1.0 API
#define pprzlink_msg_v1_send_DEBUG_MCU_LINK pprz_msg_send_DEBUG_MCU_LINK
#define DOWNLINK_SEND_DEBUG_MCU_LINK(_trans, _dev, i2c_nb_err, i2c_mcu1_nb_err, ppm_rate) pprz_msg_send_DEBUG_MCU_LINK(&((_trans).trans_tx), &((_dev).device), AC_ID, i2c_nb_err, i2c_mcu1_nb_err, ppm_rate)
/**
 * Sends a DEBUG_MCU_LINK message (API V1.0 version)
 *
 * @param trans A pointer to the transport_tx structure used for sending the message
 * @param dev A pointer to the link_device structure through which the message will be sent
 * @param ac_id The id of the sender of the message
 * @param _i2c_nb_err 
 * @param _i2c_mcu1_nb_err 
 * @param _ppm_rate 
 */
static inline void pprz_msg_send_DEBUG_MCU_LINK(struct transport_tx *trans, struct link_device *dev, uint8_t ac_id, uint8_t *_i2c_nb_err, uint8_t *_i2c_mcu1_nb_err, uint8_t *_ppm_rate) {
    struct pprzlink_msg msg;
    msg.trans = trans;
    msg.dev = dev;
    msg.sender_id = ac_id;
    msg.receiver_id = 0;
    msg.component_id = 0;
    pprzlink_msg_v2_send_DEBUG_MCU_LINK(&msg,_i2c_nb_err,_i2c_mcu1_nb_err,_ppm_rate);
}


#else // DOWNLINK

#define DOWNLINK_SEND_DEBUG_MCU_LINK(_trans, _dev, i2c_nb_err, i2c_mcu1_nb_err, ppm_rate) {}
static inline void pprz_send_msg_DEBUG_MCU_LINK(struct transport_tx *trans __attribute__((unused)), struct link_device *dev __attribute__((unused)), uint8_t ac_id __attribute__((unused)), uint8_t *_i2c_nb_err __attribute__((unused)), uint8_t *_i2c_mcu1_nb_err __attribute__((unused)), uint8_t *_ppm_rate __attribute__((unused))) {}

#endif // DOWNLINK


/** Getter for field i2c_nb_err in message DEBUG_MCU_LINK
  *
  * @param _payload : a pointer to the DEBUG_MCU_LINK message
  * @return 
  */
static inline uint8_t pprzlink_get_DL_DEBUG_MCU_LINK_i2c_nb_err(uint8_t * _payload __attribute__((unused)))
{
    return _PPRZ_VAL_uint8_t(_payload, 4);
}


/** Getter for field i2c_mcu1_nb_err in message DEBUG_MCU_LINK
  *
  * @param _payload : a pointer to the DEBUG_MCU_LINK message
  * @return 
  */
static inline uint8_t pprzlink_get_DL_DEBUG_MCU_LINK_i2c_mcu1_nb_err(uint8_t * _payload __attribute__((unused)))
{
    return _PPRZ_VAL_uint8_t(_payload, 5);
}


/** Getter for field ppm_rate in message DEBUG_MCU_LINK
  *
  * @param _payload : a pointer to the DEBUG_MCU_LINK message
  * @return 
  */
static inline uint8_t pprzlink_get_DL_DEBUG_MCU_LINK_ppm_rate(uint8_t * _payload __attribute__((unused)))
{
    return _PPRZ_VAL_uint8_t(_payload, 6);
}


/* Compatibility macros */
#define DL_DEBUG_MCU_LINK_i2c_nb_err(_payload) pprzlink_get_DL_DEBUG_MCU_LINK_i2c_nb_err(_payload)
#define DL_DEBUG_MCU_LINK_i2c_mcu1_nb_err(_payload) pprzlink_get_DL_DEBUG_MCU_LINK_i2c_mcu1_nb_err(_payload)
#define DL_DEBUG_MCU_LINK_ppm_rate(_payload) pprzlink_get_DL_DEBUG_MCU_LINK_ppm_rate(_payload)



#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _VAR_MESSAGES_telemetry_DEBUG_MCU_LINK_H_

