/*
 * Copyright (C) 2008-2010 The Paparazzi Team
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file subsystems/ins/ins_int.c
 *
 * INS for rotorcrafts combining vertical and horizontal filters.
 *
 */

#include "subsystems/ins/ins_int.h"

#include "subsystems/abi.h"

#include "subsystems/imu.h"
#include "subsystems/gps.h"

#include "generated/airframe.h"

#if USE_VFF
#include "subsystems/ins/vf_float.h"
#endif

#if USE_HFF
#include "subsystems/ins/hf_float.h"
#endif

#ifdef SITL
#include "nps_fdm.h"
#include <stdio.h>
#endif

#include "math/pprz_geodetic_int.h"
#include "math/pprz_isa.h"

#include "generated/flight_plan.h"

#ifndef USE_INS_NAV_INIT
#define USE_INS_NAV_INIT TRUE
PRINT_CONFIG_MSG("USE_INS_NAV_INIT defaulting to TRUE")
#endif

/* gps transformed to LTP-NED  */
struct LtpDef_i  ins_ltp_def;
         bool_t  ins_ltp_initialised;
struct NedCoor_i ins_gps_pos_cm_ned;
struct NedCoor_i ins_gps_speed_cm_s_ned;
#if USE_HFF
/* horizontal gps transformed to NED in meters as float */
struct FloatVect2 ins_gps_pos_m_ned;
struct FloatVect2 ins_gps_speed_m_s_ned;
#endif

/* barometer                   */
#if USE_VFF
bool_t ins_baro_initialised;
float ins_qfe;
float ins_baro_alt;
#ifndef INS_BARO_ID
#define INS_BARO_ID ABI_BROADCAST
#endif
abi_event baro_ev;
static void baro_cb(uint8_t sender_id, const float *pressure);
#endif

/* output                      */
struct NedCoor_i ins_ltp_pos;
struct NedCoor_i ins_ltp_speed;
struct NedCoor_i ins_ltp_accel;


void ins_init() {
#if USE_INS_NAV_INIT
  ins_ltp_initialised = TRUE;

  /** FIXME: should use the same code than MOVE_WP in firmwares/rotorcraft/datalink.c */
  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = INT32_RAD_OF_DEG(NAV_LAT0);
  llh_nav0.lon = INT32_RAD_OF_DEG(NAV_LON0);
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_i(&ins_ltp_def, &ecef_nav0);
  ins_ltp_def.hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(&ins_ltp_def);
#else
  ins_ltp_initialised  = FALSE;
#endif
#if USE_VFF
  // Bind to BARO_ABS message
  AbiBindMsgBARO_ABS(INS_BARO_ID, &baro_ev, baro_cb);
  ins_baro_initialised = FALSE;
  vff_init(0., 0., 0.);
#endif
  ins.vf_realign = FALSE;
  ins.hf_realign = FALSE;
#if USE_HFF
  b2_hff_init(0., 0., 0., 0.);
#endif
  INT32_VECT3_ZERO(ins_ltp_pos);
  INT32_VECT3_ZERO(ins_ltp_speed);
  INT32_VECT3_ZERO(ins_ltp_accel);

  // TODO correct init
  ins.status = INS_RUNNING;

}

void ins_periodic( void ) {
}

void ins_realign_h(struct FloatVect2 pos __attribute__ ((unused)), struct FloatVect2 speed __attribute__ ((unused))) {
#if USE_HFF
  b2_hff_realign(pos, speed);
#endif /* USE_HFF */
}

void ins_realign_v(float z __attribute__ ((unused))) {
#if USE_VFF
  vff_realign(z);
#endif
}

void ins_propagate() {
  /* untilt accels */
  struct Int32Vect3 accel_meas_body;
  INT32_RMAT_TRANSP_VMULT(accel_meas_body, imu.body_to_imu_rmat, imu.accel);
  struct Int32Vect3 accel_meas_ltp;
  INT32_RMAT_TRANSP_VMULT(accel_meas_ltp, (*stateGetNedToBodyRMat_i()), accel_meas_body);

#if USE_VFF
  float z_accel_meas_float = ACCEL_FLOAT_OF_BFP(accel_meas_ltp.z);
  if (ins_baro_initialised) {
    vff_propagate(z_accel_meas_float);
    ins_ltp_accel.z = ACCEL_BFP_OF_REAL(vff_zdotdot);
    ins_ltp_speed.z = SPEED_BFP_OF_REAL(vff_zdot);
    ins_ltp_pos.z   = POS_BFP_OF_REAL(vff_z);
  }
  else { // feed accel from the sensors
    // subtract -9.81m/s2 (acceleration measured due to gravity, but vehivle not accelerating in ltp)
    ins_ltp_accel.z = accel_meas_ltp.z + ACCEL_BFP_OF_REAL(9.81);
  }
#else
  ins_ltp_accel.z = accel_meas_ltp.z + ACCEL_BFP_OF_REAL(9.81);
#endif /* USE_VFF */

#if USE_HFF
  /* propagate horizontal filter */
  b2_hff_propagate();
#else
  ins_ltp_accel.x = accel_meas_ltp.x;
  ins_ltp_accel.y = accel_meas_ltp.y;
#endif /* USE_HFF */

  INS_NED_TO_STATE();
}

#if USE_VFF
static void baro_cb(uint8_t __attribute__((unused)) sender_id, const float *pressure) {
  if (!ins_baro_initialised) {
    ins_qfe = *pressure;
    ins_baro_initialised = TRUE;
  }
  if (ins.vf_realign) {
    ins.vf_realign = FALSE;
    ins_qfe = *pressure;
    vff_realign(0.);
    ins_ltp_accel.z = ACCEL_BFP_OF_REAL(vff_zdotdot);
    ins_ltp_speed.z = SPEED_BFP_OF_REAL(vff_zdot);
    ins_ltp_pos.z   = POS_BFP_OF_REAL(vff_z);
  }
  else {
    ins_baro_alt = pprz_isa_height_of_pressure(*pressure, ins_qfe);
    vff_update(ins_baro_alt);
  }
  INS_NED_TO_STATE();
}
#endif

void ins_update_baro() {
}


void ins_update_gps(void) {
#if USE_GPS
  if (gps.fix == GPS_FIX_3D) {
    if (!ins_ltp_initialised) {
      ltp_def_from_ecef_i(&ins_ltp_def, &gps.ecef_pos);
      ins_ltp_def.lla.alt = gps.lla_pos.alt;
      ins_ltp_def.hmsl = gps.hmsl;
      ins_ltp_initialised = TRUE;
      stateSetLocalOrigin_i(&ins_ltp_def);
    }
    ned_of_ecef_point_i(&ins_gps_pos_cm_ned, &ins_ltp_def, &gps.ecef_pos);
    ned_of_ecef_vect_i(&ins_gps_speed_cm_s_ned, &ins_ltp_def, &gps.ecef_vel);
#if USE_HFF
    VECT2_ASSIGN(ins_gps_pos_m_ned, ins_gps_pos_cm_ned.x, ins_gps_pos_cm_ned.y);
    VECT2_SDIV(ins_gps_pos_m_ned, ins_gps_pos_m_ned, 100.);
    VECT2_ASSIGN(ins_gps_speed_m_s_ned, ins_gps_speed_cm_s_ned.x, ins_gps_speed_cm_s_ned.y);
    VECT2_SDIV(ins_gps_speed_m_s_ned, ins_gps_speed_m_s_ned, 100.);
    if (ins.hf_realign) {
      ins.hf_realign = FALSE;
#ifdef SITL
      struct FloatVect2 true_pos, true_speed;
      VECT2_COPY(true_pos, fdm.ltpprz_pos);
      VECT2_COPY(true_speed, fdm.ltpprz_ecef_vel);
      b2_hff_realign(true_pos, true_speed);
#else
      const struct FloatVect2 zero = {0.0, 0.0};
      b2_hff_realign(ins_gps_pos_m_ned, zero);
#endif
    }
    b2_hff_update_gps();
#if !USE_VFF /* vff not used */
    ins_ltp_pos.z =  (ins_gps_pos_cm_ned.z * INT32_POS_OF_CM_NUM) / INT32_POS_OF_CM_DEN;
    ins_ltp_speed.z =  (ins_gps_speed_cm_s_ned.z * INT32_SPEED_OF_CM_S_NUM) INT32_SPEED_OF_CM_S_DEN;
#endif /* vff not used */
#endif /* hff used */


#if !USE_HFF /* hff not used */
#if !USE_VFF /* neither hf nor vf used */
    INT32_VECT3_SCALE_3(ins_ltp_pos, ins_gps_pos_cm_ned, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
    INT32_VECT3_SCALE_3(ins_ltp_speed, ins_gps_speed_cm_s_ned, INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);
#else /* only vff used */
    INT32_VECT2_SCALE_2(ins_ltp_pos, ins_gps_pos_cm_ned, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
    INT32_VECT2_SCALE_2(ins_ltp_speed, ins_gps_speed_cm_s_ned, INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);
#endif

#if USE_GPS_LAG_HACK
    VECT2_COPY(d_pos, ins_ltp_speed);
    INT32_VECT2_RSHIFT(d_pos, d_pos, 11);
    VECT2_ADD(ins_ltp_pos, d_pos);
#endif
#endif /* hff not used */

    INS_NED_TO_STATE();
  }
#endif /* USE_GPS */
}

void ins_update_sonar() {
}
