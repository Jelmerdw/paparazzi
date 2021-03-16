/* fake generated airframe file */

#ifndef AIRFRAME_H
#define AIRFRAME_H

#define PERIODIC_FREQUENCY 512

#if defined STABILIZATION_ATTITUDE_TYPE_FLOAT

#define SECTION_STABILIZATION_ATTITUDE 1
#define STABILIZATION_ATTITUDE_SP_MAX_PHI 0.7853981625
#define STABILIZATION_ATTITUDE_SP_MAX_THETA 0.7853981625
#define STABILIZATION_ATTITUDE_SP_MAX_R 1.570796325

#define STABILIZATION_ATTITUDE_REF_OMEGA_P {RadOfDeg(400)}
#define STABILIZATION_ATTITUDE_REF_ZETA_P {0.85}
#define STABILIZATION_ATTITUDE_REF_MAX_P 6.981317
#define STABILIZATION_ATTITUDE_REF_MAX_PDOT RadOfDeg(4000.)
#define STABILIZATION_ATTITUDE_REF_OMEGA_Q {RadOfDeg(400)}
#define STABILIZATION_ATTITUDE_REF_ZETA_Q {0.85}
#define STABILIZATION_ATTITUDE_REF_MAX_Q 6.981317
#define STABILIZATION_ATTITUDE_REF_MAX_QDOT RadOfDeg(4000.)
#define STABILIZATION_ATTITUDE_REF_OMEGA_R {RadOfDeg(250)}
#define STABILIZATION_ATTITUDE_REF_ZETA_R {0.85}
#define STABILIZATION_ATTITUDE_REF_MAX_R 3.14159265
#define STABILIZATION_ATTITUDE_REF_MAX_RDOT RadOfDeg(1800.)

#elif defined STABILIZATION_ATTITUDE_TYPE_INT

#define SECTION_STABILIZATION_ATTITUDE 1
#define STABILIZATION_ATTITUDE_SP_MAX_PHI 0.7853981625
#define STABILIZATION_ATTITUDE_SP_MAX_THETA 0.7853981625
#define STABILIZATION_ATTITUDE_SP_MAX_R 1.570796325

#define STABILIZATION_ATTITUDE_REF_OMEGA_P 6.981317
#define STABILIZATION_ATTITUDE_REF_ZETA_P 0.85
#define STABILIZATION_ATTITUDE_REF_MAX_P 6.981317
#define STABILIZATION_ATTITUDE_REF_MAX_PDOT RadOfDeg(4000.)
#define STABILIZATION_ATTITUDE_REF_OMEGA_Q 6.981317
#define STABILIZATION_ATTITUDE_REF_ZETA_Q 0.85
#define STABILIZATION_ATTITUDE_REF_MAX_Q 6.981317
#define STABILIZATION_ATTITUDE_REF_MAX_QDOT RadOfDeg(4000.)
#define STABILIZATION_ATTITUDE_REF_OMEGA_R 4.363323125
#define STABILIZATION_ATTITUDE_REF_ZETA_R 0.85
#define STABILIZATION_ATTITUDE_REF_MAX_R 3.14159265
#define STABILIZATION_ATTITUDE_REF_MAX_RDOT RadOfDeg(1800.)

#endif


#endif // AIRFRAME_H
