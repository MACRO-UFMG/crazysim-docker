#include "param.h"
#include "log.h"
#include "num.h"
#include "math3d.h"
#include "physicalConstants.h"
#include "stabilizer_types.h"
#include <math.h>

#define CF_MASS 0.0288f

typedef struct {
  bool initialized;
  struct vec prev_D_C;
  struct vec prev_Phi;
  struct vec prev_a_Phi;
  struct vec prev_phi;
  struct vec filt_D_C_dot;
  struct vec filt_Phi_dot;
  struct vec filt_a_Phi_dot;
  struct vec filt_phi_dot;
  float prev_T1;
  float uT;
  float uT_dot;
} controllerPseudo_t;

static controllerPseudo_t g_self = {
  .initialized = false,
};

static struct mat33 CRAZYFLIE_INERTIA =
    {{{16.6e-6f, 0.83e-6f, 0.72e-6f},
      {0.83e-6f, 16.6e-6f, 1.8e-6f},
      {0.72e-6f, 1.8e-6f, 29.3e-6f}}};

static float k_G = 2.0f;
static float v_r = 0.5f;
static float k_p = 0.5f;
static float k_v = 12.0f;

static float k_o = 10.68f;
static float k_pv = 0.0f;

static float k_w = 60.00f;
static float k_pvo = 0.0f;

static float k_T = 19.29f;
static float omega_yaw = 0.0f;
static float sign_direction = 1.0f;
static float deriv_alpha = 0.25f;
static uint8_t use_integrated_thrust = 0; // 0: no integration, 1: integrate T1, 2: integrate T2

static float dbg_uT = 0.0f;
static float dbg_uT_dot = 0.0f;
static float dbg_T0 = 0.0f;
static float dbg_T1 = 0.0f;
static float dbg_T2 = 0.0f;
static float dbg_zT = 0.0f;
static float dbg_DC_norm = 0.0f;
static float dbg_zv_norm = 0.0f;
static float dbg_zo_norm = 0.0f;
static float dbg_zw_norm = 0.0f;
static float dbg_phi_norm = 0.0f;
static float dbg_phi_dot_norm = 0.0f;

static inline struct quat qqmul2(struct quat q1, struct quat q2) {
  struct vec q1_vec = quatimagpart(q1);
  struct vec q2_vec = quatimagpart(q2);
  float w = q1.w*q2.w - vdot(q1_vec, q2_vec);
  struct vec v = vadd3(
    vscl(q1.w, q2_vec),
    vscl(q2.w, q1_vec),
    vcross(q1_vec, q2_vec)
  );
  return quatvw(v, w);
}

static inline struct vec qvrot2(struct vec v, struct quat q) {
  // struct quat v_quat = qqmul2(q, qqmul2(quatvw(v, 0.0f), qinv(q)));
  struct quat v_quat = qqmul2(qqmul2(q, quatvw(v, 0.0f)), qinv(q));
  return quatimagpart(v_quat);
}

static inline struct vec vnormalize_safe(struct vec v) {
  const float n = vmag(v);
  if (n < 1.0e-6f) {
    return vzero();
  }
  return vscl(1.0f/n, v);
}

static inline float clamp_positive(float x, float floor) {
  if (x < floor) {
    return floor;
  }
  return x;
}

static inline float clampf_local(float x, float min_val, float max_val) {
  if (x < min_val) {
    return min_val;
  }
  if (x > max_val) {
    return max_val;
  }
  return x;
}

static inline struct vec lpf_vec(struct vec prev, struct vec in, float alpha) {
  const float a = clampf_local(alpha, 0.0f, 1.0f);
  return vadd(vscl(1.0f - a, prev), vscl(a, in));
}

void controllerPseudoReset(controllerPseudo_t* self) {
  self->initialized = false;
  self->prev_D_C = vzero();
  self->prev_Phi = vzero();
  self->prev_a_Phi = vzero();
  self->prev_phi = vzero();
  self->filt_D_C_dot = vzero();
  self->filt_Phi_dot = vzero();
  self->filt_a_Phi_dot = vzero();
  self->filt_phi_dot = vzero();
  self->prev_T1 = 0.0f;
  self->uT = CF_MASS * GRAVITY_MAGNITUDE;
  self->uT_dot = 0.0f;
}

void controllerPseudoInit(controllerPseudo_t* self) {
  *self = g_self;
  controllerPseudoReset(self);
}

bool controllerPseudoTest(controllerPseudo_t* self) {
  return true;
}

#define UPDATE_RATE RATE_500_HZ

void controllerPseudo(
    controllerPseudo_t* self,
    control_t* control,
    const setpoint_t* setpoint,
    const sensorData_t* sensors,
    const state_t* state,
    const stabilizerStep_t stabilizerStep) {
  if (!RATE_DO_EXECUTE(UPDATE_RATE, stabilizerStep)) {
    return;
  }

  // minimum and maximum body rates
  static float omega_rp_max = 30;
  static float omega_yaw_max = 10;
  static float heuristic_rp = 12;
  static float heuristic_yaw = 5;


  const float dt = (float)(1.0f/UPDATE_RATE);

  const struct vec p = mkvec(state->position.x, state->position.y, state->position.z);
  const struct vec v = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);
  struct quat o = qnormalize(mkquat(
    state->attitudeQuaternion.x,
    state->attitudeQuaternion.y,
    state->attitudeQuaternion.z,
    state->attitudeQuaternion.w
  ));
  if (o.w < 0) {
    o = qneg(o);
  }
  const struct vec w = mkvec(
    radians(sensors->gyro.x),
    radians(sensors->gyro.y),
    radians(sensors->gyro.z)
  );

  if (!self->initialized) {
    self->initialized = true;
    self->prev_D_C = vzero();
    self->prev_Phi = v;
    self->prev_a_Phi = vzero();
    self->prev_phi = w;
    self->filt_D_C_dot = vzero();
    self->filt_Phi_dot = vzero();
    self->filt_a_Phi_dot = vzero();
    self->filt_phi_dot = vzero();
    self->prev_T1 = self->uT;
  }

  // Approximate path primitive from setpoint fields: c_star from position and T_C from desired velocity.
  const struct vec c_star = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
  const struct vec T_C = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
  // const struct vec T_C = vzero(); // Ablate T_C for testing

  struct vec D_C = vsub(p, c_star);
  // if (vmag(D_C) > 0.5f) {
  //     D_C = vscl(0.5f/vmag(D_C), D_C);    
  // }

  const float G_gain = (2.0f/(float)M_PI) * atanf(k_G * vmag(D_C));
  const float H_arg = fmaxf(0.0f, 1.0f - G_gain*G_gain);
  const float H_gain = sign_direction * sqrtf(H_arg);

  const struct vec Phi_S = vadd(
    vscl(-G_gain, vnormalize_safe(D_C)),
    vscl(H_gain, vnormalize_safe(T_C))
  );

  // // Phi_T = -Proj_null(T_C) * d(D_C)/dt, with d(D_C)/dt approximated numerically.
  // const struct vec D_C_dot_raw = vscl(1.0f/dt, vsub(D_C, self->prev_D_C));
  // self->filt_D_C_dot = lpf_vec(self->filt_D_C_dot, D_C_dot_raw, deriv_alpha);
  // const struct vec D_C_dot = self->filt_D_C_dot;
  // self->prev_D_C = D_C;
  // const float tdot = vdot(T_C, D_C_dot);
  // const struct vec proj_t = vscl(tdot, T_C);
  // const struct vec proj_null_D = vsub(D_C_dot, proj_t);
  // // const struct vec Phi_T = vneg(proj_null_D);
  // const struct vec Phi_T = vzero(); // Ablate Phi_T for testing
  // const float pspt = vdot(Phi_S, Phi_T);
  // const float eta_inside = fmaxf(0.0f, pspt*pspt + v_r*v_r - vmag2(Phi_T));
  // const float eta = -pspt + sqrtf(eta_inside);
  // const struct vec Phi = vadd(vscl(eta, Phi_S), Phi_T);
  
  const struct vec Phi = vscl(v_r, Phi_S);
  const struct vec Phi_dot_raw = vscl(1.0f/dt, vsub(Phi, self->prev_Phi));
  // self->filt_Phi_dot = lpf_vec(self->filt_Phi_dot, Phi_dot_raw, deriv_alpha);
  // const struct vec Phi_dot = self->filt_Phi_dot;
  self->prev_Phi = Phi;

  // // Step 2
  const struct vec z_v = vsub(v, Phi);
  const struct vec k_hat = mkvec(0.0f, 0.0f, 1.0f);
  const struct vec a_Phi = vadd4(
    vscl(GRAVITY_MAGNITUDE, k_hat),
    vscl(-k_p, D_C),
    vscl(-k_v, z_v),
    Phi_dot_raw
  );

  const struct vec k_B = qvrot2(k_hat, o);

  const float T0 = CF_MASS * vdot(a_Phi, k_B);
  // const float T0 = CF_MASS * vmag(a_Phi);
  const struct vec a_Phi_dot_raw = vscl(1.0f/dt, vsub(a_Phi, self->prev_a_Phi));
  self->filt_a_Phi_dot = lpf_vec(self->filt_a_Phi_dot, a_Phi_dot_raw, deriv_alpha);
  const struct vec a_Phi_dot = self->filt_a_Phi_dot;
  self->prev_a_Phi = a_Phi;

  if (use_integrated_thrust <= 0) {
    self->uT = T0;
  }

  // Step 3
  const struct vec z_o = vsub(vscl(self->uT/CF_MASS, k_B), a_Phi);

  const struct vec phi_L = vadd3(
    a_Phi_dot,
    vscl(-k_o, z_o),
    vscl(-k_pv, z_v)
  );
  const struct vec phi_rot = qvrot2(phi_L, qinv(o));
  struct vec tmp = vzero();
  if (self->uT == 0.0f) {
    tmp = vscl(0.0f, vcross(k_hat, phi_rot));
  } else {
    tmp = vscl(CF_MASS/self->uT, vcross(k_hat, phi_rot));
  }
  struct vec phi = vadd(
    tmp,
    vscl(omega_yaw, k_hat)
  );
  const float T1 = CF_MASS * vdot(phi_L, k_B);
  if (use_integrated_thrust == 1) {
    self->uT = self->uT + dt*T1;
  }

  const float T1_dot = (T1 - self->prev_T1) / dt;
  self->prev_T1 = T1;
  
  // apply the rotation heuristic
  if (phi.x * w.x < 0 && fabsf(w.x) > heuristic_rp) { // desired rotational rate in direction opposite to current rotational rate
    phi.x = omega_rp_max * (w.x < 0 ? -1 : 1); // maximum rotational rate in direction of current rotation
  }
  if (phi.y * w.y < 0 && fabsf(w.y) > heuristic_rp) { // desired rotational rate in direction opposite to current rotational rate
    phi.y = omega_rp_max * (w.y < 0 ? -1 : 1); // maximum rotational rate in direction of current rotation
  }
  if (phi.z * w.z < 0 && fabsf(w.z) > heuristic_yaw) { // desired rotational rate in direction opposite to current rotational rate
    phi.z = omega_yaw_max * (w.z < 0 ? -1 : 1); // maximum rotational rate in direction of current rotation
  }
  // scale the commands to satisfy rate constraints
  float scaling = 1;
  scaling = fmax(scaling, fabsf(phi.x) / omega_rp_max);
  scaling = fmax(scaling, fabsf(phi.y) / omega_rp_max);
  scaling = fmax(scaling, fabsf(phi.z) / omega_yaw_max);
  phi.x /= scaling;
  phi.y /= scaling;
  phi.z /= scaling;

  const struct vec phi_dot_raw = vscl(1.0f/dt, vsub(phi, self->prev_phi));
  // self->filt_phi_dot = lpf_vec(self->filt_phi_dot, phi_dot_raw, deriv_alpha);
  // const struct vec phi_dot = self->filt_phi_dot;
  self->prev_phi = phi;
  
  // Step 4
  const struct vec z_w = vsub(w, phi);
  const float z_T = self->uT_dot - T1;

  const struct vec coriolis = vcross(w, mvmul(CRAZYFLIE_INERTIA, w));
  const struct vec pvo_term = vcross(k_hat, qvrot2(z_o, qinv(o)));

  struct vec tau_Phi = vadd4(
    coriolis,
    mvmul(CRAZYFLIE_INERTIA, phi_dot_raw),
    vscl(-k_w, z_w),
    vscl(-k_pvo*(self->uT/CF_MASS), pvo_term)
  );

  tau_Phi = vadd(
    // mvmul(CRAZYFLIE_INERTIA, phi_dot_raw),
    vzero(),
    mvmul(CRAZYFLIE_INERTIA, vscl(-k_w, z_w))
  );

  const float T2_Phi = T1_dot - k_T*z_T - (k_pvo/CF_MASS)*vdot(z_o, k_B);

  if (use_integrated_thrust == 2) {
    self->uT_dot = self->uT_dot + T2_Phi*dt;
    self->uT = self->uT + self->uT_dot*dt;
  } 

  if (setpoint->mode.z == modeDisable) {
    control->thrustSi = 0.0f;
    control->torqueX = 0.0f;
    control->torqueY = 0.0f;
    control->torqueZ = 0.0f;
    controllerPseudoReset(self);
  } else {
    control->thrustSi = self->uT;
    control->torqueX = tau_Phi.x;
    control->torqueY = tau_Phi.y;
    control->torqueZ = tau_Phi.z;
  }
  control->controlMode = controlModeForceTorque;

  dbg_uT = self->uT;
  dbg_uT_dot = self->uT_dot;
  dbg_T0 = T0;
  dbg_T1 = T1;
  dbg_T2 = T2_Phi;
  dbg_zT = z_T;
  dbg_DC_norm = vmag(D_C);
  dbg_zv_norm = vmag(z_v);
  dbg_zo_norm = vmag(z_o);
  dbg_zw_norm = vmag(z_w);
  // dbg_phi_norm = vmag(phi);
  // dbg_phi_dot_norm = vmag(phi_dot);
}

void controllerOutOfTreeInit(void) {
  controllerPseudoInit(&g_self);
}

void controllerOutOfTree(
    control_t* control,
    const setpoint_t* setpoint,
    const sensorData_t* sensors,
    const state_t* state,
    const stabilizerStep_t stabilizerStep) {
  controllerPseudo(&g_self, control, setpoint, sensors, state, stabilizerStep);
}

bool controllerOutOfTreeTest(void) {
  return controllerPseudoTest(&g_self);
}

PARAM_GROUP_START(ctrlPseudo)
PARAM_ADD(PARAM_FLOAT, k_G, &k_G)
PARAM_ADD(PARAM_FLOAT, v_r, &v_r)
PARAM_ADD(PARAM_FLOAT, k_p, &k_p)
PARAM_ADD(PARAM_FLOAT, k_v, &k_v)
PARAM_ADD(PARAM_FLOAT, k_o, &k_o)
PARAM_ADD(PARAM_FLOAT, k_pv, &k_pv)
PARAM_ADD(PARAM_FLOAT, k_w, &k_w)
PARAM_ADD(PARAM_FLOAT, k_pvo, &k_pvo)
PARAM_ADD(PARAM_FLOAT, k_T, &k_T)
PARAM_ADD(PARAM_FLOAT, omega_yaw, &omega_yaw)
PARAM_ADD(PARAM_FLOAT, sign_dir, &sign_direction)
PARAM_ADD(PARAM_FLOAT, deriv_alpha, &deriv_alpha)
PARAM_ADD(PARAM_UINT8, use_uT_int, &use_integrated_thrust)
PARAM_GROUP_STOP(ctrlPseudo)

LOG_GROUP_START(ctrlPseudo)
LOG_ADD(LOG_FLOAT, uT, &dbg_uT)
LOG_ADD(LOG_FLOAT, uT_dot, &dbg_uT_dot)
LOG_ADD(LOG_FLOAT, T0, &dbg_T0)
LOG_ADD(LOG_FLOAT, T1, &dbg_T1)
LOG_ADD(LOG_FLOAT, T2, &dbg_T2)
LOG_ADD(LOG_FLOAT, zT, &dbg_zT)
LOG_ADD(LOG_FLOAT, zv_norm, &dbg_zv_norm)
LOG_ADD(LOG_FLOAT, DC_norm, &dbg_DC_norm)
LOG_ADD(LOG_FLOAT, zo_norm, &dbg_zo_norm)
LOG_ADD(LOG_FLOAT, zw_norm, &dbg_zw_norm)
LOG_ADD(LOG_FLOAT, phi_norm, &dbg_phi_norm)
LOG_ADD(LOG_FLOAT, phi_dot_norm, &dbg_phi_dot_norm)
LOG_GROUP_STOP(ctrlPseudo)
