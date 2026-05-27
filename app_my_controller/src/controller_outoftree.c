#include "param.h"
#include "num.h"
#include "math3d.h"
#include "physicalConstants.h"
#include "stabilizer_types.h"

#define CF_MASS 0.027f

typedef struct {
  bool initialized;
  struct vec prev_D_C;
  struct vec prev_Phi;
  struct vec prev_a_Phi;
  struct vec prev_phi;
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

static float k_G = 1.0f;
static float v_r = 1.0f;
static float k_p = 1.5f;
static float k_v = 1.2f;
static float k_o = 1.0f;
static float k_pv = 1.0f;
static float k_w = 2.0f;
static float k_pvo = 1.0f;
static float k_T = 2.0f;
static float omega_yaw = 0.0f;
static float sign_direction = 1.0f;

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
  struct quat v_quat = qqmul2(q, qqmul2(quatvw(v, 0.0f), qinv(q)));
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

void controllerPseudoReset(controllerPseudo_t* self) {
  self->initialized = false;
  self->prev_D_C = vzero();
  self->prev_Phi = vzero();
  self->prev_a_Phi = vzero();
  self->prev_phi = vzero();
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

#define UPDATE_RATE RATE_250_HZ

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

  const float dt = (float)(1.0f/UPDATE_RATE);

  const struct vec p = mkvec(state->position.x, state->position.y, state->position.z);
  const struct vec v = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);
  const struct quat o = qnormalize(mkquat(
    state->attitudeQuaternion.x,
    state->attitudeQuaternion.y,
    state->attitudeQuaternion.z,
    state->attitudeQuaternion.w
  ));
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
    self->prev_T1 = self->uT;
  }

  // Approximate path primitive from setpoint fields: c_star from position and T_C from desired velocity.
  const struct vec c_star = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
  struct vec T_C = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);

  const struct vec D_C = vsub(p, c_star);

  const float D_norm = vmag(D_C);
  const float G_gain = (2.0f/(float)M_PI) * atanf(k_G * D_norm);
  const float H_arg = fmaxf(0.0f, 1.0f - G_gain*G_gain);
  const float H_gain = sign_direction * sqrtf(H_arg);

  const struct vec D_dir = vnormalize_safe(D_C);
  T_C = vnormalize_safe(T_C);

  const struct vec Phi_S = vadd(vscl(-G_gain, D_dir), vscl(H_gain, T_C));

  // Phi_T = -Proj_null(T_C) * d(D_C)/dt, with d(D_C)/dt approximated numerically.
  const struct vec D_C_dot = vscl(1.0f/dt, vsub(D_C, self->prev_D_C));
  self->prev_D_C = D_C;

  const float tdot = vdot(T_C, D_C_dot);
  const struct vec proj_t = vscl(tdot, T_C);
  const struct vec proj_null_D = vsub(D_C_dot, proj_t);
  const struct vec Phi_T = vneg(proj_null_D);

  const float pspt = vdot(Phi_S, Phi_T);
  const float eta_inside = fmaxf(0.0f, pspt*pspt + v_r*v_r - vmag2(Phi_T));
  const float eta = -pspt + sqrtf(eta_inside);

  const struct vec Phi = vadd(vscl(eta, Phi_S), Phi_T);
  const struct vec Phi_dot = vscl(1.0f/dt, vsub(Phi, self->prev_Phi));
  self->prev_Phi = Phi;

  // Step 2
  const struct vec z_v = vsub(v, Phi);
  const struct vec k_hat = mkvec(0.0f, 0.0f, 1.0f);
  const struct vec a_Phi = vadd4(
    vscl(GRAVITY_MAGNITUDE, k_hat),
    vscl(-k_p, D_C),
    vscl(-k_v, z_v),
    Phi_dot
  );
  const struct vec a_Phi_dot = vscl(1.0f/dt, vsub(a_Phi, self->prev_a_Phi));
  self->prev_a_Phi = a_Phi;

  // Step 3
  const struct vec k_B = qvrot2(k_hat, o);
  const float uT_safe = clamp_positive(self->uT, 0.05f);
  const float uT_dot_prev = self->uT_dot;

  const struct vec z_o = vsub(vscl(uT_safe/CF_MASS, k_B), a_Phi);

  const struct vec Phi_L = vadd3(
    a_Phi_dot,
    vscl(-k_o, z_o),
    vscl(-k_pv, z_v)
  );

  const struct vec phi_rot = qvrot2(Phi_L, qinv(o));
  const struct vec phi = vadd(
    vscl(CF_MASS/uT_safe, vcross(k_hat, phi_rot)),
    vscl(omega_yaw, k_hat)
  );

  const struct vec phi_dot = vscl(1.0f/dt, vsub(phi, self->prev_phi));
  self->prev_phi = phi;

  const float T1 = CF_MASS * vdot(Phi_L, k_B);
  const float T1_dot = (T1 - self->prev_T1) / dt;
  self->prev_T1 = T1;

  // Step 4
  const struct vec z_w = vsub(w, phi);
  const float z_T = uT_dot_prev - T1;

  const struct vec coriolis = vcross(w, mvmul(CRAZYFLIE_INERTIA, w));
  const struct vec pvo_term = vcross(k_hat, qvrot2(z_o, qinv(o)));

  const struct vec tau_Phi = vadd4(
    coriolis,
    mvmul(CRAZYFLIE_INERTIA, phi_dot),
    vscl(-k_w, z_w),
    vscl(-k_pvo*(uT_safe/CF_MASS), pvo_term)
  );

  const float T2_Phi = T1_dot - k_T*z_T - (k_pvo/CF_MASS)*vdot(z_o, k_B);

  // Integration & output
  self->uT_dot = uT_dot_prev + T2_Phi*dt;
  self->uT = clamp_positive(self->uT + self->uT_dot*dt, 0.0f);

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
PARAM_GROUP_STOP(ctrlPseudo)
