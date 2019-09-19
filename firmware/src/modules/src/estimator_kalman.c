/**
 * Authored by Martin Ziran Xu within his Bachelor Thesis at the Technical University of Berlin, October 2018
 * Topic: Implementation of model-based nonlinear state estimation algorithms on a multicopter
 *
 * The structure of the stateEstimatorUpdate() function and the interface for reading and processing measurements are based on
 * Michael Hamer's (http://www.mikehamer.info) estimator_kalman.c, which can be found under:
 * https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/estimator_kalman.c
 *
 * Pritpal Singh Manpuria has implemented a linear Kalman Filter based on Hamer's Firmware within his Master Thesis.
 * Within the bachelor thesis this code has been modified to perform a model-based state estimation algorithm
 *
 */

#ifdef PLATFORM_CF1
#error ESTIMATOR = kalman is only compatible with the Crazyflie 2.0 // since it requires an FPU
#endif

#include "estimator_kalman.h"

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "sensors.h"

#include "log.h"
#include "param.h"

#include "math.h"
#include "arm_math.h"



/**
 * Primary Extended Kalman filter functions:
 * xp_EKF and Pp_EKF: prior mean and variance, equals to x(k|k-1) in thesis
 * xm_EKF and Pm_EKF: posterior mean and variance, equals to x(k|k) in thesis
 * motorCmds: input of the system, numerical values in between [0, 65535]
 * Q_EKF: model noise variance
 * R_pos_EKF: position measurement noise variance
 * R_IMU_EKF: IMU measurement noise variance: gyro, acc
 * x,y,z will be named as 1,2,3 direction within this file
 * */

/*Prediction Step using motorCmds and system input*/
static void predictionStep_EKF(float timestep, float motorCmds[]);

/*IMU Measurement Update*/
static void updateWithIMU_EKF(float timestep, Axis3f *acc, Axis3f *gyro);

/*Position Measurement Update*/
static void updateWithPosition_EKF(positionMeasurement_t *xyz, float dt);

/*Saves estimate for position control*/
static void stateEstimatorExternalizeState(state_t *state, sensorData_t *sensors, uint32_t tick);



float invSqrt(float x);


/*Functions for reading and saving measurements*/
// Distance-to-point measurements
static xQueueHandle distDataQueue;
#define DIST_QUEUE_LENGTH (10)

static inline bool stateEstimatorHasDistanceMeasurement(distanceMeasurement_t *dist) {
  return (pdTRUE == xQueueReceive(distDataQueue, dist, 0));
}

// Direct measurements of Crazyflie position
static xQueueHandle posDataQueue;
#define POS_QUEUE_LENGTH (10)

static inline bool stateEstimatorHasPositionMeasurement(positionMeasurement_t *pos) {
  return (pdTRUE == xQueueReceive(posDataQueue, pos, 0));
}

// Measurements of a UWB Tx/Rx
static xQueueHandle tdoaDataQueue;
#define UWB_QUEUE_LENGTH (10)

static inline bool stateEstimatorHasTDOAPacket(tdoaMeasurement_t *uwb) {
  return (pdTRUE == xQueueReceive(tdoaDataQueue, uwb, 0));
}

// Measurements of TOF from laser sensor
static xQueueHandle tofDataQueue;
#define TOF_QUEUE_LENGTH (10)

static inline bool stateEstimatorHasTOFPacket(tofMeasurement_t *tof) {
  return (pdTRUE == xQueueReceive(tofDataQueue, tof, 0));
}

/**
 * Constants used in the estimator
 */

#define DEG_TO_RAD (PI/180.0f)
#define RAD_TO_DEG (180.0f/PI)
#define GRAVITY_MAGNITUDE (9.81f) // we use the magnitude such that the sign/direction is explicit in calculations


/**
 * Tuning parameters
 */
#define PREDICT_RATE RATE_100_HZ // Rate of the Prediction Step
// The bounds on the covariance, will be ensured after each covariance update
#define MAX_COVARIANCE (100)
#define MAX_NEGCOVARIANCE (-100)
#define MIN_COVARIANCE (1e-6f)



/*Internal Variables*/
static bool isInit = false;
static bool resetEstimation = true;
static int32_t lastPrediction;
static int32_t lastPosUpdate;
static Axis3f accAccumulator;
static Axis3f gyroAccumulator;
static uint32_t accAccumulatorCount;
static uint32_t gyroAccumulatorCount;
static bool quadIsFlying = false;
static bool failsafeaktiv = false;
bool doneEKFPrediction = false;
static float motorCmds_accumulator[4];
static uint16_t motorCmds_count;
static float motorCmds_current[4];

/*Parameters*/
static float accBias_y = 0.0f; // Defines the bias of the accelerometer in y
static uint16_t posUpdate_states = 6; //Defines how many states are being updated in the position measuement update, difference between EKF(16) and EKFModi(6)

/*Debug variables*/
static float posMeas[3];
static float IMUMeas_debug[6];
static float motor_forces[4];

/*Model Parameters of the crazyflie 2.0*/
static const float mB = 32.64e-3f;
static const float Ixx = 26.3052e-6f;
static const float Iyy = 26.0375e-6f;
static const float Izz = 30.4167e-6f;
static const float kappa = 0.005964552f;
static const float l_arm = 32.5e-3f;

/*Define state as n_state = 16 including additional bias moments (nb1, nb2, nb3)*/
typedef enum
{
  STATE_X_EKF, STATE_Y_EKF, STATE_Z_EKF, STATE_VX_EKF, STATE_VY_EKF, STATE_VZ_EKF, STATE_Q0_EKF,STATE_Q1_EKF,STATE_Q2_EKF,STATE_Q3_EKF, STATE_P_EKF, STATE_Q_EKF, STATE_R_EKF, STATE_NB1_EKF, STATE_NB2_EKF, STATE_NB3_EKF,STATE_DIM_EKF
}stateIdxEKF_t;

/*Define nonlinear process noise on velocity, angular verlocity and bias moments, n_v = 9*/
typedef enum
{
	V_VX, V_VY, V_VZ, V_P, V_Q, V_R, V_NB1, V_NB2, V_NB3, V_DIM
}QIdxEKF_t;

/*Define IMU measurement: GYRO and ACC Output, n_IMU = 6*/
typedef enum
{
	GYRO_X, GYRO_Y, GYRO_Z, ACC_X, ACC_Y, ACC_Z, IMU_DIM
}RIMUIdxEKF_t;

/*Define pos measurement, n_IMU = 3*/
typedef enum
{
	POS_X, POS_Y, POS_Z, POS_DIM
}RPOSIdxEKF_t;

/*Define state and variance for EKF*/
static float xp_EKF[STATE_DIM_EKF]; //prior mean of state estimate
static arm_matrix_instance_f32 xp_EKF_m = { STATE_DIM_EKF, 1, (float *) xp_EKF};
static float xm_EKF[STATE_DIM_EKF]; //posterior mean of state
static arm_matrix_instance_f32 xm_EKF_m = { STATE_DIM_EKF, 1, (float *) xm_EKF};
static float Pp_EKF[STATE_DIM_EKF][STATE_DIM_EKF]; //prior variace of state
static arm_matrix_instance_f32 Pp_EKF_m = { STATE_DIM_EKF, STATE_DIM_EKF, (float *) Pp_EKF};
static float Pm_EKF[STATE_DIM_EKF][STATE_DIM_EKF]; //posterior variance of state
static arm_matrix_instance_f32 Pm_EKF_m = { STATE_DIM_EKF, STATE_DIM_EKF, (float *) Pm_EKF};

/*Define Model and Measurement Noise Variances*/
static float R_IMU_EKF[IMU_DIM][IMU_DIM]; //Measurement noise variance for IMU measurement
static arm_matrix_instance_f32 R_IMU_EKF_m = { IMU_DIM, IMU_DIM, (float *) R_IMU_EKF};
static float R_pos_EKF[POS_DIM][POS_DIM]; //Measurement noise variance for pos measurement
//static arm_matrix_instance_f32 R_pos_EKF_m = { POS_DIM, POS_DIM, (float *) R_pos_EKF};
static float Q_EKF [V_DIM][V_DIM]; //Process noise variance
static arm_matrix_instance_f32 Q_EKF_m = { V_DIM, V_DIM, (float *) Q_EKF};

/*Define Linearized matrices for EKF and identity for EKF algorithm*/
static float identity_state[STATE_DIM_EKF][STATE_DIM_EKF]; //eye matrix with n_state x n_state dimension
static arm_matrix_instance_f32 identity_state_m = {STATE_DIM_EKF, STATE_DIM_EKF, (float *) identity_state};
static float A_EKF[STATE_DIM_EKF][STATE_DIM_EKF]; //Linearized state matrix
static arm_matrix_instance_f32 A_EKF_m = { STATE_DIM_EKF, STATE_DIM_EKF, (float *) A_EKF};
static float L_EKF[STATE_DIM_EKF][V_DIM]; //Linearized process noise matrix
static arm_matrix_instance_f32 L_EKF_m = { STATE_DIM_EKF, V_DIM, (float *) L_EKF};
static float H_IMU[IMU_DIM][STATE_DIM_EKF]; //Linearized IMU measurement equation
static arm_matrix_instance_f32 H_IMU_m = { IMU_DIM, STATE_DIM_EKF, (float *) H_IMU};
static float H_pos[POS_DIM][STATE_DIM_EKF]; //Linearized pos measurement equation
static arm_matrix_instance_f32 H_pos_m = { POS_DIM, STATE_DIM_EKF, (float *) H_pos};

/*Parametrize Model Noise Variance: displayed are variances*/
static float q_process_vx = 5e-7f;
static float q_process_vy = 5e-7f;
static float q_process_vz = 1e-4f;
static float q_process_p = 3e-2f;
static float q_process_q = 3e-2f;
static float q_process_r = 1e-4f;
static float q_process_nb1 = 1e-6f;
static float q_process_nb2 = 1e-6f;
static float q_process_nb3 = 1e-8f;

/*Parametrize Measurement Noise Variance: displayed are standard derivations (will be squared in stateEstimatorInit) */
static float r_meas_posx = 0.0019f;
static float r_meas_posy = 0.0001f;
static float r_meas_posz = 0.0008f;
static float r_meas_gyrox = DEG_TO_RAD*1.1424f;
static float r_meas_gyroy = DEG_TO_RAD*1.5955f;
static float r_meas_gyroz = DEG_TO_RAD*1.2572f;
static float r_meas_accx = GRAVITY_MAGNITUDE*0.0283f;
static float r_meas_accy = GRAVITY_MAGNITUDE*0.0362f;
static float r_meas_accz = GRAVITY_MAGNITUDE*0.0208f;



/**
 * Supporting and utility functions for matrix manipulations using arm_math
 */
static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst)); }
static inline void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst)); }
static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }
static inline void mat_add(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_add_f32(pSrcA, pSrcB, pDst)); }
static inline void mat_sub(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_sub_f32(pSrcA, pSrcB, pDst)); }
//static inline float arm_sqrt(float32_t in)
//{ float pOut = 0; arm_status result = arm_sqrt_f32(in, &pOut); configASSERT(ARM_MATH_SUCCESS == result); return pOut; }

/*Helper Functions*/
float invSqrt(float x)
{
  /*Calculates inv Square root*/
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

static void cleanVariance(float dirtyMatrix[STATE_DIM_EKF][STATE_DIM_EKF]){
	/*Cleans variance: symmetry + boundness: assumes that dirtyMatrix has STATE_DIM_EKF dimension*/
	float tol = 1e-10f;
	for (int i = 0; i<STATE_DIM_EKF;i++){
		if (dirtyMatrix[i][i]<tol){
			//diagonal entries can't be negative
			dirtyMatrix[i][i] = 0;
		}
		for (int j = i+1; j<STATE_DIM_EKF; j++){
			float p = 0.5f*dirtyMatrix[i][j] + 0.5f*dirtyMatrix[j][i];
			if (p>0 && p<tol){
				dirtyMatrix[i][j] = dirtyMatrix[j][i] = 0;
			}else if (p<0 && p>tol){
				dirtyMatrix[i][j] = dirtyMatrix[j][i] = 0;
			}else {
				dirtyMatrix[i][j] = dirtyMatrix[j][i] = p;
			}
		}
	}
}
static void normalizeQuaternions(float *state){
	/*Ensures unity norm for quaternions*/
	float q0 = state[STATE_Q0_EKF];
	float q1 = state[STATE_Q1_EKF];
	float q2 = state[STATE_Q2_EKF];
	float q3 = state[STATE_Q3_EKF];

	float recipNorm = invSqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	state[STATE_Q0_EKF] =q0* recipNorm;
	state[STATE_Q1_EKF]=q1* recipNorm;
	state[STATE_Q2_EKF] =q2* recipNorm;
	state[STATE_Q3_EKF] =q3* recipNorm;
}

/*Main EKF Function*/
static void predictionStep_EKF(float timestep, float motorCmds[]){
	/*Prediction Step of EKF: calculate xp(k+1) and Pp(k+1) from xm(k) and Pm(k)*/
	float s1 = xm_EKF[STATE_X_EKF];
	float s2 = xm_EKF[STATE_Y_EKF];
	float s3 = xm_EKF[STATE_Z_EKF];
	float v1 = xm_EKF[STATE_VX_EKF];
	float v2 = xm_EKF[STATE_VY_EKF];
	float v3 = xm_EKF[STATE_VZ_EKF];
	float q0 = xm_EKF[STATE_Q0_EKF];
	float q1 = xm_EKF[STATE_Q1_EKF];
	float q2 = xm_EKF[STATE_Q2_EKF];
	float q3 = xm_EKF[STATE_Q3_EKF];
	float p = xm_EKF[STATE_P_EKF];
	float q = xm_EKF[STATE_Q_EKF];
	float r = xm_EKF[STATE_R_EKF];
	float nb1 = xm_EKF[STATE_NB1_EKF];
	float nb2 = xm_EKF[STATE_NB2_EKF];
	float nb3 = xm_EKF[STATE_NB3_EKF];

	//Calculate motor forces [N] from PWM commands
	float cp1  = (float) (2.13e-11f*motorCmds[0]*motorCmds[0]+1.03e-6f*motorCmds[0]+5.48e-4f);
	float cp2  = (float) (2.13e-11f*motorCmds[1]*motorCmds[1]+1.03e-6f*motorCmds[1]+5.48e-4f);
	float cp3  = (float) (2.13e-11f*motorCmds[2]*motorCmds[2]+1.03e-6f*motorCmds[2]+5.48e-4f);
	float cp4  = (float) (2.13e-11f*motorCmds[3]*motorCmds[3]+1.03e-6f*motorCmds[3]+5.48e-4f);

	/*For debugging*/
	motorCmds_current[0] = motorCmds[0];
	motorCmds_current[1] = motorCmds[1];
	motorCmds_current[2] = motorCmds[2];
	motorCmds_current[3] = motorCmds[3];

	motor_forces[0] = cp1;
	motor_forces[1] = cp2;
	motor_forces[2] = cp3;
	motor_forces[3] = cp4;

	//Calculate force moments using mixer equation
	float f_tot = cp1 + cp2 + cp3 +cp4; //f_tot
	float n1 = -l_arm*cp1-l_arm*cp2+l_arm*cp3+l_arm*cp4; //n1
	float n2 = -l_arm*cp1+l_arm*cp2+l_arm*cp3-l_arm*cp4; //n2
	float n3 = -kappa*cp1+kappa*cp2-kappa*cp3+kappa*cp4; //n3


	// calculate state prediction using system dynamics: save in xp_EKF
	float thrust_norm = f_tot/mB;

	if (motorCmds[0] <= 1 && motorCmds[1] <= 1 && motorCmds[2] <= 1 && motorCmds[3] <= 1){
		/*crazyflie is still on the ground: no prediction step, just assume that the state will be the same as before*/
		xp_EKF[STATE_X_EKF] = s1;
		xp_EKF[STATE_Y_EKF] = s2;
		xp_EKF[STATE_Z_EKF] = s3;
		xp_EKF[STATE_VX_EKF] = v1;
		xp_EKF[STATE_VY_EKF] = v2;
		xp_EKF[STATE_VZ_EKF] = v3;
		xp_EKF[STATE_Q0_EKF] = q0;
		xp_EKF[STATE_Q1_EKF] = q1;
		xp_EKF[STATE_Q2_EKF] = q2;
		xp_EKF[STATE_Q3_EKF] = q3;
		xp_EKF[STATE_P_EKF] = p;
		xp_EKF[STATE_Q_EKF] = q;
		xp_EKF[STATE_R_EKF] = r;
		xp_EKF[STATE_NB1_EKF] = nb1;
		xp_EKF[STATE_NB2_EKF] = nb2;
		xp_EKF[STATE_NB3_EKF] = nb3;
		/*Pp is not being updated and set to Pm*/
		for (int i = 0; i<STATE_DIM_EKF; i++){
			for (int j = 0; j<STATE_DIM_EKF; j++){
				Pp_EKF[i][j] = Pm_EKF[i][j];
			}
		}
	}
	else {
		/*Quadcopter is flying*/
		//Calculate xp(k+1) through nonlinear system dynamics
		xp_EKF[STATE_X_EKF] = s1+timestep*v1;
		xp_EKF[STATE_Y_EKF] = s2+timestep*v2;
		xp_EKF[STATE_Z_EKF] = s3+timestep*v3;
		xp_EKF[STATE_VX_EKF] = v1+timestep*(2*(q1*q3+q0*q2)*thrust_norm);
		xp_EKF[STATE_VY_EKF] = v2+timestep*(2*(q2*q3-q0*q1)*thrust_norm);
		xp_EKF[STATE_VZ_EKF] = v3+timestep*((q0*q0-q1*q1-q2*q2+q3*q3)*thrust_norm-GRAVITY_MAGNITUDE);
		xp_EKF[STATE_Q0_EKF] = q0+timestep*0.5f*(-p*q1-q*q2-r*q3);
		xp_EKF[STATE_Q1_EKF] = q1+timestep*0.5f*(p*q0+r*q2-q*q3);
		xp_EKF[STATE_Q2_EKF] = q2+timestep*0.5f*(q*q0-r*q1+p*q3);
		xp_EKF[STATE_Q3_EKF] = q3+timestep*0.5f*(r*q0+q*q1-p*q2);
		xp_EKF[STATE_P_EKF] = p+timestep*(n1-nb1-(Izz-Iyy)*q*r)/Ixx;
		xp_EKF[STATE_Q_EKF] = q+timestep*(n2-nb2-(Ixx-Izz)*r*p)/Iyy;
		xp_EKF[STATE_R_EKF] = r+timestep*(n3-nb3-(Iyy-Ixx)*p*q)/Izz;
		xp_EKF[STATE_NB1_EKF] = nb1;
		xp_EKF[STATE_NB2_EKF] = nb2;
		xp_EKF[STATE_NB3_EKF] = nb3;

		//Calculate variance prediction Pp(k+1)
		//1. build A Matrix:
		A_EKF[STATE_X_EKF][STATE_X_EKF] =1;
		A_EKF[STATE_Y_EKF][STATE_Y_EKF] = 1;
		A_EKF[STATE_Z_EKF][STATE_Z_EKF] = 1;
		A_EKF[STATE_VX_EKF][STATE_VX_EKF] = 1;
		A_EKF[STATE_VY_EKF][STATE_VY_EKF]  = 1;
		A_EKF[STATE_VZ_EKF][STATE_VZ_EKF] = 1;
		A_EKF[STATE_Q0_EKF][STATE_Q0_EKF] = 1;
		A_EKF[STATE_Q1_EKF][STATE_Q1_EKF] = 1;
		A_EKF[STATE_Q2_EKF][STATE_Q2_EKF] = 1;
		A_EKF[STATE_Q3_EKF][STATE_Q3_EKF] = 1;
		A_EKF[STATE_P_EKF][STATE_P_EKF] = 1;
		A_EKF[STATE_Q_EKF][STATE_Q_EKF] = 1;
		A_EKF[STATE_R_EKF][STATE_R_EKF] = 1;
		A_EKF[STATE_NB1_EKF] [STATE_NB1_EKF]= 1;
		A_EKF[STATE_NB2_EKF][STATE_NB2_EKF] = 1;
		A_EKF[STATE_NB3_EKF][STATE_NB3_EKF]= 1;

		A_EKF[STATE_X_EKF][STATE_VX_EKF] = timestep;
		A_EKF[STATE_Y_EKF][STATE_VY_EKF] = timestep;
		A_EKF[STATE_Z_EKF][STATE_VZ_EKF] = timestep;

		A_EKF[STATE_VX_EKF][STATE_Q0_EKF] = timestep*2*q2*thrust_norm;
		A_EKF[STATE_VX_EKF][STATE_Q1_EKF] = timestep*2*q3*thrust_norm;
		A_EKF[STATE_VX_EKF][STATE_Q2_EKF] = timestep*2*q0*thrust_norm;
		A_EKF[STATE_VX_EKF][STATE_Q3_EKF] = timestep*2*q1*thrust_norm;
		A_EKF[STATE_VY_EKF][STATE_Q0_EKF] = -timestep*2*q1*thrust_norm;
		A_EKF[STATE_VY_EKF][STATE_Q1_EKF] = -timestep*2*q0*thrust_norm;
		A_EKF[STATE_VY_EKF][STATE_Q2_EKF] = timestep*2*q3*thrust_norm;
		A_EKF[STATE_VY_EKF][STATE_Q3_EKF] = timestep*2*q2*thrust_norm;
		A_EKF[STATE_VZ_EKF][STATE_Q0_EKF] = timestep*2*q0*thrust_norm;
		A_EKF[STATE_VZ_EKF][STATE_Q1_EKF] = -timestep*2*q1*thrust_norm;
		A_EKF[STATE_VZ_EKF][STATE_Q2_EKF] = -timestep*2*q2*thrust_norm;
		A_EKF[STATE_VZ_EKF][STATE_Q3_EKF] = timestep*2*q3*thrust_norm;

		A_EKF[STATE_Q0_EKF] [STATE_Q1_EKF] = -timestep*0.5f*p;
		A_EKF[STATE_Q0_EKF] [STATE_Q2_EKF] = -timestep*0.5f*q;
		A_EKF[STATE_Q0_EKF] [STATE_Q3_EKF] = -timestep*0.5f*r;
		A_EKF[STATE_Q0_EKF] [STATE_P_EKF] = -timestep*0.5f*q1;
		A_EKF[STATE_Q0_EKF] [STATE_Q_EKF] = -timestep*0.5f*q2;
		A_EKF[STATE_Q0_EKF] [STATE_R_EKF] = -timestep*0.5f*q3;
		A_EKF[STATE_Q1_EKF] [STATE_Q0_EKF] = timestep*0.5f*p;
		A_EKF[STATE_Q1_EKF] [STATE_Q2_EKF] = timestep*0.5f*r;
		A_EKF[STATE_Q1_EKF] [STATE_Q3_EKF] = -timestep*0.5f*q;
		A_EKF[STATE_Q1_EKF] [STATE_P_EKF] = timestep*0.5f*q0;
		A_EKF[STATE_Q1_EKF] [STATE_Q_EKF] = -timestep*0.5f*q3;
		A_EKF[STATE_Q1_EKF] [STATE_R_EKF] = timestep*0.5f*q2;
		A_EKF[STATE_Q2_EKF] [STATE_Q0_EKF] = timestep*0.5f*q;
		A_EKF[STATE_Q2_EKF] [STATE_Q1_EKF] = -timestep*0.5f*r;
		A_EKF[STATE_Q2_EKF] [STATE_Q3_EKF] = timestep*0.5f*p;
		A_EKF[STATE_Q2_EKF] [STATE_P_EKF] = timestep*0.5f*q3;
		A_EKF[STATE_Q2_EKF] [STATE_Q_EKF] = timestep*0.5f*q0;
		A_EKF[STATE_Q2_EKF] [STATE_R_EKF] = -timestep*0.5f*q1;
		A_EKF[STATE_Q3_EKF] [STATE_Q0_EKF] = timestep*0.5f*r;
		A_EKF[STATE_Q3_EKF] [STATE_Q1_EKF] = timestep*0.5f*q;
		A_EKF[STATE_Q3_EKF] [STATE_Q2_EKF] = -timestep*0.5f*p;
		A_EKF[STATE_Q3_EKF] [STATE_P_EKF] = -timestep*0.5f*q2;
		A_EKF[STATE_Q3_EKF] [STATE_Q_EKF] = timestep*0.5f*q1;
		A_EKF[STATE_Q3_EKF] [STATE_R_EKF] = timestep*0.5f*q0;

		A_EKF[STATE_P_EKF][STATE_Q_EKF] = timestep*r*(Iyy-Izz)/Ixx;
		A_EKF[STATE_P_EKF][STATE_R_EKF] = timestep*q*(Iyy-Izz)/Ixx;
		A_EKF[STATE_P_EKF][STATE_NB1_EKF] = -timestep/Ixx;
		A_EKF[STATE_Q_EKF][STATE_P_EKF] = timestep*r*(Izz-Ixx)/Iyy;
		A_EKF[STATE_Q_EKF][STATE_R_EKF] = timestep*p*(Izz-Ixx)/Iyy;
		A_EKF[STATE_Q_EKF][STATE_NB2_EKF] = -timestep/Iyy;
		A_EKF[STATE_R_EKF][STATE_P_EKF] = timestep*q*(Ixx-Iyy)/Izz;
		A_EKF[STATE_R_EKF][STATE_Q_EKF] = timestep*p*(Ixx-Iyy)/Izz;
		A_EKF[STATE_R_EKF][STATE_NB3_EKF] = -timestep/Izz;

		//2. build L matrix
		L_EKF[STATE_VX_EKF][V_VX] = 1;
		L_EKF[STATE_VY_EKF][V_VY] = 1;
		L_EKF[STATE_VZ_EKF][V_VZ] = 1;
		L_EKF[STATE_P_EKF][V_P] = 1;
		L_EKF[STATE_Q_EKF][V_Q] = 1;
		L_EKF[STATE_R_EKF][V_R] = 1;
		L_EKF[STATE_NB1_EKF][V_NB1] = 1;
		L_EKF[STATE_NB2_EKF][V_NB2] = 1;
		L_EKF[STATE_NB3_EKF][V_NB3] = 1;

		//3. calculate Pp matrix
		// Define space for saving Zwischenergebnis
		static float tmp1 [STATE_DIM_EKF][STATE_DIM_EKF]; //A*Pm*A'
		static arm_matrix_instance_f32 tmp1_m = { STATE_DIM_EKF, STATE_DIM_EKF, (float *) tmp1};
		static float tmp2 [STATE_DIM_EKF][STATE_DIM_EKF]; //L*Q*L'
		static arm_matrix_instance_f32 tmp2_m = { STATE_DIM_EKF, STATE_DIM_EKF, (float *) tmp2};
		static float tmp3 [V_DIM][STATE_DIM_EKF]; //L'
		static arm_matrix_instance_f32 tmp3_m = {V_DIM, STATE_DIM_EKF, (float *) tmp3};

		static float tmp4 [STATE_DIM_EKF][STATE_DIM_EKF]; //L*Q*L'
		static arm_matrix_instance_f32 tmp4_m = { STATE_DIM_EKF, STATE_DIM_EKF, (float *) tmp4};
		static float tmp5 [V_DIM][STATE_DIM_EKF]; //L'
		static arm_matrix_instance_f32 tmp5_m = {V_DIM, STATE_DIM_EKF, (float *) tmp5};

		mat_trans(&A_EKF_m, &tmp1_m); //tmp1 = A'
		mat_mult(&Pm_EKF_m, &tmp1_m, &tmp4_m); //tmp4 = Pm*A'
		mat_mult(&A_EKF_m, &tmp4_m, &tmp1_m); //tmp1 = A*Pm*A';
		mat_trans(&L_EKF_m, &tmp3_m); //tmp3 = L'
		mat_mult(&Q_EKF_m, &tmp3_m, &tmp5_m); //tmp5 = Q*L'
		mat_mult(&L_EKF_m, &tmp5_m, &tmp2_m); //tmp2 = L*Q*L'
		mat_add(&tmp1_m, &tmp2_m, &Pp_EKF_m); //Pp = A*Pm*A'+L*Q*L'
		}

	/*Clean result xp_EKF and Pp_EKF*/
	normalizeQuaternions(xp_EKF);
	cleanVariance(Pp_EKF);
}

static void updateWithIMU_EKF(float timestep, Axis3f *acc, Axis3f *gyro){
	/*Performs Measurement update with IMU measurements from gyro and acc Output*/
	float q0 = xp_EKF[STATE_Q0_EKF];
	float q1 = xp_EKF[STATE_Q1_EKF];
	float q2 = xp_EKF[STATE_Q2_EKF];
	float q3 = xp_EKF[STATE_Q3_EKF];
	float p = xp_EKF[STATE_P_EKF];
	float q = xp_EKF[STATE_Q_EKF];
	float r = xp_EKF[STATE_R_EKF];

	//1.) build H_IMU matrix:
	H_IMU[GYRO_X][STATE_P_EKF] = 1; //gyro Output already converted to rad/s
    H_IMU[GYRO_Y][STATE_Q_EKF] = 1;
    H_IMU[GYRO_Z][STATE_R_EKF] = 1;

	H_IMU[ACC_X][STATE_Q0_EKF] = -GRAVITY_MAGNITUDE*2*q2; //acc Output in m/s²
	H_IMU[ACC_X][STATE_Q1_EKF] = GRAVITY_MAGNITUDE*2*q3;
	H_IMU[ACC_X][STATE_Q2_EKF] = -GRAVITY_MAGNITUDE*2*q0;
	H_IMU[ACC_X][STATE_Q3_EKF] = GRAVITY_MAGNITUDE*2*q1;

	H_IMU[ACC_Y][STATE_Q0_EKF] = GRAVITY_MAGNITUDE*2*q1;
	H_IMU[ACC_Y][STATE_Q1_EKF] = GRAVITY_MAGNITUDE*2*q0;
	H_IMU[ACC_Y][STATE_Q2_EKF] = GRAVITY_MAGNITUDE*2*q3;
	H_IMU[ACC_Y][STATE_Q3_EKF] = GRAVITY_MAGNITUDE*2*q2;

	H_IMU[ACC_Z][STATE_Q0_EKF] = GRAVITY_MAGNITUDE*2*q0;
	H_IMU[ACC_Z][STATE_Q1_EKF] = -GRAVITY_MAGNITUDE*2*q1;
	H_IMU[ACC_Z][STATE_Q2_EKF] = -GRAVITY_MAGNITUDE*2*q2;
	H_IMU[ACC_Z][STATE_Q3_EKF] = GRAVITY_MAGNITUDE*2*q3;

	//2.) calculate gain matrix K = PpH'(HPpH'+R)⁻1: M is identity matrix (linear measurement noise)
	static float K_IMU [STATE_DIM_EKF][IMU_DIM]; //Kalman Filter Gain
	static arm_matrix_instance_f32 K_IMU_m = { STATE_DIM_EKF, IMU_DIM, (float *) K_IMU};
	static float tmp1 [IMU_DIM][IMU_DIM]; //for HPpH'+R
	static arm_matrix_instance_f32 tmp1_m = { IMU_DIM, IMU_DIM, (float *) tmp1};
	static float tmp1_inv [IMU_DIM][IMU_DIM]; //for HPpH'+R
	static arm_matrix_instance_f32 tmp1_inv_m = { IMU_DIM, IMU_DIM, (float *) tmp1_inv};
	static float tmp2 [STATE_DIM_EKF][IMU_DIM]; //for PpH'
	static arm_matrix_instance_f32 tmp2_m = { STATE_DIM_EKF, IMU_DIM, (float *) tmp2};

	static float tmp2b [STATE_DIM_EKF][IMU_DIM]; //for PpH'
	static arm_matrix_instance_f32 tmp2b_m = { STATE_DIM_EKF, IMU_DIM, (float *) tmp2b};
	static float tmp1b [IMU_DIM][IMU_DIM]; //for HPpH'+R
	static arm_matrix_instance_f32 tmp1b_m = { IMU_DIM, IMU_DIM, (float *) tmp1b};

	mat_trans(&H_IMU_m, &tmp2_m); //tmp2 = H'
	mat_mult(&Pp_EKF_m, &tmp2_m, &tmp2b_m); //tmp2b = Pp*H'
	mat_mult(&H_IMU_m, &tmp2b_m, &tmp1_m); //tmp1 = H*Pp*H';
	mat_add(&tmp1_m, &R_IMU_EKF_m, &tmp1b_m); //tmp1b = H*Pp*H'+R
	mat_inv(&tmp1b_m, &tmp1_inv_m); //tmp1_inv = (H*Pp*H'+R)⁻1
	mat_mult(&tmp2b_m, &tmp1_inv_m, &K_IMU_m); //K = Pp*H'*(H*Pp*H'+R)⁻1

	//3.) define innovation: z-h(xp)
	static float innovation_IMU[IMU_DIM];
	static arm_matrix_instance_f32 innovation_IMU_m = { IMU_DIM, 1, (float *) innovation_IMU};

	innovation_IMU[GYRO_X] = gyro->x-p; //gyro already in rad/s
	innovation_IMU[GYRO_Y] = gyro->y-q;
	innovation_IMU[GYRO_Z] = gyro->z-r;

	innovation_IMU[ACC_X] = acc->x - GRAVITY_MAGNITUDE*2*(q1*q3-q0*q2); //acc in m/s²
	innovation_IMU[ACC_Y] = acc->y - GRAVITY_MAGNITUDE*2*(q2*q3+q0*q1);
	innovation_IMU[ACC_Z] = acc->z-GRAVITY_MAGNITUDE*(q0*q0-q1*q1-q2*q2+q3*q3);

	//4.) calculate posterior mean xm = xp + K*innovation
	static float tmp3 [STATE_DIM_EKF]; //K*innovation
	static arm_matrix_instance_f32 tmp3_m = { STATE_DIM_EKF, 1, (float *) tmp3};
	mat_mult(&K_IMU_m, &innovation_IMU_m, &tmp3_m); //tmp3 = K*innovation
	mat_add(&xp_EKF_m, &tmp3_m, &xm_EKF_m); //xm = xp + K*innovation

	normalizeQuaternions(xm_EKF);

	//5.) calculate posterior variance Pm = (I-K*H)*Pp
	static float tmp4[STATE_DIM_EKF][STATE_DIM_EKF];
	static arm_matrix_instance_f32 tmp4_m = { STATE_DIM_EKF, STATE_DIM_EKF, (float *) tmp4};
	static float tmp4b[STATE_DIM_EKF][STATE_DIM_EKF];
	static arm_matrix_instance_f32 tmp4b_m = { STATE_DIM_EKF, STATE_DIM_EKF, (float *) tmp4b};

	mat_mult(&K_IMU_m, &H_IMU_m, &tmp4_m); //tmp4 = K*H
	mat_sub(&identity_state_m, &tmp4_m,&tmp4b_m); //tmp4b = (I-K*H)
	mat_mult(&tmp4b_m, &Pp_EKF_m, &Pm_EKF_m); //Pm = (I-K*H)*Pp

	cleanVariance(Pm_EKF);
}

static void updateWithPosition_EKF(positionMeasurement_t *xyz, float dt){
	/*Updates position with last measurement update estimate: xm, Pm*/

	float s1 = xm_EKF[STATE_X_EKF];
	float s2 = xm_EKF[STATE_Y_EKF];
	float s3 = xm_EKF[STATE_Z_EKF];

	//1.) build H_IMU matrix:
	H_pos[POS_X][STATE_X_EKF] = 1;
	H_pos[POS_Y][STATE_Y_EKF] = 1;
	H_pos[POS_Z][STATE_Z_EKF] = 1;

	//2.) calculate gain matrix K = PpH'(HPpH'+R)⁻1: M is identity matrix (linear measurement noise)
	static float K_pos [STATE_DIM_EKF][POS_DIM]; //Kalman Filter Gain
	static arm_matrix_instance_f32 K_pos_m = { STATE_DIM_EKF, POS_DIM, (float *) K_pos};
	static float tmp1 [POS_DIM][POS_DIM]; //for HPmH'+R
	static arm_matrix_instance_f32 tmp1_m = { POS_DIM, POS_DIM, (float *) tmp1};
	static float tmp1_inv [POS_DIM][POS_DIM]; //for HPmH'+R
	static arm_matrix_instance_f32 tmp1_inv_m = { POS_DIM, POS_DIM, (float *) tmp1_inv};
	static float tmp2 [STATE_DIM_EKF][POS_DIM]; //for Pm*H'
	static arm_matrix_instance_f32 tmp2_m = { STATE_DIM_EKF, POS_DIM, (float *) tmp2};
	for (int i = 0; i<POS_DIM; i++){
		for (int j = 0; j<POS_DIM; j++){
			tmp1[i][j] = Pm_EKF[i][j]+R_pos_EKF[i][j];
		}
	}
	for (int i = 0; i<STATE_DIM_EKF; i++){
		for (int j = 0; j<POS_DIM; j++){
			tmp2[i][j] = Pm_EKF[i][j];
		}
	}
	mat_inv(&tmp1_m, &tmp1_inv_m); //tmp1_inv = (H*Pm*H'+R)⁻1
	mat_mult(&tmp2_m, &tmp1_inv_m, &K_pos_m); //K = Pm*H'*(H*Pm*H'+R)⁻1

	//3.) define innovation: z-h(xp)
	static float innovation_pos[POS_DIM];
	static arm_matrix_instance_f32 innovation_pos_m = { POS_DIM, 1, (float *) innovation_pos};
	innovation_pos[POS_X] = xyz->pos[0]-s1;
	innovation_pos[POS_Y] = xyz->pos[1]-s2;
	innovation_pos[POS_Z] = xyz->pos[2]-s3;

	//For Debugging:
	posMeas[0] = xyz->pos[0];
	posMeas[1] = xyz->pos[1];
	posMeas[2] = xyz->pos[2];

	//4.) calculate posterior mean xm = xm + K*innovation
	static float tmp3 [STATE_DIM_EKF]; //K*innovation
	static arm_matrix_instance_f32 tmp3_m = { STATE_DIM_EKF, 1, (float *) tmp3};

	mat_mult(&K_pos_m, &innovation_pos_m, &tmp3_m); //tmp3 = K*innovation
	for (int i = 0; i<posUpdate_states; i++){
		xm_EKF[i] = xm_EKF[i]+tmp3[i];
	}
	normalizeQuaternions(xm_EKF);

	//5.) calculate posterior variance Pm = (I-K*H)*Pp
	static float tmp4[STATE_DIM_EKF][STATE_DIM_EKF];
	static arm_matrix_instance_f32 tmp4_m = { STATE_DIM_EKF, STATE_DIM_EKF, (float *) tmp4};
	static float tmp4b[STATE_DIM_EKF][STATE_DIM_EKF];
	static arm_matrix_instance_f32 tmp4b_m = { STATE_DIM_EKF, STATE_DIM_EKF, (float *) tmp4b};

	mat_mult(&K_pos_m, &H_pos_m, &tmp4_m); //tmp4 = K*H
	mat_sub(&identity_state_m, &tmp4_m,&tmp4b_m); //tmp4b = (I-K*H)
	//TODO: Pm_EKF will be saved in itself

	for (int i = 0;i<STATE_DIM_EKF; i++){
		Pp_EKF[i][i] = Pm_EKF[i][i];
		for (int j = i+1; j<STATE_DIM_EKF; j++){
			Pp_EKF[i][j] = Pm_EKF[i][j];
			Pp_EKF[j][i] = Pm_EKF[i][j];
		}
	}

	mat_mult(&tmp4b_m, &Pp_EKF_m, &Pm_EKF_m); //Pm = (I-K*H)*Pm

	cleanVariance(Pm_EKF);
}



// --------------------------------------------------


/*This is the main function, which will be called by stabilizer.c*/
void stateEstimatorUpdate(state_t *state, sensorData_t *sensors, control_t *control, motorCmds_t *motorCmds)
{
  // If the client (via a parameter update) triggers an estimator reset:
  if (resetEstimation) { stateEstimatorInit(); resetEstimation = false; }

  // Tracks whether a prediction step and IMU Measurement Update has been made, necessary for writing the gyro measurements to the angular velocity
  doneEKFPrediction = false;

  uint32_t tick = xTaskGetTickCount(); // would be nice if this had a precision higher than 1ms...

  // Average the last IMU measurements and Motor Cmds: add the values to an accumulator first
  if (sensorsReadAcc(&sensors->acc)) {
    accAccumulator.x += GRAVITY_MAGNITUDE*(sensors->acc.x); // accelerometer is in Gs
    accAccumulator.y += GRAVITY_MAGNITUDE*(sensors->acc.y)-accBias_y; // but the estimator requires ms^-2
    accAccumulator.z += GRAVITY_MAGNITUDE*(sensors->acc.z);
    accAccumulatorCount++;
  }

  if (sensorsReadGyro(&sensors->gyro)) {
    gyroAccumulator.x += sensors->gyro.x * DEG_TO_RAD; // gyro is in deg/sec
    gyroAccumulator.y += sensors->gyro.y * DEG_TO_RAD; // but the estimator requires rad/sec
    gyroAccumulator.z += sensors->gyro.z * DEG_TO_RAD;
    gyroAccumulatorCount++;
  }

  motorCmds_accumulator[0] += motorCmds->cmd1;
  motorCmds_accumulator[1] += motorCmds->cmd2;
  motorCmds_accumulator[2]+= motorCmds->cmd3;
  motorCmds_accumulator[3] +=motorCmds->cmd4;
  motorCmds_count += 1;


  // At a 100Hz rate: Do Prediction Step and IMU Update
  if ((tick-lastPrediction) >= configTICK_RATE_HZ/PREDICT_RATE
      && gyroAccumulatorCount > 0
      && accAccumulatorCount > 0)
  {

	//Average IMU measurements and motor cmds: devide the accumulators by the count
    gyroAccumulator.x /= gyroAccumulatorCount;
    gyroAccumulator.y /= gyroAccumulatorCount;
    gyroAccumulator.z /= gyroAccumulatorCount;
    accAccumulator.x /= accAccumulatorCount;
    accAccumulator.y /= accAccumulatorCount;
    accAccumulator.z /= accAccumulatorCount;
    motorCmds_accumulator[0] /= motorCmds_count;
    motorCmds_accumulator[1]/= motorCmds_count;
    motorCmds_accumulator[2]/= motorCmds_count;
    motorCmds_accumulator[3] /= motorCmds_count;

    //Calculate time step
    float dt = (float)(tick-lastPrediction)/configTICK_RATE_HZ;

    //Do Prediction Step and IMU Update
    predictionStep_EKF(dt, motorCmds_accumulator); //calculates xp_EKF and Pp_EKF using prediction step: PWM is needed input
    updateWithIMU_EKF(dt, &accAccumulator, &gyroAccumulator); //calculates xm_EKF and Pm_EKF using IMU measurements and xp_EKF/Pp_EKF

    /*Reset accumulators and counts*/
    memset(motorCmds_accumulator, 0, sizeof(motorCmds_accumulator));
    motorCmds_count = 0;
    accAccumulator = (Axis3f){.axis={0}};//drag negligible
    accAccumulatorCount = 0;
    gyroAccumulator = (Axis3f){.axis={0}};
    gyroAccumulatorCount = 0;

    /*Set Counter*/
    doneEKFPrediction = true;
    lastPrediction = tick;

    /*Debug stuff*/
    IMUMeas_debug[0] = gyroAccumulator.x;
    IMUMeas_debug[1] = gyroAccumulator.y;
    IMUMeas_debug[2] = gyroAccumulator.z;
    IMUMeas_debug[3] = accAccumulator.x;
    IMUMeas_debug[4] = accAccumulator.y;
    IMUMeas_debug[5] = accAccumulator.z;
  }


  /*Perform position measurement update, if measurement is available*/
  positionMeasurement_t pos;
  while (stateEstimatorHasPositionMeasurement(&pos))
  {
	float dt_pos = (float)(tick-lastPosUpdate)/configTICK_RATE_HZ;
	pos.timestamp =tick;
    lastPosUpdate=tick;

    updateWithPosition_EKF(&pos, dt_pos);
  }


  // If PosMeasurements are not available for a longer period or the state estimate is out of the safety box, the failsave-mode is activated
  float dt_pos_fallback = (float)(tick-lastPosUpdate)/configTICK_RATE_HZ;
  failsafeaktiv =(dt_pos_fallback>1
		  || state->position.x >3 || state->position.x < 0
		  || state->position.y >1 || state->position.y < -1
		  || state->position.z >2 || state->position.z < 0 );

  /*Inernatl state is externalized for position control*/
  stateEstimatorExternalizeState(state, sensors, tick);

}


static void stateEstimatorExternalizeState(state_t *state, sensorData_t *sensors, uint32_t tick)
{
   //position state is already in world frame
  state->position = (point_t){
      .timestamp = tick,
      .x = xm_EKF[STATE_X_EKF],
      .y = xm_EKF[STATE_Y_EKF],
      .z = xm_EKF[STATE_Z_EKF]
  };

  // velocity is already in world frame
  state->velocity = (velocity_t){
      .timestamp = tick,
	  .x = xm_EKF[STATE_VX_EKF],
	  .y = xm_EKF[STATE_VY_EKF],
	  .z = xm_EKF[STATE_VZ_EKF]
  };

  // convert the new attitude into Euler YPR
  float q0 = xm_EKF[STATE_Q0_EKF];
  float q1 = xm_EKF[STATE_Q1_EKF];
  float q2 = xm_EKF[STATE_Q2_EKF];
  float q3 = xm_EKF[STATE_Q3_EKF];

  float yaw = atan2f(2*(q1*q2+q0*q3) , q0*q0 + q1*q1 - q2*q2 - q3*q3);
  float pitch = asinf(-2*(q1*q3 - q0*q2));
  float roll = atan2f(2*(q2*q3+q0*q1) , q0*q0 - q1*q1 - q2*q2 + q3*q3);

  // Save attitude in deg
  state->attitude = (attitude_t){
      .timestamp = tick,
      .roll = roll*RAD_TO_DEG,
      .pitch = -pitch*RAD_TO_DEG,
      .yaw = yaw*RAD_TO_DEG
  };

  // Save quaternion, hopefully one day this could be used in a better controller.
  state->attitudeQuaternion = (quaternion_t){
      .timestamp = tick,
      .w = q0,
      .x = q1,
      .y = q2,
      .z = q3
  };

  /*Save angular velocity: if no EKF Update has been made, the gyro meas are treated as state estimates*/
  if (doneEKFPrediction) {
	  state->angularVel = (Axis3f){
		  .x  = RAD_TO_DEG*xm_EKF[STATE_P_EKF],
		  .y  = RAD_TO_DEG*xm_EKF[STATE_Q_EKF],
		  .z  = RAD_TO_DEG*xm_EKF[STATE_R_EKF],
	  };
  } else {
	  state->angularVel = (Axis3f){
		  .x  = sensors->gyro.x,
		  .y  = sensors->gyro.y,
		  .z  = sensors->gyro.z,
	  };
  }


  //Save the Failsave Status
  state->failsave = failsafeaktiv;

  // save tiltcomp = R[2][2] for position control in z
  state->tiltcomp = q0*q0-q1*q1-q2*q2+q3*q3;
}


/*At the beginning the state estimator is initialized*/
void stateEstimatorInit(void) {
  if (!isInit)
  {
    distDataQueue = xQueueCreate(DIST_QUEUE_LENGTH, sizeof(distanceMeasurement_t));
    posDataQueue = xQueueCreate(POS_QUEUE_LENGTH, sizeof(positionMeasurement_t));
    tdoaDataQueue = xQueueCreate(UWB_QUEUE_LENGTH, sizeof(tdoaMeasurement_t));
    tofDataQueue = xQueueCreate(TOF_QUEUE_LENGTH, sizeof(tofMeasurement_t));
  }
  else
  {
    xQueueReset(distDataQueue);
    xQueueReset(posDataQueue);
    xQueueReset(tdoaDataQueue);
    xQueueReset(tofDataQueue);
  }

  /*Initialize counter*/
  lastPrediction = xTaskGetTickCount();
  lastPosUpdate = xTaskGetTickCount();

  /*Initialize accumulator an count*/
  accAccumulator = (Axis3f){.axis={0}};
  gyroAccumulator = (Axis3f){.axis={0}};
  accAccumulatorCount = 0;
  gyroAccumulatorCount = 0;
  memset(motorCmds_accumulator, 0, sizeof(motorCmds_accumulator));
  motorCmds_count = 0;

  /*Initialize matrices with 0 first: will be set later*/
  memset(Pm_EKF, 0, sizeof(Pm_EKF));
  memset(xm_EKF, 0, sizeof(xm_EKF));
  memset(Pp_EKF, 0, sizeof(Pp_EKF));
  memset(xp_EKF, 0, sizeof(xp_EKF));
  memset(R_IMU_EKF, 0, sizeof(R_IMU_EKF));
  memset(R_pos_EKF, 0, sizeof(R_pos_EKF));
  memset(Q_EKF, 0, sizeof(Q_EKF));
  memset(A_EKF, 0, sizeof(A_EKF));
  memset(L_EKF, 0, sizeof(L_EKF));
  memset(H_IMU, 0, sizeof(H_IMU));
  memset(H_pos, 0, sizeof(H_pos));

  /*Initial state estimate with x0*/
  xm_EKF[STATE_X_EKF] = 0.92f;
  xm_EKF[STATE_Y_EKF] = -0.08f;
  xm_EKF[STATE_Z_EKF] = 0.45f;
  xm_EKF[STATE_VX_EKF] = 0;
  xm_EKF[STATE_VY_EKF] = 0;
  xm_EKF[STATE_VZ_EKF] = 0;
  xm_EKF[STATE_Q0_EKF] = 1;
  xm_EKF[STATE_Q1_EKF] = 0;
  xm_EKF[STATE_Q2_EKF] = 0;
  xm_EKF[STATE_Q3_EKF] = 0;
  xm_EKF[STATE_P_EKF] = 0;
  xm_EKF[STATE_Q_EKF] = 0;
  xm_EKF[STATE_R_EKF] = 0;
  xm_EKF[STATE_NB1_EKF] = 0;
  xm_EKF[STATE_NB2_EKF] = 0;
  xm_EKF[STATE_NB3_EKF] = 0;

  //Initialize Pm_EKF with P0:
  Pm_EKF[STATE_X_EKF][STATE_X_EKF] = 0.01f;
  Pm_EKF[STATE_Y_EKF][STATE_Y_EKF] = 0.01f;
  Pm_EKF[STATE_Z_EKF][STATE_Z_EKF] = 0.01f;
  Pm_EKF[STATE_VX_EKF][STATE_VX_EKF] = 0.000001f;
  Pm_EKF[STATE_VY_EKF][STATE_VY_EKF]  = 0.000001f;
  Pm_EKF[STATE_VZ_EKF][STATE_VZ_EKF] = 0.000001f;
  Pm_EKF[STATE_Q0_EKF][STATE_Q0_EKF] = 0.0001f;
  Pm_EKF[STATE_Q1_EKF][STATE_Q1_EKF] = 0.0001f;
  Pm_EKF[STATE_Q2_EKF][STATE_Q2_EKF] = 0.0001f;
  Pm_EKF[STATE_Q3_EKF][STATE_Q3_EKF] = 0.0001f;
  Pm_EKF[STATE_P_EKF][STATE_P_EKF] = 0.000001f;
  Pm_EKF[STATE_Q_EKF][STATE_Q_EKF] = 0.000001f;
  Pm_EKF[STATE_R_EKF][STATE_R_EKF] = 0.000001f;
  Pm_EKF[STATE_NB1_EKF] [STATE_NB1_EKF]= 1e-6f;
  Pm_EKF[STATE_NB2_EKF][STATE_NB2_EKF] = 1e-6f;
  Pm_EKF[STATE_NB3_EKF][STATE_NB3_EKF]= 1e-8f;

  /*Initialize prior state and variance to the same as the posterior*/
  for (int i = 0; i<STATE_DIM_EKF; i++){
	  xp_EKF[i] = xm_EKF[i];
	  Pp_EKF[i][i] = Pm_EKF[i][i];
  }

  /*Set Process noise matrix Q_EKF*/
  Q_EKF[V_VX][V_VX] = q_process_vx;
  Q_EKF[V_VY][V_VY] = q_process_vy;
  Q_EKF[V_VZ][V_VZ] = q_process_vz;
  Q_EKF[V_P][V_P] = q_process_p;
  Q_EKF[V_Q][V_Q] = q_process_q;
  Q_EKF[V_R][V_R] = q_process_r;
  Q_EKF[V_NB1][V_NB1] = q_process_nb1;
  Q_EKF[V_NB2][V_NB2] = q_process_nb2;
  Q_EKF[V_NB3][V_NB3] = q_process_nb3;

  /*Set Measurement noise matrix R_pos and R_IMU: parameters will be squared here*/
  R_pos_EKF [POS_X][POS_X] = powf(r_meas_posx, 2);
  R_pos_EKF [POS_Y][POS_Y] = powf(r_meas_posy, 2);
  R_pos_EKF [POS_Z][POS_Z] = powf(r_meas_posz, 2);
  R_IMU_EKF[GYRO_X][GYRO_X] = powf(r_meas_gyrox, 2);
  R_IMU_EKF[GYRO_Y][GYRO_Y] = powf(r_meas_gyroy, 2);
  R_IMU_EKF[GYRO_Z][GYRO_Z] = powf(r_meas_gyroz, 2);
  R_IMU_EKF[ACC_X][ACC_X] = powf(r_meas_accx, 2);
  R_IMU_EKF[ACC_Y][ACC_Y] = powf(r_meas_accy, 2);
  R_IMU_EKF[ACC_Z][ACC_Z] = powf(r_meas_accz, 2);

  /*Create an identity matrix for EKF algorithm calculation*/
  for (int i = 0; i<STATE_DIM_EKF; i++){
	 identity_state[i][i] = 1;
  }

  isInit = true;
}

/*Functons for enqueueing measurements*/
static bool stateEstimatorEnqueueExternalMeasurement(xQueueHandle queue, void *measurement)
{
  portBASE_TYPE result;
  bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

  if (isInInterrupt) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    result = xQueueSendFromISR(queue, measurement, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken == pdTRUE)
    {
      portYIELD();
    }
  } else {
    result = xQueueSend(queue, measurement, 0);
  }
  return (result==pdTRUE);
}

bool stateEstimatorEnqueueTDOA(tdoaMeasurement_t *uwb)
{
  return stateEstimatorEnqueueExternalMeasurement(tdoaDataQueue, (void *)uwb);
}

bool stateEstimatorEnqueuePosition(positionMeasurement_t *pos)
{
  return stateEstimatorEnqueueExternalMeasurement(posDataQueue, (void *)pos);
}

bool stateEstimatorEnqueueDistance(distanceMeasurement_t *dist)
{
  return stateEstimatorEnqueueExternalMeasurement(distDataQueue, (void *)dist);
}

bool stateEstimatorEnqueueTOF(tofMeasurement_t *tof)
{
  // A distance (distance) [m] to the ground along the z_B axis.
  return stateEstimatorEnqueueExternalMeasurement(tofDataQueue, (void *)tof);
}

bool stateEstimatorTest(void)
{
  return isInit;
}

/*Parmeter and Logging*/

PARAM_GROUP_START(EKF)
  PARAM_ADD(PARAM_UINT16, posUpdate_states, &posUpdate_states)
  PARAM_ADD(PARAM_FLOAT, accBias_y, &accBias_y)
PARAM_GROUP_STOP(EKF)

LOG_GROUP_START(debug_EKF)
    LOG_ADD(LOG_FLOAT, pos_meas_x, &posMeas[0])
    LOG_ADD(LOG_FLOAT, pos_meas_y, &posMeas[1])
    LOG_ADD(LOG_FLOAT, pos_meas_z, &posMeas[2])

	LOG_ADD(LOG_FLOAT, gyro_x, &IMUMeas_debug[0])
	LOG_ADD(LOG_FLOAT, gyro_y, &IMUMeas_debug[1])
	LOG_ADD(LOG_FLOAT, gyro_z, &IMUMeas_debug[2])
	LOG_ADD(LOG_FLOAT, acc_x, &IMUMeas_debug[3])
	LOG_ADD(LOG_FLOAT, acc_y, &IMUMeas_debug[4])
	LOG_ADD(LOG_FLOAT, acc_z, &IMUMeas_debug[5])

	LOG_ADD(LOG_FLOAT, cmd1 , &motorCmds_current[0])
	LOG_ADD(LOG_FLOAT, cmd2 , &motorCmds_current[1])
	LOG_ADD(LOG_FLOAT, cmd3 , &motorCmds_current[2])
	LOG_ADD(LOG_FLOAT, cmd4 , &motorCmds_current[3])

	LOG_ADD(LOG_FLOAT, cp1 , &motor_forces[0])
	LOG_ADD(LOG_FLOAT, cp2 , &motor_forces[1])
	LOG_ADD(LOG_FLOAT, cp3 , &motor_forces[2])
	LOG_ADD(LOG_FLOAT, cp4 , &motor_forces[3])
LOG_GROUP_STOP(debug_EKF)

LOG_GROUP_START(EKF_Martin)
  LOG_ADD(LOG_FLOAT, s1, &xm_EKF[STATE_X_EKF])
  LOG_ADD(LOG_FLOAT, s2, &xm_EKF[STATE_Y_EKF])
  LOG_ADD(LOG_FLOAT, s3, &xm_EKF[STATE_Z_EKF])
  LOG_ADD(LOG_FLOAT, v1, &xm_EKF[STATE_VX_EKF])
  LOG_ADD(LOG_FLOAT, v2, &xm_EKF[STATE_VY_EKF])
  LOG_ADD(LOG_FLOAT, v3, &xm_EKF[STATE_VZ_EKF])
  LOG_ADD(LOG_FLOAT, q0, &xm_EKF[STATE_Q0_EKF])
  LOG_ADD(LOG_FLOAT, q1, &xm_EKF[STATE_Q1_EKF])
  LOG_ADD(LOG_FLOAT, q2, &xm_EKF[STATE_Q2_EKF])
  LOG_ADD(LOG_FLOAT, q3, &xm_EKF[STATE_Q3_EKF])
  LOG_ADD(LOG_FLOAT, p, &xm_EKF[STATE_P_EKF])
  LOG_ADD(LOG_FLOAT, q, &xm_EKF[STATE_Q_EKF])
  LOG_ADD(LOG_FLOAT, r, &xm_EKF[STATE_R_EKF])
  LOG_ADD(LOG_FLOAT, nb1, &xm_EKF[STATE_NB1_EKF])
  LOG_ADD(LOG_FLOAT, nb2, &xm_EKF[STATE_NB2_EKF])
  LOG_ADD(LOG_FLOAT, nb3, &xm_EKF[STATE_NB3_EKF])
  LOG_ADD(LOG_UINT8, Failsafe, &failsafeaktiv)
LOG_GROUP_STOP(EKF_Martin)


LOG_GROUP_START(kalman)
	LOG_ADD(LOG_FLOAT, stateX, &xm_EKF[STATE_X_EKF])
	LOG_ADD(LOG_FLOAT, stateY, &xm_EKF[STATE_Y_EKF])
	LOG_ADD(LOG_FLOAT, stateZ, &xm_EKF[STATE_Z_EKF])
LOG_GROUP_STOP(kalman)


/*Do not delete this one: the failsave mode will be set by the GUI, if Crazyflie is deconnected*/
PARAM_GROUP_START(kalman)
  PARAM_ADD(PARAM_UINT8, resetEstimation, &resetEstimation)
  PARAM_ADD(PARAM_UINT8, Failsafe, &failsafeaktiv)
  PARAM_ADD(PARAM_UINT8, quadIsFlying, &quadIsFlying)
PARAM_GROUP_STOP(kalman)

//PARAM_GROUP_START(measNoiseVariance)
//  PARAM_ADD(PARAM_FLOAT, r_meas_posx, &r_meas_posx)
//  PARAM_ADD(PARAM_FLOAT, r_meas_posy, &r_meas_posy)
//  PARAM_ADD(PARAM_FLOAT, r_meas_posz, &r_meas_posz)
//  PARAM_ADD(PARAM_FLOAT, r_meas_gyrox, &r_meas_gyrox)
//  PARAM_ADD(PARAM_FLOAT, r_meas_gyroy, &r_meas_gyroy)
//  PARAM_ADD(PARAM_FLOAT, r_meas_gyroz, &r_meas_gyroz)
//  PARAM_ADD(PARAM_FLOAT, r_meas_accx, &r_meas_accx)
//  PARAM_ADD(PARAM_FLOAT, r_meas_accy, &r_meas_accy)
//  PARAM_ADD(PARAM_FLOAT, r_meas_accz, &r_meas_accz)
//PARAM_GROUP_STOP(measNoiseVariance)
