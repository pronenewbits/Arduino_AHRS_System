#include <Wire.h>
#include <elapsedMillis.h>
#include "konfig.h"
#include "matrix.h"
#include "ekf.h"
#include "simple_mpu9250.h"



/* ================================================= RLS Variables/function declaration ================================================= */
float_prec  RLS_lambda = 0.999; /* Forgetting factor */
Matrix RLS_theta(4,1);          /* The variables we want to indentify */
Matrix RLS_P(4,4);              /* Inverse of correction estimation */
Matrix RLS_in(4,1);             /* Input data */
Matrix RLS_out(1,1);            /* Output data */
Matrix RLS_gain(4,1);           /* RLS gain */
uint32_t RLS_u32iterData = 0;   /* To track how much data we take */


/* ================================================= EKF Variables/function declaration ================================================= */
/* EKF initialization constant */
#define P_INIT      (10.)
#define Q_INIT      (1e-6)
#define R_INIT_ACC  (0.0015/10.)
#define R_INIT_MAG  (0.0015/10.)
/* P(k=0) variable --------------------------------------------------------------------------------------------------------- */
float_prec EKF_PINIT_data[SS_X_LEN*SS_X_LEN] = {P_INIT, 0,      0,      0,
                                                0,      P_INIT, 0,      0,
                                                0,      0,      P_INIT, 0,
                                                0,      0,      0,      P_INIT};
Matrix EKF_PINIT(SS_X_LEN, SS_X_LEN, EKF_PINIT_data);
/* Q constant -------------------------------------------------------------------------------------------------------------- */
float_prec EKF_QINIT_data[SS_X_LEN*SS_X_LEN] = {Q_INIT, 0,      0,      0,
                                                0,      Q_INIT, 0,      0,
                                                0,      0,      Q_INIT, 0,
                                                0,      0,      0,      Q_INIT};
Matrix EKF_QINIT(SS_X_LEN, SS_X_LEN, EKF_QINIT_data);
/* R constant -------------------------------------------------------------------------------------------------------------- */
float_prec EKF_RINIT_data[SS_Z_LEN*SS_Z_LEN] = {R_INIT_ACC, 0,          0,          0,          0,          0,
                                                0,          R_INIT_ACC, 0,          0,          0,          0,
                                                0,          0,          R_INIT_ACC, 0,          0,          0,
                                                0,          0,          0,          R_INIT_MAG, 0,          0,
                                                0,          0,          0,          0,          R_INIT_MAG, 0,
                                                0,          0,          0,          0,          0,          R_INIT_MAG};
Matrix EKF_RINIT(SS_Z_LEN, SS_Z_LEN, EKF_RINIT_data);
/* Nonlinear & linearization function -------------------------------------------------------------------------------------- */
bool Main_bUpdateNonlinearX(Matrix &X_Next, Matrix &X, Matrix &U);
bool Main_bUpdateNonlinearY(Matrix &Y, Matrix &X, Matrix &U);
bool Main_bCalcJacobianF(Matrix &F, Matrix &X, Matrix &U);
bool Main_bCalcJacobianH(Matrix &H, Matrix &X, Matrix &U);
/* EKF variables ----------------------------------------------------------------------------------------------------------- */
Matrix quaternionData(SS_X_LEN, 1);
Matrix Y(SS_Z_LEN, 1);
Matrix U(SS_U_LEN, 1);
EKF EKF_IMU(quaternionData, EKF_PINIT, EKF_QINIT, EKF_RINIT, 
            Main_bUpdateNonlinearX, Main_bUpdateNonlinearY, Main_bCalcJacobianF, Main_bCalcJacobianH);



/* =============================================== Sharing Variables/function declaration =============================================== */
/* Gravity vector constant (align with global Z-axis) */
#define IMU_ACC_Z0          (1)

/* Magnetic vector constant (align with local magnetic vector) */
float_prec IMU_MAG_B0_data[3] = {cos(0), sin(0), 0.000000};
Matrix IMU_MAG_B0(3, 1, IMU_MAG_B0_data);

/* The hard-magnet bias */
float_prec HARD_IRON_BIAS_data[3] = {8.832973, 7.243323, 23.95714};
Matrix HARD_IRON_BIAS(3, 1, HARD_IRON_BIAS_data);

/* An MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68 */
SimpleMPU9250 IMU(Wire, 0x68);

/* State machine for hard-iron bias identification or EKF running */
enum {
    STATE_EKF_RUNNING = 0,
    STATE_MAGNETO_BIAS_IDENTIFICATION,
    STATE_NORTH_VECTOR_IDENTIFICATION
} STATE_AHRS;



/* ============================================== Auxiliary Variables/function declaration ============================================== */
elapsedMillis timerCollectData = 0;
uint64_t u64compuTime;
char bufferTxSer[100];
void serialFloatPrint(float f);
/* The command from the PC */
char cmd;




void setup() {
    /* Serial initialization -------------------------------------- */
    Serial.begin(115200);
    while(!Serial) {}
    Serial.println("Calibrating IMU bias...");

    /* IMU initialization ----------------------------------------- */
    int status = IMU.begin();   /* start communication with IMU */
    if (status < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        while(1) {}
    }
    
    /* RLS initialization ----------------------------------------- */
    RLS_theta.vSetToZero();
    RLS_P.vSetIdentity();
    RLS_P = RLS_P * 1000;
    
    /* EKF initialization ----------------------------------------- */
    /* x(k=0) = [1 0 0 0]' */
    quaternionData.vSetToZero();
    quaternionData[0][0] = 1.0;
    EKF_IMU.vReset(quaternionData, EKF_PINIT, EKF_QINIT, EKF_RINIT);
    
    snprintf(bufferTxSer, sizeof(bufferTxSer)-1, "EKF in Teensy 4.0 (%s)\r\n", (FPU_PRECISION == PRECISION_SINGLE)?"Float32":"Double64");
    Serial.print(bufferTxSer);
    STATE_AHRS = STATE_EKF_RUNNING;
}




void loop() {
    
    if (timerCollectData >= SS_DT_MILIS) {      /* We running the RLS/EKF at sampling time = 20 ms */
        timerCollectData = 0;
        
        /* Read the raw data */
        IMU.readSensor();
        float Ax = IMU.getAccelX_mss();
        float Ay = IMU.getAccelY_mss();
        float Az = IMU.getAccelZ_mss();
        float Bx = IMU.getMagX_uT();
        float By = IMU.getMagY_uT();
        float Bz = IMU.getMagZ_uT();
        float p = IMU.getGyroX_rads();
        float q = IMU.getGyroY_rads();
        float r = IMU.getGyroZ_rads();
            
        if (STATE_AHRS == STATE_EKF_RUNNING) {  /* Run the EKF algorithm */
            
            /* ================== Read the sensor data / simulate the system here ================== */
            
            /* Input 1:3 = gyroscope */
            U[0][0] = p;  U[1][0] = q;  U[2][0] = r;
            /* Output 1:3 = accelerometer */
            Y[0][0] = Ax; Y[1][0] = Ay; Y[2][0] = Az;
            /* Output 4:6 = magnetometer */
            Y[3][0] = Bx; Y[4][0] = By; Y[5][0] = Bz;
            
            /* Compensating Hard-Iron Bias for magnetometer */
            Y[3][0] = Y[3][0]-HARD_IRON_BIAS[0][0];
            Y[4][0] = Y[4][0]-HARD_IRON_BIAS[1][0];
            Y[5][0] = Y[5][0]-HARD_IRON_BIAS[2][0];
            
            /* Normalizing the output vector */
            float_prec _normG = sqrt(Y[0][0] * Y[0][0]) + (Y[1][0] * Y[1][0]) + (Y[2][0] * Y[2][0]);
            Y[0][0] = Y[0][0] / _normG;
            Y[1][0] = Y[1][0] / _normG;
            Y[2][0] = Y[2][0] / _normG;
            float_prec _normM = sqrt(Y[3][0] * Y[3][0]) + (Y[4][0] * Y[4][0]) + (Y[5][0] * Y[5][0]);
            Y[3][0] = Y[3][0] / _normM;
            Y[4][0] = Y[4][0] / _normM;
            Y[5][0] = Y[5][0] / _normM;
            /* ------------------ Read the sensor data / simulate the system here ------------------ */
            
            
            /* ============================= Update the Kalman Filter ============================== */
            u64compuTime = micros();
            if (!EKF_IMU.bUpdate(Y, U)) {
                quaternionData.vSetToZero();
                quaternionData[0][0] = 1.0;
                EKF_IMU.vReset(quaternionData, EKF_PINIT, EKF_QINIT, EKF_RINIT);
                Serial.println("Whoop ");
            }
            u64compuTime = (micros() - u64compuTime);
            /* ----------------------------- Update the Kalman Filter ------------------------------ */
            
        } else if (STATE_AHRS == STATE_MAGNETO_BIAS_IDENTIFICATION) {
            
            /* ================== Read the sensor data / simulate the system here ================== */
            RLS_in[0][0] =  Bx;
            RLS_in[1][0] =  By;
            RLS_in[2][0] =  Bz;
            RLS_in[3][0] =  1;
            RLS_out[0][0] = (Bx*Bx) + (By*By) + (Bz*Bz);
            
            float err = (RLS_out - (RLS_in.Transpose() * RLS_theta))[0][0];
            RLS_gain  = RLS_P*RLS_in / (RLS_lambda + RLS_in.Transpose()*RLS_P*RLS_in)[0][0];
            RLS_P     = (RLS_P - RLS_gain*RLS_in.Transpose()*RLS_P)/RLS_lambda;
            RLS_theta = RLS_theta + err*RLS_gain;
            
            RLS_u32iterData++;

            Matrix P_check(RLS_P.GetDiagonalEntries());
            if ((P_check.Transpose()*P_check)[0][0] < 1e-4) {
                /* The data collection is finished, go back to state EKF running */
                STATE_AHRS = STATE_NORTH_VECTOR_IDENTIFICATION;
                
                /* Reconstruct the matrix compensation solution */
                HARD_IRON_BIAS[0][0] = RLS_theta[0][0] / 2.0;
                HARD_IRON_BIAS[1][0] = RLS_theta[1][0] / 2.0;
                HARD_IRON_BIAS[2][0] = RLS_theta[2][0] / 2.0;
                
                Serial.println("Calibration finished, the hard-iron bias identified:");
                snprintf(bufferTxSer, sizeof(bufferTxSer)-1, "%f %f %f\r\n", HARD_IRON_BIAS[0][0], HARD_IRON_BIAS[1][0], HARD_IRON_BIAS[2][0]);
                Serial.println(bufferTxSer);
                Serial.println("Set the X axis facing north *with z-accelerometer pointed down*, then send command 'f' to finished, 'a' to abort");
            }

            
            if ((RLS_u32iterData % 100) == 0) {
                /* Print every 200 data, so the user can get updated value */
                Matrix P_check(RLS_P.GetDiagonalEntries());
                
                Serial.println("Calibration in progress, the hard-iron bias identified so far:");
                snprintf(bufferTxSer, sizeof(bufferTxSer)-1, "%.3f %.3f %.3f (P = %f)\r\n", RLS_theta[0][0] / 2.0, RLS_theta[1][0] / 2.0, RLS_theta[2][0] / 2.0, (P_check.Transpose()*P_check)[0][0]);
                Serial.println(bufferTxSer);
            }
            if (RLS_u32iterData >= 2000) {
                /* We take the data too long but the error still large, terminate without updating the hard-iron bias */
                STATE_AHRS = STATE_EKF_RUNNING;
                
                Serial.println("Calibration timeout, the hard-iron bias won't be updated\r\n");
            }
            
        } else if (STATE_AHRS == STATE_NORTH_VECTOR_IDENTIFICATION) {
        } else {
            /* You should not be here! */
            STATE_AHRS = STATE_EKF_RUNNING;
        }
    }
    
    
    /* Event serial data: The serial data is sent by responding to command from the PC running Processing scipt or from user command */
    if (Serial.available()) {
        cmd = Serial.read();
        
        
        if (STATE_AHRS == STATE_EKF_RUNNING) {
            
            if (cmd == 'c') {
                /* User want to do online hard-iron bias identification */
                Serial.println("Hard iron bias identification command entered\r\n");
                Serial.println("Rotate the MPU9250 in every direction\r\n");
                Serial.println("Send 'a' to abort\r\n");
                
                /* Initialize RLS */
                RLS_theta.vSetToZero();
                RLS_P.vSetIdentity();
                RLS_P = RLS_P * 1000;
                RLS_u32iterData = 0;
                STATE_AHRS = STATE_MAGNETO_BIAS_IDENTIFICATION;
                
            } else if (cmd == 'p') {
                
                Serial.println("North vector value:");
                snprintf(bufferTxSer, sizeof(bufferTxSer)-1, "%f %f %f\r\n", IMU_MAG_B0[0][0], IMU_MAG_B0[1][0], IMU_MAG_B0[2][0]);
                Serial.print(bufferTxSer);
                
                Serial.println("Hard iron bias value:");
                snprintf(bufferTxSer, sizeof(bufferTxSer)-1, "%f %f %f\r\n", HARD_IRON_BIAS[0][0], HARD_IRON_BIAS[1][0], HARD_IRON_BIAS[2][0]);
                Serial.print(bufferTxSer);
                
            } else if (cmd == 'n') {
                Serial.println("North vector identification command entered\r\n");
                Serial.println("Set the X axis facing north *with z-accelerometer pointed down*, then send command 'f'");
                STATE_AHRS = STATE_NORTH_VECTOR_IDENTIFICATION;
            }
            
            /* Process the command data from Processing script */
            if (cmd == 'v') {
                snprintf(bufferTxSer, sizeof(bufferTxSer)-1, "EKF in Teensy 4.0 (%s)", (FPU_PRECISION == PRECISION_SINGLE)?"Float32":"Double64");
                Serial.print('\n');
                
            } else if (cmd == 'q') {
                /* =========================== Print to serial (for plotting) ========================== */
                quaternionData = EKF_IMU.GetX();

                while (!Serial.available());
                uint8_t count = Serial.read();
                for (uint8_t _i = 0; _i < count; _i++) {
                    serialFloatPrint(quaternionData[0][0]);
                    Serial.print(",");
                    serialFloatPrint(quaternionData[1][0]);
                    Serial.print(",");
                    serialFloatPrint(quaternionData[2][0]);
                    Serial.print(",");
                    serialFloatPrint(quaternionData[3][0]);
                    Serial.print(",");
                    serialFloatPrint((float)u64compuTime);
                    Serial.print(",");
                    Serial.println("");
                }
            }
            
        } else if (STATE_AHRS == STATE_MAGNETO_BIAS_IDENTIFICATION) {
            
            if (cmd == 'a') {
                /* User want to terminate hard iron bias identification */
                Serial.println("Calibration aborted, the hard-iron bias won't be updated\r\n");
                STATE_AHRS = STATE_EKF_RUNNING;
            }

        } else if (STATE_AHRS == STATE_NORTH_VECTOR_IDENTIFICATION) {
            
            if (cmd == 'a') {
                /* User want to terminate hard iron bias identification */
                Serial.println("Calibration aborted, the hard-iron bias won't be updated\r\n");
                STATE_AHRS = STATE_EKF_RUNNING;
                
            } else if (cmd == 'f') {
                float Ax = IMU.getAccelX_mss();
                float Ay = IMU.getAccelY_mss();
                float Az = IMU.getAccelZ_mss();
                float Bx = IMU.getMagX_uT() - HARD_IRON_BIAS[0][0];
                float By = IMU.getMagY_uT() - HARD_IRON_BIAS[1][0];
                float Bz = IMU.getMagZ_uT() - HARD_IRON_BIAS[2][0];
                
                /* Normalizing the acceleration vector & projecting the gravitational vector (gravity is negative acceleration) */
                float_prec _normG = sqrt((Ax * Ax) + (Ay * Ay) + (Az * Az));
                Ax = Ax / _normG;
                Ay = Ay / _normG;
                Az = Az / _normG;
                
                /* Normalizing the magnetic vector */
                _normG = sqrt((Bx * Bx) + (By * By) + (Bz * Bz));
                Bx = Bx / _normG;
                By = By / _normG;
                Bz = Bz / _normG;
        
                /* Projecting the magnetic vector into plane orthogonal to the gravitational vector */
                float pitch = asin(-Ax);
                float roll = asin(Ay/cos(pitch));
                float m_tilt_x =  Bx*cos(pitch)             + By*sin(roll)*sin(pitch)   + Bz*cos(roll)*sin(pitch);
                float m_tilt_y =                            + By*cos(roll)              - Bz*sin(roll);
                /* float m_tilt_z = -Bx*sin(pitch)             + By*sin(roll)*cos(pitch)   + Bz*cos(roll)*cos(pitch); */
                
                float mag_dec = atan2(m_tilt_y, m_tilt_x);
                IMU_MAG_B0[0][0] = cos(mag_dec);
                IMU_MAG_B0[1][0] = sin(mag_dec);
                IMU_MAG_B0[2][0] = 0;
                
                Serial.println("North identification finished, the north vector identified:");
                snprintf(bufferTxSer, sizeof(bufferTxSer)-1, "%.3f %.3f %.3f\r\n", IMU_MAG_B0[0][0], IMU_MAG_B0[1][0], IMU_MAG_B0[2][0]);
                Serial.print(bufferTxSer);
                
                STATE_AHRS = STATE_EKF_RUNNING;
            }

        }
    }
}


/* Function to interface with the Processing script in the PC */
void serialFloatPrint(float f) {
    byte * b = (byte *) &f;
    for (int i = 0; i < 4; i++) {
        byte b1 = (b[i] >> 4) & 0x0f;
        byte b2 = (b[i] & 0x0f);

        char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
        char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

        Serial.print(c1);
        Serial.print(c2);
    }
}

bool Main_bUpdateNonlinearX(Matrix &X_Next, Matrix &X, Matrix &U)
{
    /* Insert the nonlinear update transformation here
     *          x(k+1) = f[x(k), u(k)]
     *
     * The quaternion update function:
     *  q0_dot = 1/2. * (  0   - p*q1 - q*q2 - r*q3)
     *  q1_dot = 1/2. * ( p*q0 +   0  + r*q2 - q*q3)
     *  q2_dot = 1/2. * ( q*q0 - r*q1 +  0   + p*q3)
     *  q3_dot = 1/2. * ( r*q0 + q*q1 - p*q2 +  0  )
     * 
     * Euler method for integration:
     *  q0 = q0 + q0_dot * dT;
     *  q1 = q1 + q1_dot * dT;
     *  q2 = q2 + q2_dot * dT;
     *  q3 = q3 + q3_dot * dT;
     */
    float_prec q0, q1, q2, q3;
    float_prec p, q, r;
    
    q0 = X[0][0];
    q1 = X[1][0];
    q2 = X[2][0];
    q3 = X[3][0];
    
    p = U[0][0];
    q = U[1][0];
    r = U[2][0];
    
    X_Next[0][0] = (0.5 * (+0.00 -p*q1 -q*q2 -r*q3))*SS_DT + q0;
    X_Next[1][0] = (0.5 * (+p*q0 +0.00 +r*q2 -q*q3))*SS_DT + q1;
    X_Next[2][0] = (0.5 * (+q*q0 -r*q1 +0.00 +p*q3))*SS_DT + q2;
    X_Next[3][0] = (0.5 * (+r*q0 +q*q1 -p*q2 +0.00))*SS_DT + q3;
    
    
    /* ======= Additional ad-hoc quaternion normalization to make sure the quaternion is a unit vector (i.e. ||q|| = 1) ======= */
    if (!X_Next.bNormVector()) {
        /* System error, return false so we can reset the UKF */
        return false;
    }
    
    return true;
}

bool Main_bUpdateNonlinearY(Matrix &Y, Matrix &X, Matrix &U)
{
    /* Insert the nonlinear measurement transformation here
     *          y(k)   = h[x(k), u(k)]
     *
     * The measurement output is the gravitational and magnetic projection to the body
     *     DCM     = [(+(q0**2)+(q1**2)-(q2**2)-(q3**2)),                        2*(q1*q2+q0*q3),                        2*(q1*q3-q0*q2)]
     *               [                   2*(q1*q2-q0*q3),     (+(q0**2)-(q1**2)+(q2**2)-(q3**2)),                        2*(q2*q3+q0*q1)]
     *               [                   2*(q1*q3+q0*q2),                        2*(q2*q3-q0*q1),     (+(q0**2)-(q1**2)-(q2**2)+(q3**2))]
     * 
     *  G_proj_sens = DCM * [0 0 1]             --> Gravitational projection to the accelerometer sensor
     *  M_proj_sens = DCM * [Mx My Mz]          --> (Earth) magnetic projection to the magnetometer sensor
     */
    float_prec q0, q1, q2, q3;
    float_prec q0_2, q1_2, q2_2, q3_2;

    q0 = X[0][0];
    q1 = X[1][0];
    q2 = X[2][0];
    q3 = X[3][0];

    q0_2 = q0 * q0;
    q1_2 = q1 * q1;
    q2_2 = q2 * q2;
    q3_2 = q3 * q3;
    
    Y[0][0] = (2*q1*q3 -2*q0*q2) * IMU_ACC_Z0;

    Y[1][0] = (2*q2*q3 +2*q0*q1) * IMU_ACC_Z0;

    Y[2][0] = (+(q0_2) -(q1_2) -(q2_2) +(q3_2)) * IMU_ACC_Z0;
    
    Y[3][0] = (+(q0_2)+(q1_2)-(q2_2)-(q3_2)) * IMU_MAG_B0[0][0]
             +(2*(q1*q2+q0*q3)) * IMU_MAG_B0[1][0]
             +(2*(q1*q3-q0*q2)) * IMU_MAG_B0[2][0];

    Y[4][0] = (2*(q1*q2-q0*q3)) * IMU_MAG_B0[0][0]
             +(+(q0_2)-(q1_2)+(q2_2)-(q3_2)) * IMU_MAG_B0[1][0]
             +(2*(q2*q3+q0*q1)) * IMU_MAG_B0[2][0];

    Y[5][0] = (2*(q1*q3+q0*q2)) * IMU_MAG_B0[0][0]
             +(2*(q2*q3-q0*q1)) * IMU_MAG_B0[1][0]
             +(+(q0_2)-(q1_2)-(q2_2)+(q3_2)) * IMU_MAG_B0[2][0];
    return true;
}

bool Main_bCalcJacobianF(Matrix &F, Matrix &X, Matrix &U)
{
    /* In Main_bUpdateNonlinearX():
     *  q0 = q0 + q0_dot * dT;
     *  q1 = q1 + q1_dot * dT;
     *  q2 = q2 + q2_dot * dT;
     *  q3 = q3 + q3_dot * dT;
     */
    float_prec p, q, r;

    p = U[0][0];
    q = U[1][0];
    r = U[2][0];

    F[0][0] =  1.000;
    F[1][0] =  0.5*p * SS_DT;
    F[2][0] =  0.5*q * SS_DT;
    F[3][0] =  0.5*r * SS_DT;

    F[0][1] = -0.5*p * SS_DT;
    F[1][1] =  1.000;
    F[2][1] = -0.5*r * SS_DT;
    F[3][1] =  0.5*q * SS_DT;

    F[0][2] = -0.5*q * SS_DT;
    F[1][2] =  0.5*r * SS_DT;
    F[2][2] =  1.000;
    F[3][2] = -0.5*p * SS_DT;

    F[0][3] = -0.5*r * SS_DT;
    F[1][3] = -0.5*q * SS_DT;
    F[2][3] =  0.5*p * SS_DT;
    F[3][3] =  1.000;
    
    return true;
}

bool Main_bCalcJacobianH(Matrix &H, Matrix &X, Matrix &U)
{
    float_prec q0, q1, q2, q3;

    q0 = X[0][0];
    q1 = X[1][0];
    q2 = X[2][0];
    q3 = X[3][0];
    
    H[0][0] = -2*q2 * IMU_ACC_Z0;
    H[1][0] = +2*q1 * IMU_ACC_Z0;
    H[2][0] = +2*q0 * IMU_ACC_Z0;
    H[3][0] =  2*q0*IMU_MAG_B0[0][0] + 2*q3*IMU_MAG_B0[1][0] - 2*q2*IMU_MAG_B0[2][0];
    H[4][0] = -2*q3*IMU_MAG_B0[0][0] + 2*q0*IMU_MAG_B0[1][0] + 2*q1*IMU_MAG_B0[2][0];
    H[5][0] =  2*q2*IMU_MAG_B0[0][0] - 2*q1*IMU_MAG_B0[1][0] + 2*q0*IMU_MAG_B0[2][0];
    
    H[0][1] = +2*q3 * IMU_ACC_Z0;
    H[1][1] = +2*q0 * IMU_ACC_Z0;
    H[2][1] = -2*q1 * IMU_ACC_Z0;
    H[3][1] =  2*q1*IMU_MAG_B0[0][0]+2*q2*IMU_MAG_B0[1][0] + 2*q3*IMU_MAG_B0[2][0];
    H[4][1] =  2*q2*IMU_MAG_B0[0][0]-2*q1*IMU_MAG_B0[1][0] + 2*q0*IMU_MAG_B0[2][0];
    H[5][1] =  2*q3*IMU_MAG_B0[0][0]-2*q0*IMU_MAG_B0[1][0] - 2*q1*IMU_MAG_B0[2][0];
    
    H[0][2] = -2*q0 * IMU_ACC_Z0;
    H[1][2] = +2*q3 * IMU_ACC_Z0;
    H[2][2] = -2*q2 * IMU_ACC_Z0;
    H[3][2] = -2*q2*IMU_MAG_B0[0][0]+2*q1*IMU_MAG_B0[1][0] - 2*q0*IMU_MAG_B0[2][0];
    H[4][2] =  2*q1*IMU_MAG_B0[0][0]+2*q2*IMU_MAG_B0[1][0] + 2*q3*IMU_MAG_B0[2][0];
    H[5][2] =  2*q0*IMU_MAG_B0[0][0]+2*q3*IMU_MAG_B0[1][0] - 2*q2*IMU_MAG_B0[2][0];
    
    H[0][3] = +2*q1 * IMU_ACC_Z0;
    H[1][3] = +2*q2 * IMU_ACC_Z0;
    H[2][3] = +2*q3 * IMU_ACC_Z0;
    H[3][3] = -2*q3*IMU_MAG_B0[0][0]+2*q0*IMU_MAG_B0[1][0] + 2*q1*IMU_MAG_B0[2][0];
    H[4][3] = -2*q0*IMU_MAG_B0[0][0]-2*q3*IMU_MAG_B0[1][0] + 2*q2*IMU_MAG_B0[2][0];
    H[5][3] =  2*q1*IMU_MAG_B0[0][0]+2*q2*IMU_MAG_B0[1][0] + 2*q3*IMU_MAG_B0[2][0];
    
    return true;
}



void SPEW_THE_ERROR(char const * str)
{
    #if (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_PC)
        cout << (str) << endl;
    #elif (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO)
        Serial.println(str);
    #else
        /* Silent function */
    #endif
    while(1);
}
