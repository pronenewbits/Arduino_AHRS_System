/***************************************************************************************************
 *  Class for Discrete Extended Kalman Filter 
 *  The system to be estimated is defined as a discrete nonlinear dynamic dystem:
 *              x(k) = f[x(k-1), u(k-1)] + v(k)     ; x = Nx1,    u = Mx1
 *              y(k) = h[x(k)] + n(k)               ; y = Zx1
 *
 *        Where:
 *          x(k) : State Variable at time-k                          : Nx1
 *          y(k) : Measured output at time-k                         : Zx1
 *          u(k) : System input at time-k                            : Mx1
 *          v(k) : Process noise, AWGN assumed, w/ covariance Qn     : Nx1
 *          n(k) : Measurement noise, AWGN assumed, w/ covariance Rn : Nx1
 *
 *          f(..), h(..) is a nonlinear transformation of the system to be estimated.
 *
 ***************************************************************************************************
 *      Extended Kalman Filter algorithm:
 *          Initialization:
 *              x(k=0|k=0) = Expected value of x at time-0 (i.e. x(k=0)), typically set to zero.
 *              P(k=0|k=0) = Identity matrix * covariant(P(k=0)), typically initialized with some 
 *                            big number.
 *              Q, R       = Covariance matrices of process & measurement. As this implementation 
 *                            the noise as AWGN (and same value for every variable), this is set
 *                            to Q=diag(QInit,...,QInit) and R=diag(RInit,...,RInit).
 * 
 * 
 *          EKF Calculation (every sampling time):
 *              Calculate the Jacobian matrix of f (i.e. F):
 *                  F = d(f(..))/dx |x(k-1|k-1),u(k-1)                               ...{EKF_1}
 * 
 *              Predict x(k) through nonlinear function f:
 *                  x(k|k-1) = f[x(k-1|k-1), u(k-1)]                                 ...{EKF_2}
 * 
 *              Predict P(k) using linearized f (i.e. F):
 *                  P(k|k-1)  = F*P(k-1|k-1)*F' + Q                                  ...{EKF_3}
 * 
 *              Calculate the Jacobian matrix of h (i.e. C):
 *                  C = d(h(..))/dx |x(k|k-1)                                        ...{EKF_4}
 * 
 *              Predict residual covariance S using linearized h (i.e. H):
 *                  S       = C*P(k|k-1)*C' + R                                      ...{EKF_5}
 * 
 *              Calculate the kalman gain:
 *                  K       = P(k|k-1)*C'*(S^-1)                                     ...{EKF_6}
 * 
 *              Correct x(k) using kalman gain:
 *                  x(k|k) = x(k|k-1) + K*[y(k) - h(x(k|k-1))]                       ...{EKF_7}
 * 
 *              Correct P(k) using kalman gain:
 *                  P(k|k)  = (I - K*C)*P(k|k-1)                                     ...{EKF_8}
 * 
 * 
 *        *Additional Information:
 *              - Pada contoh di atas X~(k=0|k=0) = [0]. Untuk mempercepat konvergensi bisa digunakan
 *                  informasi plant-spesific. Misal pada implementasi Kalman Filter untuk sensor
 *                  IMU (Inertial measurement unit) dengan X = [quaternion], dengan asumsi IMU
 *                  awalnya menghadap ke atas tanpa rotasi, X~(k=0|k=0) = [1, 0, 0, 0]'
 * 
 * 
 * See https://github.com/pronenewbits for more!
 **************************************************************************************************/
#include "ekf.h"


EKF::EKF(Matrix &XInit, Matrix &P, Matrix &Q, Matrix &R,
               bool (*bNonlinearUpdateX)(Matrix &, Matrix &, Matrix &), bool (*bNonlinearUpdateY)(Matrix &, Matrix &, Matrix &), 
               bool (*bCalcJacobianF)(Matrix &, Matrix &, Matrix &), bool (*bCalcJacobianH)(Matrix &, Matrix &, Matrix &))
{
    /* Initialization:
     *  x(k=0|k=0)  = Expected value of x at time-0 (i.e. x(k=0)), typically set to zero.
     *  P (k=0|k=0) = Identity matrix * covariant(P(k=0)), typically initialized with some 
     *                 big number.
     *  Q, R        = Covariance matrices of process & measurement. As this implementation 
     *                 the noise as AWGN (and same value for every variable), this is set
     *                 to Q=diag(QInit,...,QInit) and R=diag(RInit,...,RInit).
     */
    this->X_Est = XInit.Copy();
    this->P = P.Copy();
    this->Q = Q.Copy();
    this->R = R.Copy();
    this->bNonlinearUpdateX = bNonlinearUpdateX;
    this->bNonlinearUpdateY = bNonlinearUpdateY;
    this->bCalcJacobianF = bCalcJacobianF;
    this->bCalcJacobianH = bCalcJacobianH;
}

void EKF::vReset(Matrix &XInit, Matrix &P, Matrix &Q, Matrix &R)
{
    this->X_Est = XInit.Copy();
    this->P = P.Copy();
    this->Q = Q.Copy();
    this->R = R.Copy();
    
//     this->P.vPrint();
//     this->Q.vPrint();
//     this->R.vPrint();
}

bool EKF::bUpdate(Matrix &Y, Matrix &U)
{
    /* Run once every sampling time */
    
    
    /* =============== Calculate the Jacobian matrix of f (i.e. F) =============== */
    /* F = d(f(..))/dx |x(k-1|k-1),u(k-1)                               ...{EKF_1} */
    if (!bCalcJacobianF(F, X_Est, U)) {
        return false;
    }
    
    
    /* =========================== Prediction of x & P =========================== */
    /* x(k|k-1) = f[x(k-1|k-1), u(k-1)]                                 ...{EKF_2} */
    if (!bNonlinearUpdateX(X_Est, X_Est, U)) {
        return false;
    }

    /* P(k|k-1)  = F*P(k-1|k-1)*F' + Q                                  ...{EKF_3} */
    P = F*P*(F.Transpose()) + Q;
    
    
    
    /* =============== Calculate the Jacobian matrix of h (i.e. H) =============== */
    /* H = d(h(..))/dx |x(k|k-1)                                        ...{EKF_4} */
    if (!bCalcJacobianH(H, X_Est, U)) {
        return false;
    }
    
    /* =========================== Correction of x & P =========================== */
    /* S       = H*P(k|k-1)*H' + R                                      ...{EKF_5} */
    S = (H*P*(H.Transpose())) + R;

    /* K       = P(k|k-1)*H'*(S^-1)                                     ...{EKF_6} */
    Gain = P*(H.Transpose())*(S.Invers());
    if (!Gain.bMatrixIsValid()) {
        return false;
    }

    /* x(k|k) = x(k|k-1) + K*[y(k) - h(x(k|k-1))]                       ...{EKF_7} */
    if (!bNonlinearUpdateY(Y_Est, X_Est, U)) {
        return false;
    }
    X_Est = X_Est + (Gain * (Y - Y_Est));

    /* P(k|k)  = (I - K*H)*P(k|k-1)                                     ...{EKF_8} */
    Matrix I = Matrix(SS_X_LEN, SS_X_LEN);
    I.vSetIdentity();
    P = (I - (Gain*H))*P;
    
    
    return true;
}

