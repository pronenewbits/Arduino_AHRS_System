/**********************************************************************************************************************
 *  Class for Discrete Unscented Kalman Filter 
 *  Ref: Van der. Merwe, .. (2004). Sigma-Point Kalman Filters for Probabilistic Inference in Dynamic 
 *      State-Space Models (Ph.D. thesis). Oregon Health & Science University.
 *
 *  The system to be estimated is defined as a discrete nonlinear dynamic dystem:
 *              x(k+1) = f[x(k), u(k)] + v(k)           ; x = Nx1,    u = Mx1
 *              y(k)   = h[x(k), u(k)] + n(k)           ; y = Zx1
 *
 *        Where:
 *          x(k) : State Variable at time-k                          : Nx1
 *          y(k) : Measured output at time-k                         : Zx1
 *          u(k) : System input at time-k                            : Mx1
 *          v(k) : Process noise, AWGN assumed, w/ covariance  Rv    : Nx1
 *          n(k) : Measurement noise, AWGN assumed, w/ covariance Rn : Nx1
 *
 *          f(..), h(..) is a nonlinear transformation of the system to be estimated.
 *
 **********************************************************************************************************************
 *      Unscented Kalman Filter algorithm:
 *          Initialization:
 *              P (k=0|k=0) = Identitas * covariant(P(k=0)), typically initialized with some big number.
 *              x(k=0|k=0)  = Expected value of x at time-0 (i.e. x(k=0)), typically set to zero.
 *              Rv, Rn      = Covariance matrices of process & measurement. As this implementation 
 *                              the noise as AWGN (and same value for every variable), this is set
 *                              to Rv=diag(RvInit,...,RvInit) and Rn=diag(RnInit,...,RnInit).
 *              Wc, Wm      = First order & second order weight, respectively.
 *              alpha, beta, kappa, gamma = scalar constants.
 *              
 *              lambda = (alpha^2)*(N+kappa)-N,         gamma = sqrt(N+alpha)           ...{UKF_1}
 *              Wm = [lambda/(N+lambda)         1/(2(N+lambda)) ... 1/(2(N+lambda))]    ...{UKF_2}
 *              Wc = [Wm(0)+(1-alpha(^2)+beta)  1/(2(N+lambda)) ... 1/(2(N+lambda))]    ...{UKF_3}
 *              
 *          
 *          UKF Calculation (every sampling time):
 *              Calculate the Sigma Point:
 *                  Xs(k-1) = [x(k-1) ... x(k-1)]            ; Xs(k-1) = NxN
 *                  GPsq = gamma * sqrt(P(k-1))
 *                  XSigma(k-1) = [x(k-1) Xs(k-1)+GPsq Xs(k-1)-GPsq]                    ...{UKF_4}
 *              
 *              
 *              Unscented Transform XSigma [f,XSigma,u,Rv] -> [x,XSigma,P,DX]:
 *                  XSigma(k) = f(XSigma(k-1), u(k-1))                                  ...{UKF_5a}
 * 
 *                  x(k|k-1) = sum(Wm(i) * XSigma(k)(i))    ; i = 1 ... (2N+1)          ...{UKF_6a}
 * 
 *                  DX = XSigma(k)(i) - Xs(k)   ; Xs(k) = [x(k|k-1) ... x(k|k-1)]
 *                                              ; Xs(k) = Nx(2N+1)                      ...{UKF_7a}
 * 
 *                  P(k|k-1) = sum(Wc(i)*DX*DX') + Rv       ; i = 1 ... (2N+1)          ...{UKF_8a}
 *
 * 
 *              Unscented Transform YSigma [h,XSigma,u,Rn] -> [y_est,YSigma,Py,DY]:
 *                  YSigma(k) = h(XSigma(k), u(k|k-1))      ; u(k|k-1) = u(k)           ...{UKF_5b}
 * 
 *                  y_est(k) = sum(Wm(i) * YSigma(k)(i))    ; i = 1 ... (2N+1)          ...{UKF_6b}
 * 
 *                  DY = YSigma(k)(i) - Ys(k)   ; Ys(k) = [y_est(k) ... y_est(k)]
 *                                              ; Ys(k) = Zx(2N+1)                      ...{UKF_7b}
 * 
 *                  Py(k) = sum(Wc(i)*DY*DY') + Rn          ; i = 1 ... (2N+1)          ...{UKF_8b}
 * 
 * 
 *              Calculate Cross-Covariance Matrix:
 *                  Pxy(k) = sum(Wc(i)*DX*DY(i))            ; i = 1 ... (2N+1)          ...{UKF_9}
 *              
 *              
 *              Calculate the Kalman Gain:
 *                  K           = Pxy(k) * (Py(k)^-1)                                   ...{UKF_10}
 *              
 *              
 *              Update the Estimated State Variable:
 *                  x(k|k)      = x(k|k-1) + K * (y(k) - y_est(k))                      ...{UKF_11}
 *              
 *              
 *              Update the Covariance Matrix:
 *                  P(k|k)      = P(k|k-1) - K*Py(k)*K'                                 ...{UKF_12}
 *
 * 
 *        *Additional Information:
 *              - Dengan asumsi masukan plant ZOH, u(k) = u(k|k-1),
 *                  Dengan asumsi tambahan observer dijalankan sebelum pengendali, u(k|k-1) = u(k-1),
 *                  sehingga u(k) [untuk perhitungan kalman] adalah nilai u(k-1) [dari pengendali].
 *              - Notasi yang benar adalah u(k|k-1), tapi disini menggunakan notasi u(k) untuk
 *                  menyederhanakan penulisan rumus.
 *              - Pada contoh di atas X~(k=0|k=0) = [0]. Untuk mempercepat konvergensi bisa digunakan
 *                  informasi plant-spesific. Misal pada implementasi Kalman Filter untuk sensor
 *                  IMU (Inertial measurement unit) dengan X = [quaternion], dengan asumsi IMU
 *                  awalnya menghadap ke atas tanpa rotasi, X~(k=0|k=0) = [1, 0, 0, 0]'
 *
 * See https://github.com/pronenewbits for more!
 **********************************************************************************************************************/
#include "ukf.h"


UKF::UKF(Matrix &XInit, Matrix &P, Matrix &Rv, Matrix &Rn, bool (*bNonlinearUpdateX)(Matrix &, Matrix &, Matrix &), bool (*bNonlinearUpdateY)(Matrix &, Matrix &, Matrix &))
{ 
    /* Initialization:
     *  P (k=0|k=0) = Identitas * covariant(P(k=0)), typically initialized with some big number.
     *  x(k=0|k=0)  = Expected value of x at time-0 (i.e. x(k=0)), typically set to zero.
     *  Rv, Rn      = Covariance matrices of process & measurement. As this implementation 
     *                the noise as AWGN (and same value for every variable), this is set
     *                to Rv=diag(RvInit,...,RvInit) and Rn=diag(RnInit,...,RnInit).
     */
    X_Est    = XInit;
    this->P  = P.Copy();
    this->Rv = Rv.Copy();
    this->Rn = Rn.Copy();
    this->bNonlinearUpdateX = bNonlinearUpdateX;
    this->bNonlinearUpdateY = bNonlinearUpdateY;
    
    
    /* Van der. Merwe, .. (2004). Sigma-Point Kalman Filters for Probabilistic Inference in Dynamic State-Space Models 
     * (Ph.D. thesis). Oregon Health & Science University. Page 6:
     * 
     * where λ = α2(L+κ)−L is a scaling parameter. α determines the spread of the sigma points around ̄x and is usually 
     * set to a small positive value (e.g. 1e−2 ≤ α ≤ 1). κ is a secondary scaling parameter which is usually set to either 
     * 0 or 3−L (see [45] for details), and β is an extra degree of freedom scalar parameter used to incorporate any extra 
     * prior knowledge of the distribution of x (for Gaussian distributions, β = 2 is optimal).
     */
    float_prec _alpha   = 1e-2;
    float_prec _k       = 0.0;
    float_prec _beta    = 2.0;
    
    /* lambda = (alpha^2)*(N+kappa)-N,         gamma = sqrt(N+alpha)            ...{UKF_1} */
    float_prec _lambda  = (_alpha*_alpha)*(SS_X_LEN+_k) - SS_X_LEN;
    Gamma = sqrt((SS_X_LEN + _lambda));


    /* Wm = [lambda/(N+lambda)         1/(2(N+lambda)) ... 1/(2(N+lambda))]     ...{UKF_2} */
    Wm[0][0] = _lambda/(SS_X_LEN + _lambda);
    for (int16_t _i = 1; _i < Wm.i16getCol(); _i++) {
        Wm[0][_i] = 0.5/(SS_X_LEN + _lambda);
    }
    
    /* Wc = [Wm(0)+(1-alpha(^2)+beta)  1/(2(N+lambda)) ... 1/(2(N+lambda))]     ...{UKF_3} */
    Wc = Wm.Copy();
    Wc[0][0] = Wc[0][0] + (1.0-(_alpha*_alpha)+_beta);
}


void UKF::vReset(Matrix &XInit, Matrix &P, Matrix &Rv, Matrix &Rn)
{
    X_Est    = XInit;
    this->P  = P.Copy();
    this->Rv = Rv.Copy();
    this->Rn = Rn.Copy();
}


bool UKF::bUpdate(Matrix &Y, Matrix &U)
{
    /* Run once every sampling time */

    /* XSigma(k-1) = [x(k-1) Xs(k-1)+GPsq Xs(k-1)-GPsq]                     ...{UKF_4}  */
    if (!bCalculateSigmaPoint()) {
        return false;
    }
    
    
    /* Unscented Transform XSigma [f,XSigma,u,Rv] -> [x,XSigma,P,DX]:       ...{UKF_5a} - {UKF_8a} */
    if (!bUnscentedTransform(X_Est, X_Sigma, P, DX, bNonlinearUpdateX, X_Sigma, U, Rv)) {
        return false;
    }
    
    /* Unscented Transform YSigma [h,XSigma,u,Rn] -> [y_est,YSigma,Py,DY]:  ...{UKF_5b} - {UKF_8b} */
    if (!bUnscentedTransform(Y_Est, Y_Sigma, Py, DY, bNonlinearUpdateY, X_Sigma, U, Rn)) {
        return false;
    }
    
    
    /* Calculate Cross-Covariance Matrix:
     *  Pxy(k) = sum(Wc(i)*DX*DY(i))            ; i = 1 ... (2N+1)          ...{UKF_9}
     */
    for (int16_t _i = 0; _i < DX.i16getRow(); _i++) {
        for (int16_t _j = 0; _j < DX.i16getCol(); _j++) {
            DX[_i][_j] *= Wc[0][_j];
        }
    }
    Pxy = DX * (DY.Transpose());
    
    
    /* Calculate the Kalman Gain:
     *  K           = Pxy(k) * (Py(k)^-1)                                   ...{UKF_10}
     */
    Matrix PyInv = Py.Invers();
    if (!PyInv.bMatrixIsValid()) {
        return false;
    }
    Gain = Pxy * PyInv;
    
    
    /* Update the Estimated State Variable:
     *  x(k|k)      = x(k|k-1) + K * (y(k) - y_est(k))                      ...{UKF_11}
     */
    Err = Y - Y_Est;
    X_Est = X_Est + (Gain*Err);
    
    
    /* Update the Covariance Matrix:
     *  P(k|k)      = P(k|k-1) - K*Py(k)*K'                                 ...{UKF_12}
     */
    P = P - (Gain * Py * Gain.Transpose());


    return true;
}

bool UKF::bCalculateSigmaPoint(void)
{
    /* Xs(k-1) = [x(k-1) ... x(k-1)]            ; Xs(k-1) = NxN
     * GPsq = gamma * sqrt(P(k-1))
     * XSigma(k-1) = [x(k-1) Xs(k-1)+GPsq Xs(k-1)-GPsq]                     ...{UKF_4}
     */
    /* Use Cholesky Decomposition to compute sqrt(P) */
    P_Chol = P.CholeskyDec();
    if (!P_Chol.bMatrixIsValid()) {
        /* System Fail */
        return false;
    }
    P_Chol = P_Chol * Gamma;
    
    /* Xs(k-1) = [x(k-1) ... x(k-1)]            ; Xs(k-1) = NxN */
    Matrix _Y(SS_X_LEN, SS_X_LEN);
    for (int16_t _i = 0; _i < SS_X_LEN; _i++) {
        _Y = _Y.InsertVector(X_Est, _i);
    }
    
    X_Sigma.vSetToZero();
    /* XSigma(k-1) = [x(k-1) 0 0] */
    X_Sigma = X_Sigma.InsertVector(X_Est, 0);
    /* XSigma(k-1) = [x(k-1) Xs(k-1)+GPsq  0] */
    X_Sigma = X_Sigma.InsertSubMatrix((_Y + P_Chol), 0, 1);
    /* XSigma(k-1) = [x(k-1) Xs(k-1)+GPsq Xs(k-1)-GPsq] */
    X_Sigma = X_Sigma.InsertSubMatrix((_Y - P_Chol), 0, (1+SS_X_LEN));

    return true;
}

bool UKF::bUnscentedTransform(Matrix &Out, Matrix &OutSigma, Matrix &P, Matrix &DSig,
                              bool (*_vFuncNonLinear)(Matrix &xOut, Matrix &xInp, Matrix &U),
                              Matrix &InpSigma, Matrix &InpVector,
                              Matrix &_CovNoise)
{
    /* XSigma(k) = f(XSigma(k-1), u(k-1))                                  ...{UKF_5a}  */
    /* x(k|k-1) = sum(Wm(i) * XSigma(k)(i))    ; i = 1 ... (2N+1)          ...{UKF_6a}  */
    Out.vSetToZero();
    for (int16_t _j = 0; _j < InpSigma.i16getCol(); _j++) {
        /* Transform the column submatrix of sigma-points input matrix (InpSigma) */
        Matrix _AuxSigma1(InpSigma.i16getRow(), 1);
        Matrix _AuxSigma2(OutSigma.i16getRow(), 1);
        for (int16_t _i = 0; _i < InpSigma.i16getRow(); _i++) {
            _AuxSigma1[_i][0] = InpSigma[_i][_j];
        }
        if (!_vFuncNonLinear(_AuxSigma2, _AuxSigma1, InpVector)) {
            /* Somehow the transformation function is failed, propagate the error */
            return false;
        }
        
        /* Combine the transformed vector to construct sigma-points output matrix (OutSigma) */
        OutSigma = OutSigma.InsertVector(_AuxSigma2, _j);
        
        /* Calculate x(k|k-1) as weighted mean of OutSigma */
        _AuxSigma2 = _AuxSigma2 * Wm[0][_j];
        Out = Out + _AuxSigma2;
    }

    /* DX = XSigma(k)(i) - Xs(k)   ; Xs(k) = [x(k|k-1) ... x(k|k-1)]
     *                             ; Xs(k) = Nx(2N+1)                      ...{UKF_7a}  */
    Matrix _AuxSigma1(OutSigma.i16getRow(), OutSigma.i16getCol());
    for (int16_t _j = 0; _j < OutSigma.i16getCol(); _j++) {
        _AuxSigma1 = _AuxSigma1.InsertVector(Out, _j);
    }
    DSig = OutSigma - _AuxSigma1;

    /* P(k|k-1) = sum(Wc(i)*DX*DX') + Rv       ; i = 1 ... (2N+1)          ...{UKF_8a}  */
    _AuxSigma1 = DSig.Copy();
    for (int16_t _i = 0; _i < DSig.i16getRow(); _i++) {
        for (int16_t _j = 0; _j < DSig.i16getCol(); _j++) {
            _AuxSigma1[_i][_j] *= Wc[0][_j];
        }
    }
    P = (_AuxSigma1 * (DSig.Transpose())) + _CovNoise;

    return true;
}




