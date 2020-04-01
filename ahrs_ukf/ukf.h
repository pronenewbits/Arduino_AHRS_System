#ifndef UKF_H
#define UKF_H

#include "konfig.h"
#include "matrix.h"


#if (MATRIX_MAXIMUM_SIZE < (2*SS_X_LEN + 1))
    #error("MATRIX_MAXIMUM_SIZE is not big enough for UKF (need at least (2*SS_X_LEN + 1))");
#endif
#if ((MATRIX_MAXIMUM_SIZE < SS_U_LEN) || (MATRIX_MAXIMUM_SIZE < SS_X_LEN) || (MATRIX_MAXIMUM_SIZE < SS_Z_LEN))
    #error("MATRIX_MAXIMUM_SIZE is not big enough for UKF (need at least SS_U_LEN / SS_X_LEN / SS_Z_LEN)");
#endif

class UKF
{
public:
    UKF(Matrix &XInit, Matrix &P, Matrix &Rv, Matrix &Rn, bool (*bNonlinearUpdateX)(Matrix &, Matrix &, Matrix &), bool (*bNonlinearUpdateY)(Matrix &, Matrix &, Matrix &));
    void vReset(Matrix &XInit, Matrix &P, Matrix &Rv, Matrix &Rn);
    bool bUpdate(Matrix &Y, Matrix &U);
    
    Matrix GetX() { return X_Est; }
    Matrix GetY() { return Y_Est; }
    Matrix GetP() { return P; }
    Matrix GetErr() { return Err; }
    
protected:
    bool bCalculateSigmaPoint();
    bool bUnscentedTransform(Matrix &Out, Matrix &OutSigma, Matrix &P, Matrix &DSig,
                             bool (*_vFuncNonLinear)(Matrix &xOut, Matrix &xInp, Matrix &U),
                             Matrix &InpSigma, Matrix &InpVector,
                             Matrix &_CovNoise);
    
private:
    bool (*bNonlinearUpdateX)(Matrix &X_Next, Matrix &X, Matrix &U);
    bool (*bNonlinearUpdateY)(Matrix &Y, Matrix &X, Matrix &U);
    
    Matrix X_Est{SS_X_LEN, 1};
    Matrix X_Sigma{SS_X_LEN, (2*SS_X_LEN + 1)};
    
    Matrix Y_Est{SS_Z_LEN, 1};
    Matrix Y_Sigma{SS_Z_LEN, (2*SS_X_LEN + 1)};
    
    Matrix P{SS_X_LEN, SS_X_LEN};
    Matrix P_Chol{SS_X_LEN, SS_X_LEN};
    
    Matrix DX{SS_X_LEN, (2*SS_X_LEN + 1)};
    Matrix DY{SS_Z_LEN, (2*SS_X_LEN + 1)};
    
    Matrix Py{SS_Z_LEN, SS_Z_LEN};
    Matrix Pxy{SS_X_LEN, SS_Z_LEN};
    
    Matrix Wm{1, (2*SS_X_LEN + 1)};
    Matrix Wc{1, (2*SS_X_LEN + 1)};
    
    Matrix Rv{SS_X_LEN, SS_X_LEN};
    Matrix Rn{SS_Z_LEN, SS_Z_LEN};

    Matrix Err{SS_Z_LEN, 1};
    Matrix Gain{SS_X_LEN, SS_Z_LEN};
    float_prec Gamma;
};

#endif // UKF_H
