/**************************************************************************************************
 * Class for Extended Kalman Filter.
 * 
 * 
 * See https://github.com/pronenewbits for more!
 *************************************************************************************************/
#ifndef EKF_H
#define EKF_H

#include "konfig.h"
#include "matrix.h"

class EKF
{
public:
    EKF(Matrix &XInit, Matrix &P, Matrix &Q, Matrix &R,
        bool (*bNonlinearUpdateX)(Matrix &, Matrix &, Matrix &), bool (*bNonlinearUpdateY)(Matrix &, Matrix &, Matrix &), 
        bool (*bCalcJacobianF)(Matrix &, Matrix &, Matrix &), bool (*bCalcJacobianH)(Matrix &, Matrix &, Matrix &));
    void vReset(Matrix &XInit, Matrix &P, Matrix &Q, Matrix &R);
    bool bUpdate(Matrix &Y, Matrix &U);
    Matrix GetX()   { return X_Est; }
    Matrix GetY()   { return Y_Est; }
    Matrix GetP()   { return P; }
    Matrix GetErr() { return Err; }

protected:
    bool (*bNonlinearUpdateX) (Matrix &X_dot, Matrix &X, Matrix &U);
    bool (*bNonlinearUpdateY) (Matrix &Y_Est, Matrix &X, Matrix &U);
    bool (*bCalcJacobianF) (Matrix &F, Matrix &X, Matrix &U);
    bool (*bCalcJacobianH) (Matrix &H, Matrix &X, Matrix &U);

private:
    Matrix X_Est{SS_X_LEN, 1};
    Matrix P{SS_X_LEN, SS_X_LEN};
    Matrix F{SS_X_LEN, SS_X_LEN};
    Matrix H{SS_Z_LEN, SS_X_LEN};
    Matrix Y_Est{SS_Z_LEN, 1};
    Matrix Err{SS_Z_LEN, 1};
    Matrix Q{SS_X_LEN, SS_X_LEN};
    Matrix R{SS_Z_LEN, SS_Z_LEN};
    Matrix S{SS_Z_LEN, SS_Z_LEN};
    Matrix Gain{SS_X_LEN, SS_Z_LEN};
};

#endif // EKF_H
