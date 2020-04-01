/************************************************************************************
 * Matrix Class 
 *  Contain the matrix class definition and operation.
 * 
 *  Notes:
 *    - Indexing start from 0, with accessing format matrix[row][column].
 *    - The matrix data is a 2 dimensional array, with structure:
 *      ->  0 <= i16row <= (MATRIX_MAXIMUM_SIZE-1)
 *      ->  0 <= i16col <= (MATRIX_MAXIMUM_SIZE-1)
 *      ->  f32data[MATRIX_MAXIMUM_SIZE][MATRIX_MAXIMUM_SIZE] is the memory 
 *           representation of the matrix. We only use the first i16row-th
 *           and first i16col-th memory for the matrix data. The rest is unused.
 *      See below at "Data structure of Matrix class" at private member class
 *       definition for more information!
 * 
 * Class Matrix Versioning:
 *    v0.8 (2020-03-26), {PNb}:
 *      - Change indexing from int32_t to int16_t.
 *      - Add way to initialize matrix with existing float_prec array.
 *      - Add enum InitZero.
 *      - Make temporary matrix initialization inside almost all method with 
 *          NoInitMatZero argument.
 *      - Remove the 1 index buffer reserve in bMatrixIsValid function.
 *      - Add bMatrixIsPositiveDefinite method to check the positive 
 *          (semi)definiteness of a matrix.
 *      - Add GetDiagonalEntries method. 
 *      - Change SYSTEM_IMPLEMENTATION_EMBEDDED_NO_PRINT into
 *          SYSTEM_IMPLEMENTATION_EMBEDDED_CUSTOM, and make vPrint and
 *          vPrintFull as function declaration (the user must define that
 *          function somewhere). 
 * 
 *    v0.7 (2020-02-23), {PNb}:
 *      - Make the matrix class interface in English (at long last, yay?).
 * 
 * 
 *** Documentation below is for tracking purpose *************************************
 * 
 *    v0.6 (2020-01-16), {PNb}:
 *      - Tambahkan sanity check saat pengecekan MATRIX_PAKAI_BOUND_CHECKING 
 *          dengan membandingkan baris & kolom dengan MATRIX_MAXIMUM_SIZE.
 *      - Menambahkan pengecekan matrix untuk operasi dasar antar matrix (*,+,-).
 * 
 *    v0.5 (2020-01-14), {PNb}:
 *      - Buat file matrix.cpp (akhirnya!) untuk definisi fungsi di luar class.
 *      - Tambahkan operator overloading untuk operasi negatif matrix (mis. a = -b).
 *      - Tambahkan operator overloading untuk operasi penjumlahan & pengurangan 
 *          dengan scalar.
 *      - Ubah evaluasi MATRIX_PAKAI_BOUND_CHECKING menggunakan ASSERT.
 *      - Tambahkan pengecekan index selalu positif di MATRIX_PAKAI_BOUND_CHECKING.
 * 
 *    v0.4 (2020-01-10), {PNb}:
 *      - Tambahkan rounding to zero sebelum operasi sqrt(x) untuk menghindari
 *          kasus x = 0-
 *      - Fungsi QRDec mengembalikan Q' dan R (user perlu melakukan transpose
 *          lagi setelah memanggil QRDec untuk mendapatkan Q).
 *      - Menambahkan pengecekan hasil HouseholderTransformQR di dalam QRDec.
 *      - Tambah warning jika MATRIX_PAKAI_BOUND_CHECKING dinonaktifkan.
 * 
 *    v0.3_engl (2019-12-31), {PNb}:
 *      - Modifikasi dokumentasi kode buat orang asing.
 * 
 *    v0.3 (2019-12-25), {PNb}:
 *      - Menambahkan fungsi back subtitution untuk menyelesaikan permasalahan 
 *          persamaan linear Ax = B. Dengan A matrix segitiga atas & B vektor.
 *      - Memperbaiki bug pengecekan MATRIX_PAKAI_BOUND_CHECKING pada indexing kolom.
 *      - Menambahkan fungsi QR Decomposition (via Householder Transformation).
 *      - Menambahkan fungsi Householder Transformation.
 *      - Menghilangkan warning 'implicit conversion' untuk operasi pembandingan
 *          dengan float_prec_ZERO.
 *      - Menambahkan function overloading operasi InsertSubMatrix, untuk
 *          operasi insert dari SubMatrix ke SubMatrix.
 *      - Saat inisialisasi, matrix diisi nol (melalui vIsiHomogen(0.0)).
 *      - Menambahkan function overloading operator '/' dengan scalar.
 * 
 *    v0.2 (2019-11-30), {PNb}:
 *      - Fungsi yang disupport:
 *          - Operator ==
 *          - Normalisasi matrix
 *          - Cholesky Decomposition
 *          - InsertSubMatrix
 *          - InsertVector
 * 
 *    v0.1 (2019-11-29), {PNb}: 
 *      - Fungsi yang disupport:
 *          - Operasi matrix dasar
 *          - Invers
 *          - Cetak
 * 
 * See https://github.com/pronenewbits for more!
 *************************************************************************************/
#ifndef MATRIX_H
#define MATRIX_H

#include "konfig.h"

#if (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_PC)
    #include <iostream>
    #include <iomanip>      // std::setprecision

    using namespace std;
#elif (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO)
    #include <Wire.h>
#endif


class Matrix
{
public:
    typedef enum {
        InitMatWithZero,    /* Initialize matrix with zero */
        NoInitMatZero
    } InitZero;
    
    Matrix(const int16_t _i16row, const int16_t _i16col, InitZero _init = InitMatWithZero)
    {
        this->i16row = _i16row;
        this->i16col = _i16col;
        
        if (_init == InitMatWithZero) {
            this->vSetHomogen(0.0);
        }
    }
    Matrix(const int16_t _i16row, const int16_t _i16col, float_prec * initData, InitZero _init = InitMatWithZero)
    {
        this->i16row = _i16row;
        this->i16col = _i16col;
        
        if (_init == InitMatWithZero) {
            this->vSetHomogen(0.0);
        }
        for (int16_t _i = 0; _i < this->i16row; _i++) {
            for (int16_t _j = 0; _j < this->i16col; _j++) {
                (*this)[_i][_j] = *initData;
                initData++;
            }
        }
    }
    
    bool bMatrixIsValid() {
        /* Check whether the matrix is valid or not.
         * 
         *  Index is for buffer if there's some internal rouge code with 1 index buffer overflow 
         */
        if ((this->i16row > 0) && (this->i16row <= MATRIX_MAXIMUM_SIZE) && (this->i16col > 0) && (this->i16col <= MATRIX_MAXIMUM_SIZE)) {
            return true;
        } else {
            return false;
        }
    }
    
    void vSetMatrixInvalid() {
        this->i16row = -1;
        this->i16col = -1;
    }

    bool bMatrixIsSquare() {
        return (this->i16row == this->i16col);
    }
    
    int16_t i16getRow() { return this->i16row; }
    int16_t i16getCol() { return this->i16col; }
    
    /* Ref: https://stackoverflow.com/questions/6969881/operator-overload */
    class Proxy {
    public:
        Proxy(float_prec* _array, int16_t _maxColumn) : _array(_array) { this->_maxColumn = _maxColumn; }

        /* Modify to be lvalue modifiable, ref:
         * https://stackoverflow.com/questions/6969881/operator-overload#comment30831582_6969904
         * (I know this is so dirty, but it makes the code so FABULOUS :D)
         */
        float_prec & operator[](int16_t _column) {
            #if (defined(MATRIX_USE_BOUND_CHECKING))
                ASSERT((_column >= 0) && (_column < this->_maxColumn) && (_column < MATRIX_MAXIMUM_SIZE), "Matrix index out-of-bounds (at column evaluation)");
            #else
                #warning("Matrix bounds checking is disabled... good luck >:3");
            #endif
            return _array[_column];
        }
    private:
        float_prec* _array;
        int16_t _maxColumn;
    };
    Proxy operator[](int16_t _row) {
        #if (defined(MATRIX_USE_BOUND_CHECKING))
            ASSERT((_row >= 0) && (_row < this->i16row) && (_row < MATRIX_MAXIMUM_SIZE), "Matrix index out-of-bounds (at row evaluation)");
        #else
            #warning("Matrix bounds checking is disabled... good luck >:3");
        #endif
        return Proxy(f32data[_row], this->i16col);      /* Parsing column index for bound checking */
    }

    bool operator == (Matrix _compare) {
        if ((this->i16row != _compare.i16row) || (this->i16col != _compare.i16getCol())) {
            return false;
        }

        for (int16_t _i = 0; _i < this->i16row; _i++) {
            for (int16_t _j = 0; _j < this->i16col; _j++) {
                if (fabs((*this)[_i][_j] - _compare[_i][_j]) > float_prec(float_prec_ZERO)) {
                    return false;
                }
            }
        }
        return true;
    }

    Matrix operator + (Matrix _matAdd) {
        Matrix _outp(this->i16row, this->i16col, NoInitMatZero);
        if ((this->i16row != _matAdd.i16row) || (this->i16col != _matAdd.i16col)) {
            _outp.vSetMatrixInvalid();
            return _outp;
        }

        for (int16_t _i = 0; _i < this->i16row; _i++) {
            for (int16_t _j = 0; _j < this->i16col; _j++) {
                _outp[_i][_j] = (*this)[_i][_j] + _matAdd[_i][_j];
            }
        }
        return _outp;
    }

    Matrix operator - (Matrix _matSub) {
        Matrix _outp(this->i16row, this->i16col, NoInitMatZero);
        if ((this->i16row != _matSub.i16row) || (this->i16col != _matSub.i16col)) {
            _outp.vSetMatrixInvalid();
            return _outp;
        }

        for (int16_t _i = 0; _i < this->i16row; _i++) {
            for (int16_t _j = 0; _j < this->i16col; _j++) {
                _outp[_i][_j] = (*this)[_i][_j] - _matSub[_i][_j];
            }
        }
        return _outp;
    }

    Matrix operator - (void) {
        Matrix _outp(this->i16row, this->i16col, NoInitMatZero);

        for (int16_t _i = 0; _i < this->i16row; _i++) {
            for (int16_t _j = 0; _j < this->i16col; _j++) {
                _outp[_i][_j] = -(*this)[_i][_j];
            }
        }
        return _outp;
    }

    Matrix operator * (Matrix _matMul) {
        Matrix _outp(this->i16row, _matMul.i16col, NoInitMatZero);
        if ((this->i16col != _matMul.i16row)) {
            _outp.vSetMatrixInvalid();
            return _outp;
        }

        for (int16_t _i = 0; _i < this->i16row; _i++) {
            for (int16_t _j = 0; _j < _matMul.i16col; _j++) {
                _outp[_i][_j] = 0.0;
                for (int16_t _k = 0; _k < this->i16col; _k++) {
                    _outp[_i][_j] += ((*this)[_i][_k] * _matMul[_k][_j]);
                }
            }
        }
        return _outp;
    }

    void vRoundingElementToZero(const int16_t _i, const int16_t _j) {
        if (fabs((*this)[_i][_j]) < float_prec(float_prec_ZERO)) {
            (*this)[_i][_j] = 0.0;
        }
    }

    Matrix RoundingMatrixToZero() {
        for (int16_t _i = 0; _i < this->i16row; _i++) {
            for (int16_t _j = 0; _j < this->i16col; _j++) {
                if (fabs((*this)[_i][_j]) < float_prec(float_prec_ZERO)) {
                    (*this)[_i][_j] = 0.0;
                }
            }
        }
        return (*this);
    }

    void vSetHomogen(const float_prec _val) {
        for (int16_t _i = 0; _i < this->i16row; _i++) {
            for (int16_t _j = 0; _j < this->i16col; _j++) {
                (*this)[_i][_j] = _val;
            }
        }
    }

    void vSetToZero() {
        this->vSetHomogen(0.0);
    }

    void vSetRandom(const int32_t _maxRand, const int32_t _minRand) {
        for (int16_t _i = 0; _i < this->i16row; _i++) {
            for (int16_t _j = 0; _j < this->i16col; _j++) {
                (*this)[_i][_j] = float_prec((rand() % (_maxRand - _minRand + 1)) + _minRand);
            }
        }
    }

    void vSetDiag(const float_prec _val) {
        for (int16_t _i = 0; _i < this->i16row; _i++) {
            for (int16_t _j = 0; _j < this->i16col; _j++) {
                if (_i == _j) {
                    (*this)[_i][_j] = _val;
                } else {
                    (*this)[_i][_j] = 0.0;
                }
            }
        }
    }

    void vSetIdentity() {
        this->vSetDiag(1.0);
    }

    /* Insert vector into matrix at _posColumn position
     * Example: A = Matrix 3x3, B = Vector 3x1
     *
     *  C = A.InsertVector(B, 1);
     *
     *  A = [A00  A01  A02]     B = [B00]
     *      [A10  A11  A12]         [B10]
     *      [A20  A21  A22]         [B20]
     *
     *  C = [A00  B00  A02]
     *      [A10  B10  A12]
     *      [A20  B20  A22]
     */
    Matrix InsertVector(Matrix _Vector, const int16_t _posColumn) {
        Matrix _outp(this->i16col, this->i16row, NoInitMatZero);
        if ((_Vector.i16row > this->i16row) || (_Vector.i16col+_posColumn > this->i16col)) {
            /* Return false */
            _outp.vSetMatrixInvalid();
            return _outp;
        }
        _outp = this->Copy();
        for (int16_t _i = 0; _i < _Vector.i16row; _i++) {
            _outp[_i][_posColumn] = _Vector[_i][0];
        }
        return _outp;
    }

    /* Insert submatrix into matrix at _posRow & _posColumn position
     * Example: A = Matrix 4x4, B = Matrix 2x3
     *
     *  C = A.InsertSubMatrix(B, 1, 1);
     *
     *  A = [A00  A01  A02  A03]    B = [B00  B01  B02]
     *      [A10  A11  A12  A13]        [B10  B11  B12]
     *      [A20  A21  A22  A23]
     *      [A30  A31  A32  A33]
     *
     *
     *  C = [A00  A01  A02  A03]
     *      [A10  B00  B01  B02]
     *      [A20  B10  B11  B12]
     *      [A30  A31  A32  A33]
     */
    Matrix InsertSubMatrix(Matrix _subMatrix, const int16_t _posRow, const int16_t _posColumn) {
        Matrix _outp(this->i16col, this->i16row, NoInitMatZero);
        if (((_subMatrix.i16row+_posRow) > this->i16row) || ((_subMatrix.i16col+_posColumn) > this->i16col)) {
            /* Return false */
            _outp.vSetMatrixInvalid();
            return _outp;
        }
        _outp = this->Copy();
        for (int16_t _i = 0; _i < _subMatrix.i16row; _i++) {
            for (int16_t _j = 0; _j < _subMatrix.i16col; _j++) {
                _outp[_i + _posRow][_j + _posColumn] = _subMatrix[_i][_j];
            }
        }
        return _outp;
    }

    /* Insert the first _lenRow-th and first _lenColumn-th submatrix into matrix; at the matrix's _posRow and _posColumn position.
     * Example: A = Matrix 4x4, B = Matrix 2x3
     *
     *  C = A.InsertSubMatrix(B, 1, 1, 2, 2);
     *
     *  A = [A00  A01  A02  A03]    B = [B00  B01  B02]
     *      [A10  A11  A12  A13]        [B10  B11  B12]
     *      [A20  A21  A22  A23]
     *      [A30  A31  A32  A33]
     *
     *
     *  C = [A00  A01  A02  A03]
     *      [A10  B00  B01  A13]
     *      [A20  B10  B11  A23]
     *      [A30  A31  A32  A33]
     */
    Matrix InsertSubMatrix(Matrix _subMatrix, const int16_t _posRow, const int16_t _posColumn, const int16_t _lenRow, const int16_t _lenColumn) {
        Matrix _outp(this->i16col, this->i16row, NoInitMatZero);
        if (((_lenRow+_posRow) > this->i16row) || ((_lenColumn+_posColumn) > this->i16col) || (_lenRow > _subMatrix.i16row) || (_lenColumn > _subMatrix.i16col)) {
            /* Return false */
            _outp.vSetMatrixInvalid();
            return _outp;
        }
        _outp = this->Copy();
        for (int16_t _i = 0; _i < _lenRow; _i++) {
            for (int16_t _j = 0; _j < _lenColumn; _j++) {
                _outp[_i + _posRow][_j + _posColumn] = _subMatrix[_i][_j];
            }
        }
        return _outp;
    }

    /* Insert the _lenRow & _lenColumn submatrix, start from _posRowSub & _posColumnSub submatrix; 
     *  into matrix at the matrix's _posRow and _posColumn position.
     * 
     * Example: A = Matrix 4x4, B = Matrix 2x3
     *
     *  C = A.InsertSubMatrix(B, 1, 1, 0, 1, 1, 2);
     *
     *  A = [A00  A01  A02  A03]    B = [B00  B01  B02]
     *      [A10  A11  A12  A13]        [B10  B11  B12]
     *      [A20  A21  A22  A23]
     *      [A30  A31  A32  A33]
     *
     *
     *  C = [A00  A01  A02  A03]
     *      [A10  B01  B02  A13]
     *      [A20  A21  A22  A23]
     *      [A30  A31  A32  A33]
     */
    Matrix InsertSubMatrix(Matrix _subMatrix, const int16_t _posRow, const int16_t _posColumn,
                           const int16_t _posRowSub, const int16_t _posColumnSub,
                           const int16_t _lenRow, const int16_t _lenColumn) {
        Matrix _outp(this->i16col, this->i16row, NoInitMatZero);
        if (((_lenRow+_posRow) > this->i16row) || ((_lenColumn+_posColumn) > this->i16col) ||
            ((_posRowSub+_lenRow) > _subMatrix.i16row) || ((_posColumnSub+_lenColumn) > _subMatrix.i16col))
        {
            /* Return false */
            _outp.vSetMatrixInvalid();
            return _outp;
        }
        _outp = this->Copy();
        for (int16_t _i = 0; _i < _lenRow; _i++) {
            for (int16_t _j = 0; _j < _lenColumn; _j++) {
                _outp[_i + _posRow][_j + _posColumn] = _subMatrix[_posRowSub+_i][_posColumnSub+_j];
            }
        }
        return _outp;
    }
    
    /* Return the transpose of the matrix */
    Matrix Transpose() {
        Matrix _outp(this->i16col, this->i16row, NoInitMatZero);
        for (int16_t _i = 0; _i < this->i16row; _i++) {
            for (int16_t _j = 0; _j < this->i16col; _j++) {
                _outp[_j][_i] = (*this)[_i][_j];
            }
        }
        return _outp;
    }
    
    /* Normalize the vector */
    bool bNormVector() {
        float_prec _normM = 0.0;
        for (int16_t _i = 0; _i < this->i16row; _i++) {
            for (int16_t _j = 0; _j < this->i16col; _j++) {
                _normM = _normM + ((*this)[_i][_j] * (*this)[_i][_j]);
            }
        }
        
        if (_normM < float_prec(float_prec_ZERO)) {
            return false;
        }
        /* Rounding to zero to avoid case where sqrt(0-) */
        if (fabs(_normM) < float_prec(float_prec_ZERO)) {
            _normM = 0.0;
        }
        _normM = sqrt(_normM);
        for (int16_t _i = 0; _i < this->i16row; _i++) {
            for (int16_t _j = 0; _j < this->i16col; _j++) {
                (*this)[_i][_j] /= _normM;
            }
        }
        return true;
    }
    
    Matrix Copy() {
        Matrix _outp(this->i16row, this->i16col, NoInitMatZero);
        for (int16_t _i = 0; _i < this->i16row; _i++) {
            for (int16_t _j = 0; _j < this->i16col; _j++) {
                _outp[_i][_j] = (*this)[_i][_j];
            }
        }
        return _outp;
    }

    /* Invers operation using Gauss-Jordan algorithm */
    Matrix Invers() {
        Matrix _outp(this->i16row, this->i16col, NoInitMatZero);
        Matrix _temp(this->i16row, this->i16col, NoInitMatZero);
        _outp.vSetIdentity();
        _temp = this->Copy();


        /* Gauss Elimination... */
        for (int16_t _j = 0; _j < (_temp.i16row)-1; _j++) {
            for (int16_t _i = _j+1; _i < _temp.i16row; _i++) {
                if (fabs(_temp[_j][_j]) < float_prec(float_prec_ZERO)) {
                    /* Matrix is non-invertible */
                    _outp.vSetMatrixInvalid();
                    return _outp;
                }

                float_prec _tempfloat = _temp[_i][_j] / _temp[_j][_j];

                for (int16_t _k = 0; _k < _temp.i16col; _k++) {
                    _temp[_i][_k] -= (_temp[_j][_k] * _tempfloat);
                    _outp[_i][_k] -= (_outp[_j][_k] * _tempfloat);

                    _temp.vRoundingElementToZero(_i, _k);
                    _outp.vRoundingElementToZero(_i, _k);
                }

            }
        }

        #if (1)
            /* At here, the _temp matrix should be an upper triangular matrix. 
             * But because rounding error, it might not.
             */
            for (int16_t _i = 1; _i < _temp.i16row; _i++) {
                for (int16_t _j = 0; _j < _i; _j++) {
                    _temp[_i][_j] = 0.0;
                }
            }
        #endif


        /* Jordan... */
        for (int16_t _j = (_temp.i16row)-1; _j > 0; _j--) {
            for (int16_t _i = _j-1; _i >= 0; _i--) {
                if (fabs(_temp[_j][_j]) < float_prec(float_prec_ZERO)) {
                    /* Matrix is non-invertible */
                    _outp.vSetMatrixInvalid();
                    return _outp;
                }

                float_prec _tempfloat = _temp[_i][_j] / _temp[_j][_j];
                _temp[_i][_j] -= (_temp[_j][_j] * _tempfloat);
                _temp.vRoundingElementToZero(_i, _j);

                for (int16_t _k = (_temp.i16row - 1); _k >= 0; _k--) {
                    _outp[_i][_k] -= (_outp[_j][_k] * _tempfloat);
                    _outp.vRoundingElementToZero(_i, _k);
                }
            }
        }


        /* Normalization */
        for (int16_t _i = 0; _i < _temp.i16row; _i++) {
            if (fabs(_temp[_i][_i]) < float_prec(float_prec_ZERO)) {
                /* Matrix is non-invertible */
                _outp.vSetMatrixInvalid();
                return _outp;
            }

            float_prec _tempfloat = _temp[_i][_i];
            _temp[_i][_i] = 1.0;

            for (int16_t _j = 0; _j < _temp.i16row; _j++) {
                _outp[_i][_j] /= _tempfloat;
            }
        }
        return _outp;
    }
    
    /* Use elemtary row operation to reduce the matrix into upper triangular form (like in the first phase of gauss-jordan algorithm).
     * 
     * Useful if we want to check the matrix as positive definite or not (can be used before calling CholeskyDec function).
     */
    bool bMatrixIsPositiveDefinite(bool checkPosSemidefinite = false) {
        bool _posDef, _posSemiDef;
        Matrix _temp(this->i16row, this->i16col, NoInitMatZero);
        _temp = this->Copy();
        
        /* Gauss Elimination... */
        for (int16_t _j = 0; _j < (_temp.i16row)-1; _j++) {
            for (int16_t _i = _j+1; _i < _temp.i16row; _i++) {
                if (fabs(_temp[_j][_j]) < float_prec(float_prec_ZERO)) {
                    /* Q: Do we still need to check this? 
                     * A: idk, It's 3 AM here. 
                     * 
                     * NOTE TO FUTURE SELF: Confirm it!
                     */
                    return false;
                }
                
                float_prec _tempfloat = _temp[_i][_j] / _temp[_j][_j];
                
                for (int16_t _k = 0; _k < _temp.i16col; _k++) {
                    _temp[_i][_k] -= (_temp[_j][_k] * _tempfloat);
                    _temp.vRoundingElementToZero(_i, _k);
                }

            }
        }
        
        _posDef = true;
        _posSemiDef = true;
        for (int16_t _i = 0; _i < _temp.i16row; _i++) {
            if (_temp[_i][_i] < float_prec(float_prec_ZERO)) {      /* false if less than 0+ (zero included) */
                _posDef = false;
            }
            if (_temp[_i][_i] < -float_prec(float_prec_ZERO)) {     /* false if less than 0- (zero is not included) */
                _posSemiDef = false;
            }
        }
        
        if (checkPosSemidefinite) {
            return _posSemiDef;
        } else {
            return _posDef;
        }
    }
    
    
    /* For square matrix 'this' with size MxM, return vector Mx1 with entries corresponding with diagonal entries of 'this'.
     *  Example:    this = [a11 a12 a13]
     *                     [a21 a22 a23]
     *                     [a31 a32 a33]
     * 
     * out = this.GetDiagonalEntries() = [a11]
     *                                   [a22]
     *                                   [a33]
     */
    Matrix GetDiagonalEntries(void) {
        Matrix _temp(this->i16row, 1, NoInitMatZero);
        
        if (this->i16row != this->i16col) {
            _temp.vSetMatrixInvalid();
            return _temp;
        }
        for (int16_t _i = 0; _i < this->i16row; _i++) {
            _temp[_i][0] = (*this)[_i][_i];
        }
        return _temp;
    }
    
    /* Do the Cholesky Decomposition using Cholesky-Crout algorithm.
     * 
     *      A = L*L'     ; A = real, positive definite, and symmetry MxM matrix
     *
     *      L = A.CholeskyDec();
     *
     *      CATATAN! NOTE! The symmetry property is not checked at the beginning to lower 
     *          the computation cost. The processing is being done on the lower triangular
     *          component of _A. Then it is assumed the upper triangular is inherently 
     *          equal to the lower end.
     *          (as a side note, Scilab & MATLAB is using Lapack routines DPOTRF that process
     *           the upper triangular of _A. The result should be equal mathematically if A 
     *           is symmetry).
     */
    Matrix CholeskyDec()
    {
        float_prec _tempFloat;

        Matrix _outp(this->i16row, this->i16col, NoInitMatZero);
        if (this->i16row != this->i16col) {
            _outp.vSetMatrixInvalid();
            return _outp;
        }
        _outp.vSetHomogen(0.0);
        for (int16_t _j = 0; _j < this->i16col; _j++) {
            for (int16_t _i = _j; _i < this->i16row; _i++) {
                _tempFloat = (*this)[_i][_j];
                if (_i == _j) {
                    for (int16_t _k = 0; _k < _j; _k++) {
                        _tempFloat = _tempFloat - (_outp[_i][_k] * _outp[_i][_k]);
                    }
                    if (_tempFloat < float_prec(float_prec_ZERO)) {
                        /* Matrix is not positif definit */
                        _outp.vSetMatrixInvalid();
                        return _outp;
                    }
                    /* Rounding to zero to avoid case where sqrt(0-) */
                    if (fabs(_tempFloat) < float_prec(float_prec_ZERO)) {
                        _tempFloat = 0.0;
                    }
                    _outp[_i][_i] = sqrt(_tempFloat);
                } else {
                    for (int16_t _k = 0; _k < _j; _k++) {
                        _tempFloat = _tempFloat - (_outp[_i][_k] * _outp[_j][_k]);
                    }
                    if (fabs(_outp[_j][_j]) < float_prec(float_prec_ZERO)) {
                        /* Matrix is not positif definit */
                        _outp.vSetMatrixInvalid();
                        return _outp;
                    }
                    _outp[_i][_j] = _tempFloat / _outp[_j][_j];
                }
            }
        }
        return _outp;
    }

    /* Do the Householder Transformation for QR Decomposition operation.
     *              out = HouseholderTransformQR(A, i, j)
     */
    Matrix HouseholderTransformQR(const int16_t _rowTransform, const int16_t _columnTransform)
    {
        float_prec _tempFloat;
        float_prec _xLen;
        float_prec _x1;
        float_prec _u1;
        float_prec _vLen2;

        Matrix _outp(this->i16row, this->i16row, NoInitMatZero);
        Matrix _vectTemp(this->i16row, 1, NoInitMatZero);
        if ((_rowTransform >= this->i16row) || (_columnTransform >= this->i16col)) {
            _outp.vSetMatrixInvalid();
            return _outp;
        }

        /* Until here:
         *
         * _xLen    = ||x||            = sqrt(x1^2 + x2^2 + .. + xn^2)
         * _vLen2   = ||u||^2 - (u1^2) = x2^2 + .. + xn^2
         * _vectTemp= [0 0 0 .. x1=0 x2 x3 .. xn]'
         */
        _x1 = (*this)[_rowTransform][_columnTransform];
        _xLen = _x1*_x1;
        _vLen2 = 0.0;
        for (int16_t _i = _rowTransform+1; _i < this->i16row; _i++) {
            _vectTemp[_i][0] = (*this)[_i][_columnTransform];

            _tempFloat = _vectTemp[_i][0] * _vectTemp[_i][0];
            _xLen  += _tempFloat;
            _vLen2 += _tempFloat;
        }
        _xLen = sqrt(_xLen);

        /* u1    = x1+(-sign(x1))*xLen */
        if (_x1 < 0.0) {
            _u1 = _x1+_xLen;
        } else {
            _u1 = _x1-_xLen;
        }


        /* Solve vlen2 & tempHH */
        _vLen2 += (_u1*_u1);
        _vectTemp[_rowTransform][0] = _u1;

        if (fabs(_vLen2) < float_prec(float_prec_ZERO)) {
            /* x vector is collinear with basis vector e, return result = I */
            _outp.vSetIdentity();
        } else {
            /* P = -2*(u1*u1')/v_len2 + I */
            /* PR TODO: We can do many optimization here */
            for (int16_t _i = 0; _i < this->i16row; _i++) {
                _tempFloat = _vectTemp[_i][0];
                if (fabs(_tempFloat) > float_prec(float_prec_ZERO)) {
                    for (int16_t _j = 0; _j < this->i16row; _j++) {
                        if (fabs(_vectTemp[_j][0]) > float_prec(float_prec_ZERO)) {
                            _outp[_i][_j] = _vectTemp[_j][0];
                            _outp[_i][_j] = _outp[_i][_j] * _tempFloat;
                            _outp[_i][_j] = _outp[_i][_j] * (-2.0/_vLen2);
                        }
                    }
                }
                _outp[_i][_i] = _outp[_i][_i] + 1.0;
            }
        }
        return _outp;
    }

    /* Do the QR Decomposition for matrix using Householder Transformation.
     *                      A = Q*R
     * 
     * PERHATIAN! CAUTION! The matrix calculated by this function return Q' and R (Q transpose and R).
     *  Because QR Decomposition usually used to calculate solution for least-squares equation (that
     *  need Q'), we don't do the transpose of Q inside this routine to lower the computation cost).
     * 
     * Example of using QRDec to solve least-squares:
     *                      Ax = b
     *                   (QR)x = b
     *                      Rx = Q'b    --> Afterward use back-subtitution to solve x
     */
    bool QRDec(Matrix &Qt, Matrix &R)
    {
        Matrix Qn(Qt.i16row, Qt.i16col, NoInitMatZero);
        if ((this->i16row < this->i16col) || (!Qt.bMatrixIsSquare()) || (Qt.i16row != this->i16row) || (R.i16row != this->i16row) || (R.i16col != this->i16col)) {
            Qt.vSetMatrixInvalid();
            R.vSetMatrixInvalid();
            return false;
        }
        R = (*this);
        Qt.vSetIdentity();
        for (int16_t _i = 0; (_i < (this->i16row - 1)) && (_i < this->i16col-1); _i++) {
            Qn  = R.HouseholderTransformQR(_i, _i);
            if (!Qn.bMatrixIsValid()) {
                Qt.vSetMatrixInvalid();
                R.vSetMatrixInvalid();
                return false;
            }
            Qt = Qn * Qt;
            R  = Qn * R;
        }
        Qt.RoundingMatrixToZero();
        /* R.RoundingMatrixToZero(); */
        return true;
    }

    /* Do the back-subtitution opeartion for upper triangular matrix A & column matrix B to solve x:
     *                      Ax = B
     * 
     * x = BackSubtitution(&A, &B);
     *
     * CATATAN! NOTE! To lower the computation cost, we don't check that A is a upper triangular 
     *  matrix (it's assumed that user already make sure before calling this routine).
     */
    Matrix BackSubtitution(Matrix &A, Matrix &B)
    {
        Matrix _outp(A.i16row, 1, NoInitMatZero);
        if ((A.i16row != A.i16col) || (A.i16row != B.i16row)) {
            _outp.vSetMatrixInvalid();
            return _outp;
        }

        for (int16_t _i = A.i16col-1; _i >= 0; _i--) {
            _outp[_i][0] = B[_i][0];
            for (int16_t _j = _i + 1; _j < A.i16col; _j++) {
                _outp[_i][0] = _outp[_i][0] - A[_i][_j]*_outp[_j][0];
            }
            if (fabs(A[_i][_i]) < float_prec(float_prec_ZERO)) {
                _outp.vSetMatrixInvalid();
                return _outp;
            }
            _outp[_i][0] = _outp[_i][0] / A[_i][_i];
        }

        return _outp;
    }

#if (0)
    /*Not yet tested, but should be working (?)*/
    
    /* Melakukan operasi Forward-subtitution pada matrix triangular A & matrix kolom B.
     *                      Ax = B
     *
     *  Untuk menghemat komputansi, matrix A tidak dilakukan pengecekan triangular
     * (diasumsikan sudah lower-triangular).
     */
    Matrix ForwardSubtitution(Matrix &A, Matrix &B)
    {
        Matrix _outp(A.i16row, 1);
        if ((A.i16row != A.i16col) || (A.i16row != B.i16row)) {
            _outp.vSetMatrixInvalid();
            return _outp;
        }

        for (int16_t _i = 0; _i < A.i16row; _i++) {
            _outp[_i][0] = B[_i][0];
            for (int16_t _j = 0; _j < _i; _j++) {
                _outp[_i][0] = _outp[_i][0] - A[_i][_j]*_outp[_j][0];
            }
            if (fabs(A[_i][_i]) < float_prec(float_prec_ZERO)) {
                _outp.vSetMatrixInvalid();
                return _outp;
            }
            _outp[_i][0] = _outp[_i][0] / A[_i][_i];
        }
        return _outp;
    }
    /*Not yet tested, but should be working (?)*/
#endif
    
    
    /* Printing function -------------------------------------------------------------------------------------------- */
    #if (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_PC)
        void vPrint() {
            for (int16_t _i = 0; _i < this->i16row; _i++) {
                cout << "[ ";
                for (int16_t _j = 0; _j < this->i16col; _j++) {
                    cout << std::fixed << std::setprecision(3) << (*this)[_i][_j] << " ";
                }
                cout << "]";
                cout << endl;
            }
            cout << endl;
        }
        void vPrintFull() {
            for (int16_t _i = 0; _i < this->i16row; _i++) {
                cout << "[ ";
                for (int16_t _j = 0; _j < this->i16col; _j++) {
                    cout << resetiosflags( ios::fixed | ios::showpoint ) << (*this)[_i][_j] << " ";
                }
                cout << "]";
                cout << endl;
            }
            cout << endl;
        }
    #elif (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO)
        void vPrint() {
            char _bufSer[10];
            for (int16_t _i = 0; _i < this->i16row; _i++) {
                Serial.print("[ ");
                for (int16_t _j = 0; _j < this->i16col; _j++) {
                    snprintf(_bufSer, sizeof(_bufSer)-1, "%2.2f ", (*this)[_i][_j]);
                    Serial.print(_bufSer);
                }
                Serial.println("]");
            }
            Serial.println("");
        }
        void vPrintFull() {
            char _bufSer[32];
            for (int16_t _i = 0; _i < this->i16row; _i++) {
                Serial.print("[ ");
                for (int16_t _j = 0; _j < this->i16col; _j++) {
                    snprintf(_bufSer, sizeof(_bufSer)-1, "%e ", (*this)[_i][_j]);
                    Serial.print(_bufSer);
                }
                Serial.println("]");
            }
            Serial.println("");
        }
    #else
        /* User must define the print function somewhere */
        void vPrint();
        void vPrintFull();    
    #endif
    /* Printing function -------------------------------------------------------------------------------------------- */
    
    
private:
    /* Data structure of Matrix class:
     *  0 <= i16row <= (MATRIX_MAXIMUM_SIZE-1)      ; i16row is the row of the matrix. i16row is invalid if (i16row == -1)
     *  0 <= i16col <= (MATRIX_MAXIMUM_SIZE-1)      ; i16col is the column of the matrix. i16col is invalid if (i16col == -1)
     * 
     * f32data[MATRIX_MAXIMUM_SIZE][MATRIX_MAXIMUM_SIZE] is the memory representation of the matrix. We only use the first i16row-th
     *  and first i16col-th memory for the matrix data. The rest is unused.
     * 
     * This configuration might seems wasteful (yes it is). But with this, we can make the matrix library code as cleanly as possible 
     *  (like I said in the github page, I've made decision to sacrifice speed & performance to get best code readability I could get).
     * 
     * You could change the data structure of f32data if you want to make the implementation more memory efficient.
     */
    int16_t i16row;
    int16_t i16col;
    float_prec f32data[MATRIX_MAXIMUM_SIZE][MATRIX_MAXIMUM_SIZE] = {{0}};
};


Matrix operator + (const float_prec _scalar, Matrix _mat);
Matrix operator - (const float_prec _scalar, Matrix _mat);
Matrix operator * (const float_prec _scalar, Matrix _mat);
Matrix operator + (Matrix _mat, const float_prec _scalar);
Matrix operator - (Matrix _mat, const float_prec _scalar);
Matrix operator * (Matrix _mat, const float_prec _scalar);
Matrix operator / (Matrix _mat, const float_prec _scalar);

#endif // MATRIX_H
