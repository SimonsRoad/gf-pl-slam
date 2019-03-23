#pragma once

#include <vector>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
using namespace Eigen;


double get3DLineIntersection(const Vector3d line_1_sp, const Vector3d line_1_ep,
                             const Vector3d line_2_sp, const Vector3d line_2_ep,
                             Vector3d &ipt_1, Vector3d &ipt_2);

void point2LineJacobian(const Matrix<double, 6, 1> & rigframe_T_world,
                        const Matrix<double, 6, 1> & camera_T_rigframe,
                        const Vector3d & lz,
                        const Vector3d & pt3D,
                        Matrix<double, 6, 1> & jac_p2l);

void pointPair2LineJacobian(const Matrix<double, 6, 1> & rigframe_T_world,
                            const Matrix<double, 6, 1> & camera_T_rigframe,
                            const Vector3d & lz,
                            const Vector3d & spt3D,
                            const Vector3d & ept3D,
                            Matrix<double, 6, 1> & jac_spt2l,
                            Matrix<double, 6, 1> & jac_ept2l);

double getDiagonalProduct(const Matrix<double, 6, 6> & mat66);

double getDiagonalNorm(const Matrix<double, 6, 6> & mat66);

void getEigenValue(const Matrix<double, 6, 6> & mat66,
                   vector<double>& ev);

double getMinEigenValue(const Matrix<double, 6, 6> & mat66);

double getLogVolume(const Matrix<double, 6, 6> & mat66);


// set use_cholesky if M is symmetric - it's faster and more stable
// for dep paring it won't be
template <typename MatrixType>
inline typename MatrixType::Scalar logdet(const MatrixType& M) {
    using namespace Eigen;
    using std::log;
    typedef typename MatrixType::Scalar Scalar;
    Scalar ld = 0;
    LLT<Matrix<Scalar,Dynamic,Dynamic>> chol(M);
    //    auto& U = chol.matrixL();
    //    for (unsigned i = 0; i < M.rows(); ++i)
    //      ld += log(U(i,i));
    //    ld *= 2;
    ld = 2 * chol.matrixL().toDenseMatrix().diagonal().array().log().sum();
    return ld;
}
//template <typename MatrixType>
//inline typename MatrixType::Scalar logdet(const MatrixType& M, bool use_cholesky = false) {
//    using namespace Eigen;
//    using std::log;
//    typedef typename MatrixType::Scalar Scalar;
//    Scalar ld = 0;
//    if (use_cholesky) {
//        LLT<Matrix<Scalar,Dynamic,Dynamic>> chol(M);
//        //    auto& U = chol.matrixL();
//        //    for (unsigned i = 0; i < M.rows(); ++i)
//        //      ld += log(U(i,i));
//        //    ld *= 2;
//        ld = 2 * chol.matrixL().toDenseMatrix().diagonal().array().log().sum();
//    } else {
//        PartialPivLU<Matrix<Scalar,Dynamic,Dynamic>> lu(M);
//        auto& LU = lu.matrixLU();
//        Scalar c = lu.permutationP().determinant(); // -1 or 1
//        for (unsigned i = 0; i < LU.rows(); ++i) {
//            const auto& lii = LU(i,i);
//            if (lii < Scalar(0)) c *= -1;
//            ld += log(abs(lii));
//        }
//        ld += log(c);
//    }
//    return ld;
//}



