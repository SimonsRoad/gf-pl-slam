#include "linespec.h"

double get3DLineIntersection(const Vector3d line_1_sp, const Vector3d line_1_ep,
                             const Vector3d line_2_sp, const Vector3d line_2_ep,
                             Vector3d &ipt_1, Vector3d &ipt_2) {

    Vector3d   u = line_1_ep - line_1_sp;
    Vector3d   v = line_2_ep - line_2_sp;
    Vector3d   w = line_1_sp - line_2_sp;

    double    a = u.dot(u);         // always >= 0
    double    b = u.dot(v);
    double    c = v.dot(v);         // always >= 0
    double    d = u.dot(w);
    double    e = v.dot(w);
    double    D = a*c - b*b;        // always >= 0
    double    sc, tc;

    // compute the line parameters of the two closest points
    if (D < 0.0001) {          // the lines are almost parallel
        sc = 0.0;
        tc = (b>c ? d/b : e/c);    // use the largest denominator
    }
    else {
        sc = (b*e - c*d) / D;
        tc = (a*e - b*d) / D;
    }

    ipt_1 = line_1_sp + sc * u;
    ipt_2 = line_2_sp + tc * v;

    // get the difference of the two closest points
    Vector3d   dP = w + (sc * u) - (tc * v);  // =  lz[0](sc) - lz[1](tc)
    return dP.norm();   // return the closest distance
}

void getEigenValue(const Matrix<double, 6, 6> & mat66,
                   vector<double>& ev) {

    EigenSolver<MatrixXd> es(mat66, false);
    int en = es.eigenvalues().rows();

    ev.clear();
    for (int i=0; i<en; ++i) {
        ev.push_back(es.eigenvalues()(i,0).real());
    }
    std::sort(ev.begin(), ev.end());
    std::reverse(ev.begin(), ev.end());

    return ;
}

double getMinEigenValue(const Matrix<double, 6, 6> & mat66) {

    EigenSolver<MatrixXd> es(mat66, false);
    int en = es.eigenvalues().rows();

    vector<double> ev;
    ev.clear();
    for (int i=0; i<en; ++i) {
        ev.push_back(es.eigenvalues()(i,0).real());
    }
    std::sort(ev.begin(), ev.end());

    return ev[0];
}

double getLogVolume(const Matrix<double, 6, 6> & mat66) {

    EigenSolver<MatrixXd> es(mat66, false);
    int en = es.eigenvalues().rows();

    double vol = 1.0;
    for (int i=0; i<en; ++i) {
        vol *= es.eigenvalues()(i,0).real();
    }

    return std::log(vol);
}


double getDiagonalProduct(const Matrix<double, 6, 6> & mat66) {
    double prod_ = 1.0;
    for (int i=0; i<6; ++i) {
        prod_ *= mat66(i,i);
    }

    return prod_;
}


double getDiagonalNorm(const Matrix<double, 6, 6> & mat66) {
    Matrix<double, 6, 1> dvec;
    for (int i=0; i<6; ++i) {
        dvec(i, 0) = sqrt(mat66(i,i));
    }

    return dvec.norm();
}

// the main code is stole from kraken side: LineJacobian.inl
// it is copied herer for assessing contribution of points along the line towards pose optimization
// the rationale being that, there are some local maximum positions that support line cutting strategy
void point2LineJacobian(const Matrix<double, 6, 1> & rigframe_T_world,
                        const Matrix<double, 6, 1> & camera_T_rigframe,
                        const Vector3d & lz,
                        const Vector3d & pt3D,
                        Matrix<double, 6, 1> & jac_p2l) {

    // const double cam_qw( camera_T_rigframe.rotation.w );
    double cam_qx = camera_T_rigframe[0];
    double cam_qy = camera_T_rigframe[1];
    double cam_qz = camera_T_rigframe[2];
    double cam_tx = camera_T_rigframe[3];
    double cam_ty = camera_T_rigframe[4];
    double cam_tz = camera_T_rigframe[5];

    // const Type rig_qw( rigframe_T_world.rotation.w );
    double rig_qx = rigframe_T_world[0];
    double rig_qy = rigframe_T_world[1];
    double rig_qz = rigframe_T_world[2];
    double rig_tx = rigframe_T_world[3];
    double rig_ty = rigframe_T_world[4];
    double rig_tz = rigframe_T_world[5];

    // Common subexpressions that are used more than once
    double x[194];
    x[0] = sqrt(-rig_qx * rig_qx - rig_qy * rig_qy - rig_qz * rig_qz + 1.0);
    x[1] = 1.0 / x[0];
    x[2] = cam_qz * rig_qx;
    x[3] = cam_qy + x[1] * x[2];
    x[4] = cam_qx * rig_qx;
    x[5] = cam_qy * rig_qy;
    x[6] = cam_qz * rig_qz;
    x[7] = cam_qx * cam_qx;
    x[8] = cam_qy * cam_qy;
    x[9] = cam_qz * cam_qz;
    x[10] = sqrt(-x[7] - x[8] - x[9] + 1.0);
    x[11] = x[0] * x[10];
    x[12] = -x[11] + x[4] + x[5] + x[6];
    x[13] = x[12] * x[3];
    x[14] = rig_qx * x[10];
    x[15] = cam_qx + x[1] * x[14];
    x[16] = cam_qx * rig_qy;
    x[17] = rig_qz * x[10];
    x[18] = cam_qy * rig_qx;
    x[19] = cam_qz * x[0] + x[16] + x[17] - x[18];
    x[20] = x[15] * x[19];
    x[21] = x[1] * x[18];
    x[22] = -cam_qz + x[21];
    x[23] = cam_qy * rig_qz;
    x[24] = cam_qz * rig_qy;
    x[25] = cam_qx * x[0] + x[14] + x[23] - x[24];
    x[26] = -x[10];
    x[27] = x[1] * x[4];
    x[28] = x[26] + x[27];
    x[29] = rig_qy * x[10];
    x[30] = cam_qx * rig_qz;
    x[31] = cam_qy * x[0] + x[2] + x[29] - x[30];
    x[32] = x[22] * x[25] + x[28] * x[31];
    x[33] = x[13] - x[20] + x[32];
    x[34] = x[15] * x[31];
    x[35] = x[25] * x[3];
    x[36] = -x[12] * x[22] + x[19] * x[28] + x[34] + x[35];
    x[37] = x[19] * x[3];
    x[38] = x[22] * x[31];
    x[39] = x[37] + x[38];
    x[40] = 2.0 * pt3D[0];
    x[41] = -x[13] + x[20] + x[32];
    x[42] = x[3] * x[31];
    x[43] = x[19] * x[22];
    x[44] = x[12] * x[28] - x[15] * x[25];
    x[45] = x[42] + x[43] + x[44];
    x[46] = x[25] * x[28];
    x[47] = x[37] + x[46];
    x[48] = 2.0 * pt3D[1];
    x[49] = -x[42] - x[43] + x[44];
    x[50] = x[11] - x[4] - x[5] - x[6];
    x[51] = -x[19] * (x[10] - x[27]) - x[34] + x[35] + x[50] * (cam_qz - x[21]);
    x[52] = x[38] + x[46];
    x[53] = 2.0 * pt3D[2];
    x[54] = -pt3D[0] * x[51] + pt3D[1] * x[49] + x[52] * x[53];
    x[55] = x[12] * x[25];
    x[56] = x[19] * x[31];
    x[57] = -x[56];
    x[58] = x[55] + x[57];
    x[59] = x[12] * x[31];
    x[60] = x[19] * x[25];
    x[61] = x[59] + x[60];
    x[62] = 2.0 * rig_tz;
    x[63] = -x[62] * (x[7] + x[8]);
    x[64] = x[25] * x[25];
    x[65] = x[31] * x[31];
    x[66] = x[64] + x[65];
    x[67] = cam_qx * cam_qz;
    x[68] = cam_qy * x[10];
    x[69] = x[67] - x[68];
    x[70] = 2.0 * rig_tx;
    x[71] = x[69] * x[70];
    x[72] = cam_qx * x[10];
    x[73] = cam_qy * cam_qz;
    x[74] = x[72] + x[73];
    x[75] = 2.0 * rig_ty;
    x[76] = x[74] * x[75];
    x[77] = cam_tz + rig_tz + pt3D[2] - x[53] * x[66] + x[63] + x[71] + x[76];
    x[78] = 1.0 / (x[40] * x[61] - x[48] * x[58] + x[77]);
    x[79] = -x[60];
    x[80] = x[59] + x[79];
    x[81] = x[12] * x[19];
    x[82] = x[25] * x[31];
    x[83] = x[81] + x[82];
    x[84] = -x[70] * (x[8] + x[9]);
    x[85] = x[19] * x[19];
    x[86] = x[65] + x[85];
    x[87] = cam_qx * cam_qy;
    x[88] = cam_qz * x[10];
    x[89] = x[87] - x[88];
    x[90] = x[75] * x[89];
    x[91] = x[67] + x[68];
    x[92] = x[62] * x[91];
    x[93] = cam_tx + rig_tx + pt3D[0] - x[40] * x[86] + x[84] + x[90] + x[92];
    x[94] = lz[0] * x[78] * (x[48] * x[83] - x[53] * x[80] + x[93]);
    x[95] = -x[82];
    x[96] = x[81] + x[95];
    x[97] = 2.0 * x[12] * x[25] + 2.0 * x[19] * x[31];
    x[98] = -x[75] * (x[7] + x[9]);
    x[99] = x[72] - x[73];
    x[100] = -x[62] * x[99];
    x[101] = x[64] + x[85];
    x[102] = x[87] + x[88];
    x[103] = x[102] * x[70];
    x[104] = cam_ty + rig_ty + pt3D[1] + x[100] - x[101] * x[48] + x[103] + x[98];
    x[105] = lz[1] * x[78] * (pt3D[2] * x[97] + x[104] - x[40] * x[96]);
    x[106] = 2.0 * x[78];
    x[107] = x[1] * x[5] + x[26];
    x[108] = x[107] * x[12];
    x[109] = cam_qy + x[1] * x[29];
    x[110] = x[109] * x[31];
    x[111] = cam_qx - x[1] * x[24];
    x[112] = cam_qz + x[1] * x[16];
    x[113] = x[111] * x[25] - x[112] * x[19];
    x[114] = x[108] - x[110] + x[113];
    x[115] = x[111] * x[12];
    x[116] = x[109] * x[19];
    x[117] = x[107] * x[25] + x[112] * x[31];
    x[118] = -x[115] - x[116] + x[117];
    x[119] = x[111] * x[19];
    x[120] = x[107] * x[31];
    x[121] = x[119] - x[120];
    x[122] = x[109] * x[25];
    x[123] = x[112] * x[12];
    x[124] = -x[107] * x[19] + x[111] * x[31];
    x[125] = x[122] - x[123] + x[124];
    x[126] = x[115] + x[116] + x[117];
    x[127] = x[112] * x[25];
    x[128] = x[119] - x[127];
    x[129] = -x[108] + x[110] + x[113];
    x[130] = -x[122] + x[123] + x[124];
    x[131] = x[120] + x[127];
    x[132] = pt3D[0] * x[129] + pt3D[1] * x[130] + x[131] * x[53];
    x[133] = x[1] * x[6] + x[26];
    x[134] = x[12] * x[133];
    x[135] = cam_qz + x[1] * x[17];
    x[136] = x[135] * x[19];
    x[137] = cam_qx + x[1] * x[23];
    x[138] = -cam_qy + x[1] * x[30];
    x[139] = x[137] * x[25] + x[138] * x[31];
    x[140] = x[134] - x[136] + x[139];
    x[141] = x[138] * x[19];
    x[142] = x[133] * x[25];
    x[143] = -x[12] * x[137] + x[135] * x[31];
    x[144] = x[141] + x[142] + x[143];
    x[145] = x[137] * x[31];
    x[146] = x[133] * x[19];
    x[147] = x[145] + x[146];
    x[148] = -x[134] + x[136] + x[139];
    x[149] = x[137] * x[19];
    x[150] = x[133] * x[31];
    x[151] = x[12] * x[138] - x[135] * x[25];
    x[152] = x[149] + x[150] + x[151];
    x[153] = x[138] * x[25];
    x[154] = x[146] + x[153];
    x[155] = -x[141] - x[142] + x[143];
    x[156] = -x[149] - x[150] + x[151];
    x[157] = x[145] + x[153];
    x[158] = pt3D[0] * x[155] + pt3D[1] * x[156] + x[157] * x[53];
    x[159] = x[31] * x[50] + x[79];
    x[160] = -x[55] + x[56];
    x[161] = 1.0 / (-x[159] * x[40] + x[160] * x[48] + x[77]);
    x[162] = x[19] * x[50] + x[95];
    x[163] = -x[59] + x[60];
    x[164] = 2.0 * lz[0] * x[161] * (-x[162] * x[48] + x[163] * x[53] + x[93]);
    x[165] = x[25] * x[50] + x[57];
    x[166] = -x[81] + x[82];
    x[167] = x[161] * (x[104] - x[165] * x[53] + x[166] * x[40]);
    x[168] = 2.0 * lz[1] * x[69];
    x[169] = 2.0 * x[9];
    x[170] = 2.0 * x[8] - 1.0;
    x[171] = 2.0 * lz[1];
    x[172] = -lz[0] * (x[169] + x[170]) + x[102] * x[171];
    x[173] = 2.0 * lz[1] * x[74];
    x[174] = 2.0 * x[7];
    x[175] = 2.0 * lz[0];
    x[176] = -lz[1] * (x[169] + x[174] - 1.0) + x[175] * x[89];
    x[177] = x[170] + x[174];
    x[178] = -x[171] * x[99] + x[175] * x[91];

    jac_p2l[0] =
            x[106] * (lz[0] * (-pt3D[1] * x[33] - pt3D[2] * x[36] + x[39] * x[40]) +
            lz[1] * (-pt3D[0] * x[41] - pt3D[2] * x[45] + x[47] * x[48]) - x[105] * x[54] - x[54] * x[94]);
    jac_p2l[1] = x[106] * (lz[0] * (-pt3D[1] * x[118] + pt3D[2] * x[114] - x[121] * x[40]) +
            lz[1] * (-pt3D[0] * x[126] + pt3D[2] * x[125] - x[128] * x[48]) - x[105] * x[132] -
            x[132] * x[94]);
    jac_p2l[2] = x[106] * (lz[0] * (-pt3D[1] * x[140] - pt3D[2] * x[144] + x[147] * x[40]) +
            lz[1] * (-pt3D[0] * x[148] - pt3D[2] * x[152] + x[154] * x[48]) - x[105] * x[158] -
            x[158] * x[94]);
    jac_p2l[3] = x[161] * (-x[164] * x[69] - x[167] * x[168] + x[172]);
    jac_p2l[4] = x[161] * (-x[164] * x[74] - x[167] * x[173] + x[176]);
    jac_p2l[5] = x[78] * (x[105] * x[177] + x[177] * x[94] + x[178]);

}


void pointPair2LineJacobian(const Matrix<double, 6, 1> & rigframe_T_world,
                            const Matrix<double, 6, 1> & camera_T_rigframe,
                            const Vector3d & lz,
                            const Vector3d & spt3D,
                            const Vector3d & ept3D,
                            Matrix<double, 6, 1> & jac_spt2l,
                            Matrix<double, 6, 1> & jac_ept2l) {

    // const double cam_qw( camera_T_rigframe.rotation.w );
    double cam_qx = camera_T_rigframe[0];
    double cam_qy = camera_T_rigframe[1];
    double cam_qz = camera_T_rigframe[2];
    double cam_tx = camera_T_rigframe[3];
    double cam_ty = camera_T_rigframe[4];
    double cam_tz = camera_T_rigframe[5];

    // const Type rig_qw( rigframe_T_world.rotation.w );
    double rig_qx = rigframe_T_world[0];
    double rig_qy = rigframe_T_world[1];
    double rig_qz = rigframe_T_world[2];
    double rig_tx = rigframe_T_world[3];
    double rig_ty = rigframe_T_world[4];
    double rig_tz = rigframe_T_world[5];

    double sp0 = spt3D[0];
    double sp1 = spt3D[1];
    double sp2 = spt3D[2];
    double ep0 = ept3D[0];
    double ep1 = ept3D[1];
    double ep2 = ept3D[2];
    double l1 = lz[0];
    double l2 = lz[1];
    double l3 = lz[2];

    // Common subexpressions that are used more than once
    double x[194];
    x[0] = sqrt(-rig_qx * rig_qx - rig_qy * rig_qy - rig_qz * rig_qz + double(1));
    x[1] = double(1) / x[0];
    x[2] = cam_qz * rig_qx;
    x[3] = cam_qy + x[1] * x[2];
    x[4] = cam_qx * rig_qx;
    x[5] = cam_qy * rig_qy;
    x[6] = cam_qz * rig_qz;
    x[7] = cam_qx * cam_qx;
    x[8] = cam_qy * cam_qy;
    x[9] = cam_qz * cam_qz;
    x[10] = sqrt(-x[7] - x[8] - x[9] + double(1));
    x[11] = x[0] * x[10];
    x[12] = -x[11] + x[4] + x[5] + x[6];
    x[13] = x[12] * x[3];
    x[14] = rig_qx * x[10];
    x[15] = cam_qx + x[1] * x[14];
    x[16] = cam_qx * rig_qy;
    x[17] = rig_qz * x[10];
    x[18] = cam_qy * rig_qx;
    x[19] = cam_qz * x[0] + x[16] + x[17] - x[18];
    x[20] = x[15] * x[19];
    x[21] = x[1] * x[18];
    x[22] = -cam_qz + x[21];
    x[23] = cam_qy * rig_qz;
    x[24] = cam_qz * rig_qy;
    x[25] = cam_qx * x[0] + x[14] + x[23] - x[24];
    x[26] = -x[10];
    x[27] = x[1] * x[4];
    x[28] = x[26] + x[27];
    x[29] = rig_qy * x[10];
    x[30] = cam_qx * rig_qz;
    x[31] = cam_qy * x[0] + x[2] + x[29] - x[30];
    x[32] = x[22] * x[25] + x[28] * x[31];
    x[33] = x[13] - x[20] + x[32];
    x[34] = x[15] * x[31];
    x[35] = x[25] * x[3];
    x[36] = -x[12] * x[22] + x[19] * x[28] + x[34] + x[35];
    x[37] = x[19] * x[3];
    x[38] = x[22] * x[31];
    x[39] = x[37] + x[38];
    x[40] = double(2) * sp0;
    x[41] = -x[13] + x[20] + x[32];
    x[42] = x[3] * x[31];
    x[43] = x[19] * x[22];
    x[44] = x[12] * x[28] - x[15] * x[25];
    x[45] = x[42] + x[43] + x[44];
    x[46] = x[25] * x[28];
    x[47] = x[37] + x[46];
    x[48] = double(2) * sp1;
    x[49] = -x[42] - x[43] + x[44];
    x[50] = x[11] - x[4] - x[5] - x[6];
    x[51] = -x[19] * (x[10] - x[27]) - x[34] + x[35] + x[50] * (cam_qz - x[21]);
    x[52] = x[38] + x[46];
    x[53] = double(2) * sp2;
    x[54] = -sp0 * x[51] + sp1 * x[49] + x[52] * x[53];
    x[55] = x[12] * x[25];
    x[56] = x[19] * x[31];
    x[57] = -x[56];
    x[58] = x[55] + x[57];
    x[59] = x[12] * x[31];
    x[60] = x[19] * x[25];
    x[61] = x[59] + x[60];
    x[62] = double(2) * rig_tz;
    x[63] = -x[62] * (x[7] + x[8]);
    x[64] = x[25] * x[25];
    x[65] = x[31] * x[31];
    x[66] = x[64] + x[65];
    x[67] = cam_qx * cam_qz;
    x[68] = cam_qy * x[10];
    x[69] = x[67] - x[68];
    x[70] = double(2) * rig_tx;
    x[71] = x[69] * x[70];
    x[72] = cam_qx * x[10];
    x[73] = cam_qy * cam_qz;
    x[74] = x[72] + x[73];
    x[75] = double(2) * rig_ty;
    x[76] = x[74] * x[75];
    x[77] = cam_tz + rig_tz + sp2 - x[53] * x[66] + x[63] + x[71] + x[76];
    x[78] = double(1) / (x[40] * x[61] - x[48] * x[58] + x[77]);
    x[79] = -x[60];
    x[80] = x[59] + x[79];
    x[81] = x[12] * x[19];
    x[82] = x[25] * x[31];
    x[83] = x[81] + x[82];
    x[84] = -x[70] * (x[8] + x[9]);
    x[85] = x[19] * x[19];
    x[86] = x[65] + x[85];
    x[87] = cam_qx * cam_qy;
    x[88] = cam_qz * x[10];
    x[89] = x[87] - x[88];
    x[90] = x[75] * x[89];
    x[91] = x[67] + x[68];
    x[92] = x[62] * x[91];
    x[93] = cam_tx + rig_tx + sp0 - x[40] * x[86] + x[84] + x[90] + x[92];
    x[94] = l1 * x[78] * (x[48] * x[83] - x[53] * x[80] + x[93]);
    x[95] = -x[82];
    x[96] = x[81] + x[95];
    x[97] = double(2) * x[12] * x[25] + double(2) * x[19] * x[31];
    x[98] = -x[75] * (x[7] + x[9]);
    x[99] = x[72] - x[73];
    x[100] = -x[62] * x[99];
    x[101] = x[64] + x[85];
    x[102] = x[87] + x[88];
    x[103] = x[102] * x[70];
    x[104] = cam_ty + rig_ty + sp1 + x[100] - x[101] * x[48] + x[103] + x[98];
    x[105] = l2 * x[78] * (sp2 * x[97] + x[104] - x[40] * x[96]);
    x[106] = double(2) * x[78];
    x[107] = x[1] * x[5] + x[26];
    x[108] = x[107] * x[12];
    x[109] = cam_qy + x[1] * x[29];
    x[110] = x[109] * x[31];
    x[111] = cam_qx - x[1] * x[24];
    x[112] = cam_qz + x[1] * x[16];
    x[113] = x[111] * x[25] - x[112] * x[19];
    x[114] = x[108] - x[110] + x[113];
    x[115] = x[111] * x[12];
    x[116] = x[109] * x[19];
    x[117] = x[107] * x[25] + x[112] * x[31];
    x[118] = -x[115] - x[116] + x[117];
    x[119] = x[111] * x[19];
    x[120] = x[107] * x[31];
    x[121] = x[119] - x[120];
    x[122] = x[109] * x[25];
    x[123] = x[112] * x[12];
    x[124] = -x[107] * x[19] + x[111] * x[31];
    x[125] = x[122] - x[123] + x[124];
    x[126] = x[115] + x[116] + x[117];
    x[127] = x[112] * x[25];
    x[128] = x[119] - x[127];
    x[129] = -x[108] + x[110] + x[113];
    x[130] = -x[122] + x[123] + x[124];
    x[131] = x[120] + x[127];
    x[132] = sp0 * x[129] + sp1 * x[130] + x[131] * x[53];
    x[133] = x[1] * x[6] + x[26];
    x[134] = x[12] * x[133];
    x[135] = cam_qz + x[1] * x[17];
    x[136] = x[135] * x[19];
    x[137] = cam_qx + x[1] * x[23];
    x[138] = -cam_qy + x[1] * x[30];
    x[139] = x[137] * x[25] + x[138] * x[31];
    x[140] = x[134] - x[136] + x[139];
    x[141] = x[138] * x[19];
    x[142] = x[133] * x[25];
    x[143] = -x[12] * x[137] + x[135] * x[31];
    x[144] = x[141] + x[142] + x[143];
    x[145] = x[137] * x[31];
    x[146] = x[133] * x[19];
    x[147] = x[145] + x[146];
    x[148] = -x[134] + x[136] + x[139];
    x[149] = x[137] * x[19];
    x[150] = x[133] * x[31];
    x[151] = x[12] * x[138] - x[135] * x[25];
    x[152] = x[149] + x[150] + x[151];
    x[153] = x[138] * x[25];
    x[154] = x[146] + x[153];
    x[155] = -x[141] - x[142] + x[143];
    x[156] = -x[149] - x[150] + x[151];
    x[157] = x[145] + x[153];
    x[158] = sp0 * x[155] + sp1 * x[156] + x[157] * x[53];
    x[159] = x[31] * x[50] + x[79];
    x[160] = -x[55] + x[56];
    x[161] = double(1) / (-x[159] * x[40] + x[160] * x[48] + x[77]);
    x[162] = x[19] * x[50] + x[95];
    x[163] = -x[59] + x[60];
    x[164] = double(2) * l1 * x[161] * (-x[162] * x[48] + x[163] * x[53] + x[93]);
    x[165] = x[25] * x[50] + x[57];
    x[166] = -x[81] + x[82];
    x[167] = x[161] * (x[104] - x[165] * x[53] + x[166] * x[40]);
    x[168] = double(2) * l2 * x[69];
    x[169] = double(2) * x[9];
    x[170] = double(2) * x[8] - double(1);
    x[171] = double(2) * l2;
    x[172] = -l1 * (x[169] + x[170]) + x[102] * x[171];
    x[173] = double(2) * l2 * x[74];
    x[174] = double(2) * x[7];
    x[175] = double(2) * l1;
    x[176] = -l2 * (x[169] + x[174] - double(1)) + x[175] * x[89];
    x[177] = x[170] + x[174];
    x[178] = -x[171] * x[99] + x[175] * x[91];
    x[179] = double(2) * ep0;
    x[180] = double(2) * ep1;
    x[181] = double(2) * ep2;
    x[182] = -ep0 * x[51] + ep1 * x[49] + x[181] * x[52];
    x[183] = cam_tz + ep2 + rig_tz - x[181] * x[66] + x[63] + x[71] + x[76];
    x[184] = double(1) / (x[179] * x[61] - x[180] * x[58] + x[183]);
    x[185] = cam_tx + ep0 + rig_tx - x[179] * x[86] + x[84] + x[90] + x[92];
    x[186] = l1 * x[184] * (x[180] * x[83] - x[181] * x[80] + x[185]);
    x[187] = cam_ty + ep1 + rig_ty + x[100] - x[101] * x[180] + x[103] + x[98];
    x[188] = l2 * x[184] * (ep2 * x[97] - x[179] * x[96] + x[187]);
    x[189] = double(2) * x[184];
    x[190] = ep0 * x[129] + ep1 * x[130] + x[131] * x[181];
    x[191] = ep0 * x[155] + ep1 * x[156] + x[157] * x[181];
    x[192] = double(1) / (-x[159] * x[179] + x[160] * x[180] + x[183]);
    x[193] = double(2) * l1 * x[192] * (-x[162] * x[180] + x[163] * x[181] + x[185]);
    x[194] = x[192] * (-x[165] * x[181] + x[166] * x[179] + x[187]);

    jac_spt2l[0] =
            x[106] * (l1 * (-sp1 * x[33] - sp2 * x[36] + x[39] * x[40]) +
            l2 * (-sp0 * x[41] - sp2 * x[45] + x[47] * x[48]) - x[105] * x[54] - x[54] * x[94]);
    jac_spt2l[1] = x[106] * (l1 * (-sp1 * x[118] + sp2 * x[114] - x[121] * x[40]) +
            l2 * (-sp0 * x[126] + sp2 * x[125] - x[128] * x[48]) - x[105] * x[132] -
            x[132] * x[94]);
    jac_spt2l[2] = x[106] * (l1 * (-sp1 * x[140] - sp2 * x[144] + x[147] * x[40]) +
            l2 * (-sp0 * x[148] - sp2 * x[152] + x[154] * x[48]) - x[105] * x[158] -
            x[158] * x[94]);
    jac_spt2l[3] = x[161] * (-x[164] * x[69] - x[167] * x[168] + x[172]);
    jac_spt2l[4] = x[161] * (-x[164] * x[74] - x[167] * x[173] + x[176]);
    jac_spt2l[5] = x[78] * (x[105] * x[177] + x[177] * x[94] + x[178]);
    //
    jac_ept2l[0] = x[189] * (l1 * (-ep1 * x[33] - ep2 * x[36] + x[179] * x[39]) +
            l2 * (-ep0 * x[41] - ep2 * x[45] + x[180] * x[47]) - x[182] * x[186] -
            x[182] * x[188]);
    jac_ept2l[1] = x[189] * (l1 * (-ep1 * x[118] + ep2 * x[114] - x[121] * x[179]) +
            l2 * (-ep0 * x[126] + ep2 * x[125] - x[128] * x[180]) - x[186] * x[190] -
            x[188] * x[190]);
    jac_ept2l[2] = x[189] * (l1 * (-ep1 * x[140] - ep2 * x[144] + x[147] * x[179]) +
            l2 * (-ep0 * x[148] - ep2 * x[152] + x[154] * x[180]) - x[186] * x[191] -
            x[188] * x[191]);
    jac_ept2l[3] = x[192] * (-x[168] * x[194] + x[172] - x[193] * x[69]);
    jac_ept2l[4] = x[192] * (-x[173] * x[194] + x[176] - x[193] * x[74]);
    jac_ept2l[5] = x[184] * (x[177] * x[186] + x[177] * x[188] + x[178]);
}
