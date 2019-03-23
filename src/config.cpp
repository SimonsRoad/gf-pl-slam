/*****************************************************************************
**   PL-SLAM: stereo visual SLAM with points and line segment features  	**
******************************************************************************
**																			**
**	Copyright(c) 2017, Ruben Gomez-Ojeda, University of Malaga              **
**	Copyright(c) 2017, MAPIR group, University of Malaga					**
**																			**
**  This program is free software: you can redistribute it and/or modify	**
**  it under the terms of the GNU General Public License (version 3) as		**
**	published by the Free Software Foundation.								**
**																			**
**  This program is distributed in the hope that it will be useful, but		**
**	WITHOUT ANY WARRANTY; without even the implied warranty of				**
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the			**
**  GNU General Public License for more details.							**
**																			**
**  You should have received a copy of the GNU General Public License		**
**  along with this program.  If not, see <http://www.gnu.org/licenses/>.	**
**																			**
*****************************************************************************/

#include <config.h>

#define PI std::acos(-1.0)

Config::Config()
{

    // SLAM parameters
    // -----------------------------------------------------------------------------------------------------
    // track loss definition
    max_num_frame_loss    = 10; // 40; // 1000;
    // kf decision
    min_entropy_ratio     = 0.90;
    max_kf_num_frames     = 50; // 100;
    min_kf_n_feats        = 30;
    max_kf_t_dist         = 2.0;
    max_kf_r_dist         = 5.0;
    // lm numbers and errors
    min_lm_obs            = 2;
    max_common_fts_kf     = 0.8; // 0.9; // tmp change by Yipu
    max_kf_epip_p         = 1.0;
    max_kf_epip_l         = 1.0;
    max_lm_3d_err         = 1.0;
    max_lm_dir_err        = 0.5;       // cos(60deg)
    max_point_point_error = 0.1;
    max_point_line_error  = 0.1;
    max_dir_line_error    = 0.1;
    // graphs parameters
    min_lm_ess_graph      = 100;
    min_lm_cov_graph      = 30;
    // modified by Yipu
    min_kf_local_map      = 3;  // 6; //    // try increasing the size of local map for better performance
    // LBA
    lambda_lba_lm         = 0.001;     // (if auto, this is the initial tau)
    lambda_lba_k          = 10.0;
    max_iters_lba         = 20; // 40; // tmp change by Yipu
    // Loop closure
    vocabulary_p          = "../vocabulary/voc_all_datasets_orb.yml";
    vocabulary_l          = "../vocabulary/voc_all_datasets_bld.yml";
    lc_mat                = 0.50;
    lc_res                = 1.5;
    lc_unc                = 0.01;
    lc_inl                = 0.3;
    lc_trs                = 1.5;
    lc_rot                = 35.0;
    max_iters_pgo         = 100;
    lc_kf_dist            = 100;
    lc_kf_max_dist        = 20;
    lc_nkf_closest        = 4;
    lc_inlier_ratio       = 35.0;

    // StVO-PL options
    // -----------------------------------------------------------------------------------------------------
    has_points         = true;  // false; //        // true if using points
    has_lines          = true;  // false; //    // true if using line segments
    lr_in_parallel     = true;  // false; //     // true if detecting and matching features in parallel
    pl_in_parallel     = true;  // false; //    // true if detecting points and line segments in parallel
    best_lr_matches    = true;      // true if double-checking the matches between the two images
    adaptative_fast    = true;      // true if using adaptative fast_threshold
    //    use_line_weight    = true; //  false; //       // true if weight line loss term according to the level of uncertainty
    //
    use_line_conf_cut  = true; // false; //        // true if cut line before feeding into optimization based on the level of uncertainty
    ratio_disp_std     = 0.15;   // 0.5; // 0.3;  //
    ratio_disp_std_hor = 0.9;  // 0.3; //
    max_vol_line_cut   = true; // false;  //       // true if cut line with max-vol objective
    //    // gazebo
    //    max_line_match_num = 300; // 80;
    //    max_point_match_num = 1000; // 150;
    // euroc
    // try setting a much tighter budget
    //    max_line_match_num  = 100; // 50; // 100; // 300; // 30; //
    //    max_point_match_num = 50; // 100; // 500; // 30; // 150; // 50;  //
    max_line_match_num = 300; // 80;
    max_point_match_num = 500; // 1000; // 150;


    // Tracking parameters
    // -----------------------------------------------------------------------------------------------------
    // Point features
    max_dist_epip     = 2.0;         // max. epipolar distance in pixels
    min_disp          = 1.0;         // min. disparity (avoid points in the infinite)
    max_ratio_12_p    = 0.9; //  0.1; //   0.5; // 0.75; //        // min. ratio between the first and second best matches
    point_match_radius = 50.0; // 80.0; // 120.0; // 
    // Line segment features
    stereo_overlap_th = 0.5; // 0.75;
    min_line_length   = 0.025;   // 0.04; //     // min. line length (relative to img size)
    line_horiz_th     = 0.1;         // parameter to avoid horizontal lines
    desc_th_l         = 0.1;  // 0.9; //        // parameter to avoid outliers in line matching
    line_cov_th       = 10.0;        // parameter to remove noisy line segments
    line_match_radius = 80.0;
    // Adaptative FAST parameters
    fast_min_th       = 10;           // min. value for FAST threshold
    fast_max_th       = 50;          // max. value for FAST threshold
    fast_inc_th       = 5;           // base increment for the FAST threshold
    fast_feat_th      = 50;          // base number of features to increase/decrease FAST threshold
    fast_err_th       = 0.5;         // threshold for the optimization error

    // Optimization parameters
    // -----------------------------------------------------------------------------------------------------
    homog_th         = 0.0000001;   // avoid points in the infinite
    min_features     = 10; // line only      20; // other
    //
    max_iters        = 5;       // 10; // tmp change by Yipu    // max. number of iterations in the first stage of the optimization
    max_iters_ref    = 10;      // 20; // tmp change by Yipu    // max. number of iterations in the refinement stage
    min_error        = 0.0000001;   // min. error to stop the optimization
    min_error_change = 0.0000001;   // min. error change to stop the optimization
    inlier_k         = 2.0;         // factor to discard outliers before the refinement stage
    motion_step_th   = 10; // 1.0;      // 10 m/s as max. velocity

    // Feature detection parameters
    // -----------------------------------------------------------------------------------------------------
    // ORB detector
    orb_nfeatures    = 1000; // 1200; //
    orb_scale_factor = 1.2;
    orb_nlevels      = 4;   // 2; // 8; //
    orb_edge_th      = 19;
    orb_wta_k        = 2;
    orb_score        = 1;           // 0 - HARRIS  |  1 - FAST
    orb_patch_size   = 31;
    orb_fast_th      = 20;          // default FAST threshold
    // LSD parameters
    lsd_nfeatures    = 300;         // set to 0 if keeping all lines
    lsd_refine       = 1; // 2;
    lsd_scale        = 1; // 1.2; // 2; //
    lsd_octave_num   = 1; // 2; //
    lsd_sigma_scale  = 0.75; // 0.6;
    lsd_quant        = 2.0;
    lsd_ang_th       = 22.5;
    lsd_log_eps      = 1.0;
    lsd_density_th   = 0.6;
    lsd_n_bins       = 1024;

}

Config::~Config(){}

Config& Config::getInstance()
{
    static Config instance; // Instantiated on first use and guaranteed to be destroyed
    return instance;
}
