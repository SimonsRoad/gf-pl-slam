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

#include <stereoFeatures.h>

namespace StVO{

// Point feature

PointFeature::PointFeature( const Vector3d & P_, const Vector2d & pl_obs_) :
    P(P_), pl_obs(pl_obs_), level(0)
{}

PointFeature::PointFeature( const Vector2d & pl_, const double & disp_, const Vector3d & P_ ) :
    pl(pl_), disp(disp_), P(P_), inlier(true), level(0)
{}

PointFeature::PointFeature( const Vector2d & pl_, const double & disp_, const Vector3d & P_,
                            const int & idx_ ) :
    pl(pl_), disp(disp_), P(P_), inlier(true), idx(idx_), level(0)
{}

PointFeature::PointFeature( const Vector2d & pl_, const double & disp_, const Vector3d & P_,
                            const int & idx_, const int & level_ ) :
    pl(pl_), disp(disp_), P(P_), inlier(true), idx(idx_), level(level_)
{
    for( int i = 0; i < level+1; i++ )
        sigma2 *= Config::orbScaleFactor();
    sigma2 = 1.f / (sigma2*sigma2);
}

PointFeature::PointFeature( const Vector2d & pl_, const double & disp_, const Vector3d & P_,
                            const Vector2d & pl_obs_ ) :
    pl(pl_), disp(disp_), P(P_), pl_obs(pl_obs_), inlier(true), level(0)
{}


// Line segment feature

LineFeature::LineFeature( const Vector3d & sP_, const Vector3d & eP_, const Vector3d & le_obs_) :
    sP(sP_), eP(eP_), le_obs(le_obs_), level(0)
{}


LineFeature::LineFeature( const Vector3d & sP_, const Vector3d & eP_, const Vector3d & le_obs_,
                          const Vector2d & spl_obs_, const Vector2d & epl_obs_) :
    sP(sP_), eP(eP_), le_obs(le_obs_), spl_obs(spl_obs_), epl_obs(epl_obs_), level(0)
{}


LineFeature::LineFeature( const Vector2d & spl_, const double & sdisp_, const Vector3d & sP_,
                          const Vector2d & epl_, const double & edisp_, const Vector3d & eP_,
                          const Vector3d & le_) :
    spl(spl_), sdisp(sdisp_), sP(sP_), epl(epl_), edisp(edisp_), eP(eP_), le(le_), inlier(true), level(0)
{}

LineFeature::LineFeature( const Vector2d & spl_, const double & sdisp_, const Vector3d & sP_,
                          const Vector2d & epl_, const double & edisp_, const Vector3d & eP_,
                          const Vector3d & le_, const Vector3d & le_obs_) :
    spl(spl_), sdisp(sdisp_), sP(sP_), epl(epl_), edisp(edisp_), eP(eP_), le(le_), le_obs(le_obs_), inlier(true), level(0)
{}

LineFeature::LineFeature( const Vector2d & spl_, const double & sdisp_, const Vector3d & sP_,
                          const Vector2d & epl_, const double & edisp_, const Vector3d & eP_,
                          const Vector3d & le_,  const int & idx_) :
    spl(spl_), sdisp(sdisp_), sP(sP_), epl(epl_), edisp(edisp_), eP(eP_), le(le_), inlier(true), idx(idx_), level(0)
{}

LineFeature::LineFeature( const Vector2d & spl_, const double & sdisp_, const Vector3d & sP_,
                          const Vector2d & epl_, const double & edisp_, const Vector3d & eP_,
                          const Vector3d & le_,  const double & angle_, const int & idx_) :
    spl(spl_), sdisp(sdisp_), sP(sP_), epl(epl_), edisp(edisp_), eP(eP_), le(le_), inlier(true), idx(idx_), angle(angle_), level(0)
{}

LineFeature::LineFeature( const Vector2d & spl_, const double & sdisp_, const Vector3d & sP_,
                          const Vector2d & epl_, const double & edisp_, const Vector3d & eP_,
                          const Vector3d & le_,  const double & angle_, const int & idx_, const int & level_) :
    spl(spl_), sdisp(sdisp_), sP(sP_), epl(epl_), edisp(edisp_), eP(eP_), le(le_), inlier(true), idx(idx_), angle(angle_), level(level_)
{
    for( int i = 0; i < level+1; i++ )
        sigma2 *= Config::lsdScale();
    sigma2 = 1.f / (sigma2*sigma2);
}

}
