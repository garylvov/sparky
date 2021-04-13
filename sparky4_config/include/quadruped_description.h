#ifndef QUADRUPED_DESCRIPTION_H
#define QUADRUPED_DESCRIPTION_H

#include <quadruped_base/quadruped_base.h>

namespace champ
{
    namespace URDF
    {
        void loadFromHeader(champ::QuadrupedBase &base)
        {
      base.lf.hip.setOrigin(0.15347, 0.075135, 0.034036, 0.0, 0.0, 0.0);
base.lf.upper_leg.setOrigin(-0.042618, 0.047126, -0.03454, 0.0, 0.0, 0.0);
base.lf.lower_leg.setOrigin(0.0155, -8.6e-05, -0.064875, 0.0, 0.0, 0.0);
     base.lf.foot.setOrigin(0.0, -0.014, -0.11, 0.0, 0.0, 0.0);

      base.rf.hip.setOrigin(0.15646, -0.073945, 0.034286, 0.0, 0.0, 0.0);
base.rf.upper_leg.setOrigin(-0.042618, -0.047126, -0.03454, 0.0, 0.0, 0.0);
base.rf.lower_leg.setOrigin(0.0155, 8.6e-05, -0.064875, 0.0, 0.0, 0.0);
     base.rf.foot.setOrigin(0.0, 0.014, -0.11, 0.0, 0.0, 0.0);

      base.lh.hip.setOrigin(-0.157043, 0.076035, 0.034296, 0.0, 0.0, 0.0);
base.lh.upper_leg.setOrigin(0.042618, 0.047126, -0.03454, 0.0, 0.0, 0.0);
base.lh.lower_leg.setOrigin(-0.0155, -8.6e-05, -0.064875, 0.0, 0.0, 0.0);
     base.lh.foot.setOrigin(0.0, -0.014, -0.11, 0.0, 0.0, 0.0);

      base.rh.hip.setOrigin(-0.157114, -0.073945, 0.034286, 0.0, 0.0, 0.0);
base.rh.upper_leg.setOrigin(0.042617, -0.047126, -0.03454, 0.0, 0.0, 0.0);
base.rh.lower_leg.setOrigin(-0.0155, 8.6e-05, -0.064875, 0.0, 0.0, 0.0);
     base.rh.foot.setOrigin(0.0, 0.014, -0.11, 0.0, 0.0, 0.0);
        }
    }
}
#endif