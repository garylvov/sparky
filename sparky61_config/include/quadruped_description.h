#ifndef QUADRUPED_DESCRIPTION_H
#define QUADRUPED_DESCRIPTION_H

#include <quadruped_base/quadruped_base.h>

namespace champ
{
    namespace URDF
    {
        void loadFromHeader(champ::QuadrupedBase &base)
        {
      base.lf.hip.setOrigin(0.111298, 0.074744, 0.019615, 0.0, 0.0, 0.0);
base.lf.upper_leg.setOrigin(-0.036194, 0.036601, -0.042, 0.0, 0.0, 0.0);
base.lf.lower_leg.setOrigin(0.0225, -9.4e-05, -0.045, 0.0, 0.0, 0.0);
     base.lf.foot.setOrigin(0.0, -0.014, -0.1025, 0.0, 0.0, 0.0);

      base.rf.hip.setOrigin(0.111298, -0.075143, 0.019615, 0.0, 0.0, 0.0);
base.rf.upper_leg.setOrigin(-0.036194, -0.0366, -0.042, 0.0, 0.0, 0.0);
base.rf.lower_leg.setOrigin(0.0225, 9.3e-05, -0.045, 0.0, 0.0, 0.0);
     base.rf.foot.setOrigin(0.0, 0.014, -0.1025, 0.0, 0.0, 0.0);

      base.lh.hip.setOrigin(-0.111992, 0.074753, 0.019615, 0.0, 0.0, 0.0);
base.lh.upper_leg.setOrigin(0.036193, 0.0366, -0.042, 0.0, 0.0, 0.0);
base.lh.lower_leg.setOrigin(-0.0225, -9.4e-05, -0.045, 0.0, 0.0, 0.0);
     base.lh.foot.setOrigin(0.0, -0.014, -0.1025, 0.0, 0.0, 0.0);

      base.rh.hip.setOrigin(-0.109652, -0.07416, 0.019615, 0.0, 0.0, 0.0);
base.rh.upper_leg.setOrigin(0.036193, -0.0366, -0.042, 0.0, 0.0, 0.0);
base.rh.lower_leg.setOrigin(-0.0225, 9.3e-05, -0.045, 0.0, 0.0, 0.0);
     base.rh.foot.setOrigin(0.0, 0.014, -0.1025, 0.0, 0.0, 0.0);
        }
    }
}
#endif