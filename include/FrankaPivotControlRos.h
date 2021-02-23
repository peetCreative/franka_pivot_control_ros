//
// Created by peetcreative on 21.02.21.
//

#ifndef SRC_FRANKAPIVOTCONTROLROS_H
#define SRC_FRANKAPIVOTCONTROLROS_H

#include "PivotControlMessages.h"

#include <memory>

namespace franka_pivot_control_ros
{
    class FrankaPivotControlRos
    {
    private:
        std::unique_ptr<FrankaPivotControl> mFrankaPivotControl;
    public:
    };
};

#endif //SRC_FRANKAPIVOTCONTROLROS_H
