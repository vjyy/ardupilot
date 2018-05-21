#include "Copter.h"

/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
/*
 *
 * control_moor_nogps.cpp
 *
 *  Created on: 2018年3月12日
 *      Author: luokai
 *      无GPS悬停模式，实际使用自稳模式
 */

bool Copter::moor_nogps_init(bool ignore_checks)
{
    return stabilize_init(ignore_checks);
}



void Copter::moor_nogps_run()
{
    int32_t moor_alt=moor_get_alt_above_ground();
    if(moor_alt>300){
        if(position_ok()){
            set_mode(MOOR,MODE_REASON_UNKNOWN);
        }
    }
    stabilize_run();
}
