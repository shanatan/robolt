#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#include "port_IF.h"

#include "tasksync_IF.h"
#include "sensor_IF.h"

extern  TASK_MES    gkTaskMes;
extern  SENSOR_VAL  gkSensorVal;

TASK(BackGround)
{
    for(;;) {
        /* ’´‰¹”gƒZƒ“ƒT */
        gkSensorVal.sonar = ecrobot_get_sonar_sensor(PORT_SONAR);
        //display_clear(0);
        //display_goto_xy(0, 0);
        //display_int(sonar, 5);
        //display_update();

        if (gkSensorVal.sonar != 255) {
            if (gkSensorVal.sonar < 35) {
                //ecrobot_sound_tone(1000, 100, 25);
                gkTaskMes.message = MES_LOOKUP;
                } else { /* nothing */ }
        } else { /* nothing */ }

        //ecrobot_bt_data_logger((S8)0, (S8)0);
        systick_wait_ms(50);
    }
}
