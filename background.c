#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#include "port_IF.h"

#include "tasksync_IF.h"

extern TASK_MES     gkTaskMes;

TASK(BackGround)
{
    int sonar;
    U32 sonar_cnt = 0;

    for(;;) {
        /* ’´‰¹”gƒZƒ“ƒT */
        sonar = ecrobot_get_sonar_sensor(PORT_SONAR);
        if (sonar < 10) {
            sonar_cnt++;
            if (sonar_cnt > 10) {
                gkTaskMes.message = MES_LOOKUP;
            } else { /* nothing */ }
        } else { /* nothing */ }

        //ecrobot_bt_data_logger((S8)0, (S8)0);
        systick_wait_ms(50);
    }
}
