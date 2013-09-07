#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h"

#include "port_IF.h"
#include "main_IF.h"
#include "lookup_IF.h"

DeclareCounter(SysTimerCnt);

/* nxtOSEK hooks */
void ecrobot_device_initialize(void)
{
    nxt_motor_set_speed(PORT_MOTOR_L, 0, 1);
    nxt_motor_set_speed(PORT_MOTOR_R, 0, 1);
    balance_init();
    nxt_motor_set_count(PORT_MOTOR_L, 0);
    nxt_motor_set_count(PORT_MOTOR_R, 0);
    ecrobot_init_sonar_sensor(PORT_SONAR);
    ecrobot_set_light_sensor_active(PORT_LIGHT);

    /* ÉçÉO */
    if (ecrobot_get_bt_status() == BT_NO_INIT) {
        ecrobot_set_bt_device_name("ETROBOLT255");
    } else { /* NOTHING */ }

    ecrobot_init_bt_slave("LEJOS_OSEK");

    lt_ini();
    lookup_ini();
}

void ecrobot_device_terminate(void)
{
    nxt_motor_set_speed(PORT_MOTOR_L, 0, 1);
    nxt_motor_set_speed(PORT_MOTOR_R, 0, 1);
    ecrobot_term_sonar_sensor(PORT_SONAR);
}

/* nxtOSEK hook to be invoked from an ISR in category 2 */
void user_1ms_isr_type2(void)
{
    StatusType ercd;

    ercd = SignalCounter(SysTimerCnt); /* Increment OSEK Alarm Counter */
    if (ercd != E_OK) {
        ShutdownOS(ercd);
    }
}
