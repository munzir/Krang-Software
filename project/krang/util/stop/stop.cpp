#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>
#include <unistd.h>

somatic_d_t daemon_cx;

int main(int argc, char* argv[]) {
    somatic_motor_t rlwa;
    somatic_motor_t llwa;
    somatic_motor_t waist;
    somatic_motor_t torso;

    // init daemon
    somatic_d_opts_t daemon_opt;
    memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
    daemon_opt.ident = "stop";
    somatic_d_init(&daemon_cx, &daemon_opt);

    // init waist, torso, and arms
    somatic_motor_init(&daemon_cx, &rlwa, 7, "rlwa-cmd", "rlwa-state");
    somatic_motor_init(&daemon_cx, &llwa, 7, "llwa-cmd", "llwa-state");
    
    // open up the motors
    somatic_motor_reset(&daemon_cx, &llwa);
    somatic_motor_reset(&daemon_cx, &rlwa);
    
    // set larm joint limits
    double** larm_minimum_values[] = { &llwa.pos_valid_min, &llwa.vel_valid_min,
                                       &llwa.pos_limit_min, &llwa.vel_limit_min };
    double** larm_maximum_values[] = { &llwa.pos_valid_max, &llwa.vel_valid_max,
                                       &llwa.pos_limit_max, &llwa.vel_limit_max };
    for(size_t i = 0; i < 4; i++) aa_fset(*larm_minimum_values[i], -1024.1, 7);
    for(size_t i = 0; i < 4; i++) aa_fset(*larm_maximum_values[i], 1024.1, 7);
    
    // set rarm joint limits
    double** rarm_minimum_values[] = { &rlwa.pos_valid_min, &rlwa.vel_valid_min,
                                       &rlwa.pos_limit_min, &rlwa.vel_limit_min };
    double** rarm_maximum_values[] = { &rlwa.pos_valid_max, &rlwa.vel_valid_max,
                                       &rlwa.pos_limit_max, &rlwa.vel_limit_max };
    for(size_t i = 0; i < 4; i++) aa_fset(*rarm_minimum_values[i], -1024.1, 7);
    for(size_t i = 0; i < 4; i++) aa_fset(*rarm_maximum_values[i], 1024.1, 7);

    // start the daemon running
    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

    // update motors again
    somatic_motor_update(&daemon_cx, &llwa);
    somatic_motor_update(&daemon_cx, &rlwa);

    // wait a bit
    usleep(1e5);

    // send a zero velocity command just to be safe
    double qzero[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    somatic_motor_setvel(&daemon_cx, &llwa, qzero, 7);
    somatic_motor_setvel(&daemon_cx, &rlwa, qzero, 7);
    
    // and send halts
    somatic_motor_halt(&daemon_cx, &llwa);
    somatic_motor_halt(&daemon_cx, &rlwa);

    // Send stopping event
    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

    // close the motors
    somatic_motor_destroy(&daemon_cx, &llwa);
    somatic_motor_destroy(&daemon_cx, &rlwa);

    // Destroy the daemon resources
    somatic_d_destroy(&daemon_cx);

    // and we're done
    exit(0);
}
