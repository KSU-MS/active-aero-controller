#ifndef CAR_H
#define CAR_H
struct driver_inputs_t
{
    double torque_command=0;
    double steering_angle=0;
    double bse_voltage=0;
    double apps1_travel=0;
    double apps2_travel=0;
    double bse1_travel=0;
};

driver_inputs_t driver_inputs;

#endif