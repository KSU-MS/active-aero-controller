#ifndef SERVO_H
#define SERVO_H
#include <currentsensor.h>
#ifdef ARDUINO
#include <Arduino.h>
#endif
#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif
class servo
{
public:
    servo(const int pwm_pin, current_sensor* current_sensor, const float lowAngle = 0, const float highAngle = 180, const float current_lim = 5.0, bool direction_flip = false) : pwmpin(pwm_pin), _current_sensor(current_sensor), lowAngle(lowAngle), highAngle(highAngle), current_lim(current_lim), direction_flip(direction_flip) {}
    void init()
    {
        setAngle(90);
    }
    void run()
    {
        _current_sensor->run();
    }
    void setAngle(float angle)
    {
        constrain(angle, lowAngle, highAngle);
        // calculate needed duty cycle
        float percentAngle = angle / highAngle; // gives us the percent we are aiming for
        if (direction_flip)
        {
            percentAngle = (float)1.0 - percentAngle;
        }
        float dutyFloat = (percentAngle * percentRange) + minPercent;
        constrain(dutyFloat, minPercent, maxPercent);
        duty = round(dutyFloat * 255.0);
#ifdef ARDUINO
        analogWrite(pwmpin, duty);
#elif
        printf("percentAngle %f dutyFloat %f duty %d\n", percentAngle, dutyFloat, duty);
#endif
    }

private:
    const int pwmpin;
    current_sensor* _current_sensor;
    const float lowAngle;
    const float highAngle;
    const float current_lim;
    bool direction_flip;
    const float minPercent = 0.5 / 3.0;
    const float maxPercent = 2.5 / 3.0;
    const float percentRange = maxPercent - minPercent;
    uint8_t duty;
};
#endif