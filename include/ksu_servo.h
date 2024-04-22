#ifndef SERVO_H
#define SERVO_H
#include <currentsensor.h>
#ifdef ARDUINO
#include <Arduino.h>
#endif
#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif
#include <servo.h>
const float PWM_FREQUENCY = 490;
const float max_period_us = 1 / PWM_FREQUENCY * 1000000;

class ksu_servo
{
public:
    ksu_servo(Servo *myServo, const int pwm_pin, current_sensor *current_sensor, const float lowAngle = 0, const float highAngle = 180, const float current_lim = 5.0, bool direction_flip = false) : myServo(myServo), pwmpin(pwm_pin), _current_sensor(current_sensor), lowAngle(lowAngle), highAngle(highAngle), current_lim(current_lim), direction_flip(direction_flip) {}
    void init()
    {
        pinMode(this->pwmpin, OUTPUT);
        myServo->attach(pwmpin);
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
        Serial.println(percentAngle);
        float us = 500 + percentAngle * 2000.0;
        Serial.println(us);
#ifdef ARDUINO
        myServo->writeMicroseconds(us);
#else
        printf("percentAngle %f dutyFloat %f duty %d\n", percentAngle, dutyFloat, duty);
#endif
    }
    float getCurrent()
    {
        return this->_current_sensor->getCurrent();
    }
    float getCurrentLim()
    {
        return this->current_lim;
    }
    void setCurrentLim(float limit)
    {
        current_lim = limit;
    }

private:
    Servo *myServo;
    const int pwmpin;
    current_sensor *_current_sensor;
    const float lowAngle;
    const float highAngle;
    float current_lim;
    bool direction_flip;
    const float minPercent = 0.5 / 3.0;
    const float maxPercent = 2.5 / 3.0;
    const float percentRange = maxPercent - minPercent;
    uint8_t duty;
};

class drs_system
{
public:
    drs_system(ksu_servo *left, ksu_servo *right) : left_servo(left), right_servo(right) {}
    void init()
    {
        this->left_servo->init();
        this->right_servo->init();
    }
    void setAngle(float angle)
    {
        left_servo->setAngle(angle);
        right_servo->setAngle(angle);
    }
    void updateCurrents()
    {
        this->left_servo->run();
        this->right_servo->run();
    }
    void calibrate()
    {
        float calibrationAngle = 90;
        left_servo->setAngle(calibrationAngle);
        right_servo->setAngle(calibrationAngle);

        while ((left_servo->getCurrent() < left_servo->getCurrentLim() || right_servo->getCurrent() < right_servo->getCurrentLim()) && calibrationAngle <= 180)
        {
            this->updateCurrents();
            this->setAngle(calibrationAngle);
            calibrationAngle += 0.01;
        }
        // back off like 2 degrees
        maxAngle = calibrationAngle - 2;
        calibrationAngle = 90;
        delay(1000);
        this->setAngle(calibrationAngle);
        this->updateCurrents();
        delay(1000);
        while ((left_servo->getCurrent() < left_servo->getCurrentLim() || right_servo->getCurrent() < right_servo->getCurrentLim()) && calibrationAngle >= 0)
        {
            this->updateCurrents();
            this->setAngle(calibrationAngle);
            calibrationAngle -= 0.01;
        }
        // bump up like 2 degrees
        minAngle = calibrationAngle + 2;
        Serial.print("Min angle:");
        Serial.println(minAngle);
        Serial.print("Max angle:");
        Serial.println(maxAngle);
        setAngle(90);
        delay(1000);
        setAngle(maxAngle);
        delay(1000);
        setAngle(minAngle);
        delay(1000);
    }
    void setClosed()
    {
        this->setAngle(minAngle);
    };
    void setOpen()
    {
        this->setAngle(maxAngle);
    };

private:
    ksu_servo *left_servo;
    ksu_servo *right_servo;
    float maxAngle;
    float minAngle;
};
#endif