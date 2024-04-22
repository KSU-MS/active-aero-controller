#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H
#include <analogsensor.h>
#ifdef ARDUINO
#include <Arduino.h>
#endif

class current_sensor
{
public:
    current_sensor(analogSensor *sensor, const float scale, const float offset = 0) : sensor(sensor), scale(scale), offset(offset) {}
    void run()
    {
        this->sensor->run();
        this->current = (sensor->getVoltage()+offset) * scale;
    }
    float getCurrent()
    {
        return this->current;
    }

private:
    analogSensor *sensor;
    const float scale;
    float current;
    float offset;
};
#endif