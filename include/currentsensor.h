#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H
#include <analogsensor.h>
#ifdef ARDUINO
#include <Arduino.h>
#endif

class current_sensor
{
public:
    current_sensor(analogSensor *sensor, const float scale) : sensor(sensor), scale(scale) {}
    void run()
    {
        this->sensor->run();
        this->current = static_cast<float>(sensor->getVoltage()) * scale;
    }
    float getCurrent()
    {
        return this->current;
    }

private:
    analogSensor *sensor;
    const float scale;
    float current;
};
#endif