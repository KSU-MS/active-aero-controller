#ifndef ANALOG_SENSOR_H
#define ANALOG_SENSOR_H
#include <stdint.h>

class analogSensor
{
public:
    analogSensor(int (*funcPtr)(uint8_t readPin), uint8_t pin_, const uint16_t cutoff_freq) : readFunc(funcPtr), pin(pin_)
    {
        this->alpha = 2 * 3.14 * cutoff_freq / (1 + 2 * 3.14 * cutoff_freq);
    }
    void run()
    {
        this->filtval = this->readFunc(this->pin);
        // this->filtval = this->alpha * this->filtval + (1 - this->alpha) * this->rawval;
    }
    uint16_t getValue()
    {
        return this->filtval;
    }
    float getVoltage()
    {
        return (static_cast<float>(this->filtval) / static_cast<float>(1024.0)) * 5.0;
    }
    uint8_t getPin()
    {
        return this->pin;
    }
    uint16_t filtval;

private:
    int (*readFunc)(uint8_t readPin);
    uint8_t pin;
    uint16_t rawval;
    double alpha;
};

#endif