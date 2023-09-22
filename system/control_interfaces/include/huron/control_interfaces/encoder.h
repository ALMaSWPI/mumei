#pragma once

#include <math.h>
#include "generic_component.h"

namespace huron {

/**
 * Abstract class for using an encoder.
 *
 * 
 */
class Encoder : public GenericComponent {
private:
    float count_;
    float prev_count_;
    float cpr_;

public:
    Encoder();
    Encoder(const Encoder&) = delete;
    Encoder& operator=(const Encoder&) = delete;
    ~Encoder() = default;

    /**
     * Resets the encoder count.
     */
    void Reset() {
        count_ = 0.0;
        prev_count_ = 0.0;
    }

    /**
     * Gets the current encoder count.
     */
    virtual float GetCount() = 0;

    /**
     * Gets the previous encoder count.
     */
    float GetPrevCount() {
        return prev_count_;
    }

    /**
     * Gets the counts per revolution (CPR).
     */
    float GetCPR() {
        return cpr_;
    }

    /**
     * Gets the current angle in radians.
     */
    float GetAngleRadian() {
        return count_ / cpr_ * 2.0 * M_PI;
    }

    /**
     * Gets the current angle in degrees.
     */
    float GetAngleDegree() {
      return count_ / cpr_ * 360.0;
    }

    /**
     * Gets the current velocity in counts.
     */
    virtual float GetVelocity() = 0;

    /**
     * Gets the current velocity in radians.
     */
    float GetVelocityRadian() {
        return GetVelocity() / cpr_ * 2 * M_PI;
    }

    /**
     * Gets the current velocity in degrees.
     */
    float GetVelocityDegree() {
        return GetVelocity() / cpr_ * 360.0;
    }
};

}
