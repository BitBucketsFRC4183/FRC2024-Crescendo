package org.bitbuckets.disabled;

import xyz.auriium.mattlib2.hardware.IRotationEncoder;

public class DisabledIRotationEncoder implements IRotationEncoder {
    @Override
    public void forceRotationalOffset(double offset_mechanismRotations) {

    }

    @Override
    public double angularPosition_encoderRotations() {
        return 0;
    }

    @Override
    public double angularPosition_mechanismRotations() {
        return 0;
    }

    @Override
    public double angularPosition_normalizedMechanismRotations() {
        return 0;
    }

    @Override
    public double angularPosition_normalizedEncoderRotations() {
        return 0;
    }

    @Override
    public double angularVelocity_mechanismRotationsPerSecond() {
        return 0;
    }

    @Override
    public double angularVelocity_encoderRotationsPerSecond() {
        return 0;
    }
}
