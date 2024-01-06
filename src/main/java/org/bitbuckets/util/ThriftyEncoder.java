package org.bitbuckets.util;

import xyz.auriium.mattlib2.hardware.IRotationEncoder;

public class ThriftyEncoder implements IRotationEncoder {
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
    public double angularvelocity_encoderRotationsPerSecond() {
        return 0;
    }
}
