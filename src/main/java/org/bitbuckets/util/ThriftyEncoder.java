package org.bitbuckets.util;

import edu.wpi.first.wpilibj.AnalogInput;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.utils.AngleUtil;

public class ThriftyEncoder implements IRotationEncoder {

    // double check read voltage max
    private static final double READ_VOLTAGE_MAX = 4.8;
    final AnalogInput input;
    final EncoderComponent encoderComponent;

    double currentOffset;
    public ThriftyEncoder(AnalogInput input, EncoderComponent encoderComponent) {
        this.input = input;
        this.encoderComponent = encoderComponent;

        currentOffset = encoderComponent.getAbsoluteEncoderOffset();
    }



    @Override
    public void forceRotationalOffset(double offset_mechanismRotations) {
        currentOffset = offset_mechanismRotations / encoderComponent.getEncoderToMechanismCoefficient();

    }

    @Override
    public double angularPosition_encoderRotations() {
        return (input.getVoltage()) / READ_VOLTAGE_MAX - encoderComponent.getAbsoluteEncoderOffset();
    }

    @Override
    public double angularPosition_mechanismRotations() {
        return angularPosition_encoderRotations() * encoderComponent.getEncoderToMechanismCoefficient();
    }

    @Override
    public double angularPosition_normalizedMechanismRotations() {
        return AngleUtil.normalizeRotations(angularPosition_mechanismRotations());
    }

    @Override
    public double angularPosition_normalizedEncoderRotations() {
        return AngleUtil.normalizeRotations(angularPosition_encoderRotations());
    }

    @Override
    public double angularVelocity_mechanismRotationsPerSecond() {
        throw new UnsupportedOperationException("Angular velocity for mechanism rps not finished");
    }

    @Override
    public double angularVelocity_encoderRotationsPerSecond() {
        throw new UnsupportedOperationException("Angular velocity for encoder rps not finished");
    }

}
