package org.bitbuckets.util;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import xyz.auriium.mattlib2.IPeriodicLooped;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.utils.AngleUtil;

public class ThriftyAbsoluteEncoder implements IRotationEncoder, IPeriodicLooped {

    final AnalogInput input;
    final AbsoluteEncoderComponent encoderComponent;

    double offset_mechanismRotations;

    public ThriftyAbsoluteEncoder(AnalogInput input, AbsoluteEncoderComponent encoderComponent) {
        this.input = input;
        this.encoderComponent = encoderComponent;

        offset_mechanismRotations = encoderComponent.offset_mechanismRotations();
        mattRegister();
    }


    @Override
    public void logPeriodic() {
        encoderComponent.logPositionWithOffset(angularPosition_mechanismRotations());
    }

    @Override
    public void forceRotationalOffset(double offset_mechanismRotations) {
        this.offset_mechanismRotations = offset_mechanismRotations;
    }

    @Override
    public double angularPosition_encoderRotations() {
        return angularPosition_mechanismRotations() / encoderComponent.encoderToMechanismCoefficient();
    }

    @Override
    public double angularPosition_mechanismRotations() {
        double currentMechanismRotations = input.getAverageVoltage() / RobotController.getVoltage5V() * encoderComponent.encoderToMechanismCoefficient();
        return currentMechanismRotations - encoderComponent.offset_mechanismRotations() - 0.5;
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
