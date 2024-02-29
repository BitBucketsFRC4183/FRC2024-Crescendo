package org.bitbuckets.util;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.loop.IMattlibHooked;
import xyz.auriium.mattlib2.utils.AngleUtil;
import xyz.auriium.yuukonstants.exception.ExplainedException;

public class ThriftyAbsoluteEncoder implements IRotationEncoder, IMattlibHooked {

    final AnalogInput input;
    final AnalogEncoderComponent encoderComponent;

    public ThriftyAbsoluteEncoder(AnalogInput input, AnalogEncoderComponent encoderComponent) {
        this.input = input;
        this.encoderComponent = encoderComponent;

        mattRegister();
    }

    double lastPosition_encoderRotations = 0;
    double offset_mechanismRotations = 0;
    double lastTime = MathSharedStore.getTimestamp();
    double dpdtApproximation = 0;

    @Override
    public ExplainedException[] verifyInit() {
        this.lastPosition_encoderRotations = angularPosition_encoderRotations();
        this.lastTime = MathSharedStore.getTimestamp();
        this.offset_mechanismRotations = encoderComponent.offset_mechanismRotations();

        return new ExplainedException[0];
    }

    @Override
    public void logicPeriodic() {
        double currentPosition = angularPosition_encoderRotations();
        double deltaPosition = currentPosition - lastPosition_encoderRotations;
        double currentTime = MathSharedStore.getTimestamp();
        double deltaTime = currentTime - lastTime;

        this.dpdtApproximation = deltaPosition / deltaTime;
        this.lastTime = currentTime;
        this.lastPosition_encoderRotations = currentPosition;
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
        double currentMechanismRotations = input.getVoltage() / RobotController.getVoltage5V() * encoderComponent.encoderToMechanismCoefficient();
        return currentMechanismRotations - encoderComponent.offset_mechanismRotations();
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
        return angularPosition_encoderRotations() * encoderComponent.encoderToMechanismCoefficient();
    }


    @Override
    public double angularVelocity_encoderRotationsPerSecond() {
        return dpdtApproximation;
    }

}
