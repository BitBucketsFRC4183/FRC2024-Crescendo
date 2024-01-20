package org.bitbuckets.disabled;

import xyz.auriium.mattlib2.hardware.IRotationalMotor;

public class DisabledIRotationalMotor implements IRotationalMotor {

    @Override
    public void setToVoltage(double voltage) {

    }

    @Override
    public void setToPercent(double percent_zeroToOne) {

    }

    @Override
    public double reportCurrentNow_amps() {
        return 0;
    }

    @Override
    public double reportVoltageNow() {
        return 0;
    }

    @Override
    public double reportTemperatureNow() {
        return 0;
    }

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
