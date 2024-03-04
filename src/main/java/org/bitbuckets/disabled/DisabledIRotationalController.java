package org.bitbuckets.disabled;

import xyz.auriium.mattlib2.hardware.IRotationalController;

public class DisabledIRotationalController implements IRotationalController {

    @Override
    public void setToVoltage(double voltage) {

    }

    @Override public void stopActuator() {

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

    @Override
    public void controlToNormalizedReference(double setpoint_mechanismNormalizedRotations) {

    }

    @Override public void controlToNormalizedReferenceArbitrary(double v, double v1) {

    }

    @Override
    public void controlToInfiniteReference(double setpoint_mechanismRotations) {

    }

    @Override public void controlToInfiniteReferenceArbitrary(double v, double v1) {

    }

}
