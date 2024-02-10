package org.bitbuckets.disabled;

import xyz.auriium.mattlib2.hardware.ILinearVelocityController;

public class DisabledLinearVelocityController implements ILinearVelocityController {
    @Override public void controlToLinearVelocityReferenceArbitrary(double v, double v1) {

    }

    @Override public void setToVoltage(double v) {

    }

    @Override public void setToPercent(double v) {

    }

    @Override public double reportCurrentNow_amps() {
        return 0;
    }

    @Override public double reportVoltageNow() {
        return 0;
    }

    @Override public double reportTemperatureNow() {
        return 0;
    }

    @Override public void forceLinearOffset(double v) {

    }

    @Override public double linearPosition_mechanismMeters() {
        return 0;
    }

    @Override public double linearVelocity_mechanismMetersPerSecond() {
        return 0;
    }
}
