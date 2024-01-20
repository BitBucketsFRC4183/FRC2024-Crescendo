package org.bitbuckets.disabled;

import xyz.auriium.mattlib2.hardware.ILinearController;

public class DisabledILinearController implements ILinearController {
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
    public void forceLinearOffset(double linearOffset_mechanismMeters) {

    }

    @Override
    public double linearPosition_mechanismMeters() {
        return 0;
    }

    @Override
    public double linearVelocity_mechanismMetersPerSecond() {
        return 0;
    }

    @Override
    public void controlToLinearReference(double setpointMechanism_meters) {

    }

    @Override
    public void controlToLinearReference(double setpointMechanism_meters, double measurementMechanism_meters) {

    }
}
