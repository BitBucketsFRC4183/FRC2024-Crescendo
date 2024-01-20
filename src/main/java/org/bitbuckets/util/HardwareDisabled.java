package org.bitbuckets.util;

import xyz.auriium.mattlib2.hardware.ILinearMotor;

public class HardwareDisabled {

    public static ILinearMotor linearMotor_disabled() {
        return new DisabledILinearMotor();
    }

}
