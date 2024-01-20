package org.bitbuckets.util;

import org.bitbuckets.disabled.DisabledILinearController;
import org.bitbuckets.disabled.DisabledILinearMotor;
import org.bitbuckets.disabled.DisabledIRotationEncoder;
import org.bitbuckets.disabled.DisabledIRotationalMotor;
import xyz.auriium.mattlib2.hardware.ILinearController;
import xyz.auriium.mattlib2.hardware.ILinearMotor;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.hardware.IRotationalMotor;

public class HardwareDisabled {

    public static ILinearMotor linearMotor_disabled() {
        return new DisabledILinearMotor();
    }

    public static IRotationalMotor rotationalMotor_disabled() {
        return new DisabledIRotationalMotor();
    }

    public static IRotationEncoder rotationEncoder_disabled() {
        return new DisabledIRotationEncoder();
    }

    public static ILinearController linearController_disabled() {
        return new DisabledILinearController();
    }

}
