package org.bitbuckets.util;

import edu.wpi.first.wpilibj.AnalogInput;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;

public class HardwareUtil {

    public static IRotationEncoder thriftyEncoder(AbsoluteEncoderComponent encoderComponent) {
        AnalogInput input = new AnalogInput(encoderComponent.analogChannel());

        return new ThriftyAbsoluteEncoder(input, encoderComponent);
    }

}
