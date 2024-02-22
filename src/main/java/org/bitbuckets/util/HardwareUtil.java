package org.bitbuckets.util;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;

public class HardwareUtil {

    public static IRotationEncoder thriftyEncoder(AnalogEncoderComponent encoderComponent) {
        AnalogInput input = new AnalogInput(encoderComponent.analogChannel());

        return new ThriftyAbsoluteEncoder(input, encoderComponent);
    }

    public static IRotationEncoder throughboreEncoder(DigitalEncoderComponent encoderComponent) {
        return new ThroughBoreEncoder(
                new Encoder(encoderComponent.dioChannelA(), encoderComponent.dioChannelB()),
                encoderComponent
        );
    }

}
