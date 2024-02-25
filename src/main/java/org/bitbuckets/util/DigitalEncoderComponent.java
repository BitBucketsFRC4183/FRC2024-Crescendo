package org.bitbuckets.util;

import xyz.auriium.mattlib2.hardware.config.CommonMotorComponent;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;
import xyz.auriium.mattlib2.log.annote.Log;

import java.util.Optional;

public interface DigitalEncoderComponent extends INetworkedComponent {


    @Conf("dio_channelA") int dioChannelA();
    @Conf("dio_channelB") int dioChannelB();
    @Conf("encoderToMechanismCoef") double encoderToMechanismCoefficient();
    @Conf("offset") Optional<Double> offset_mechanismRotations();
    @Conf("inverted") boolean inverted();

    @Log("positionWithOffset") void logPositionWithOffset(double position_mechanismRotations);
    @Log("velocity") void logVelocity(double velocity_metersPerSecond);



}
