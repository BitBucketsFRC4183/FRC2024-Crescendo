package org.bitbuckets.util;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;
import xyz.auriium.mattlib2.log.annote.Log;

public interface AnalogEncoderComponent extends INetworkedComponent {

    @Conf("analog_channel") int analogChannel();
    @Conf("offset") double offset_mechanismRotations();
    @Conf("encoderToMechanismCoef") double encoderToMechanismCoefficient();


    @Log("positionWithOffset") void logPositionWithOffset(double position_mechanismRotations);
    @Log("velocity") void logVelocity(double velocity_metersPerSecond);



}
