package org.bitbuckets.util;

import xyz.auriium.mattlib2.hardware.config.CommonMotorComponent;
import xyz.auriium.mattlib2.hardware.config.CommonPIDComponent;
import xyz.auriium.mattlib2.hardware.config.IndividualPIDComponent;
import xyz.auriium.mattlib2.hardware.config.PIDComponent;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;

public interface CommonEncoderComponent extends INetworkedComponent {

    @Conf("analogChannel") int analogChannel();
    @Conf("absoluteOffset") double getAbsoluteEncoderOffset();
    @Conf("coefficient") double getEncoderToMechanismCoefficient();



}
