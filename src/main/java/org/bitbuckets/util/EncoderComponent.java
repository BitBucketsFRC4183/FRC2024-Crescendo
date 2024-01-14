package org.bitbuckets.util;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;

public interface EncoderComponent extends INetworkedComponent {


    @Conf("absolute_offset") double getAbsoluteEncoderOffset();
    @Conf("coefficient") double getEncoderToMechanismCoefficient();

}
