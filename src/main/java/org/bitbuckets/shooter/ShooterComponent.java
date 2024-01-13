package org.bitbuckets.shooter;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;

public interface ShooterComponent extends INetworkedComponent {

   @Conf("absolute_encoder_offset") double absEncoderOffset();


}
