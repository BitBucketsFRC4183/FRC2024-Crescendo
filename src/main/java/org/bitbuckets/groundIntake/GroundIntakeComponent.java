package org.bitbuckets.groundIntake;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;

public interface GroundIntakeComponent extends INetworkedComponent {

    @Conf("ff_ks") double ff_ks();
    @Conf("ff_kv") double ff_kv();

}
