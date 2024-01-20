package org.bitbuckets.groundIntake;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;

public interface GroundIntakeComponent extends INetworkedComponent {

    @Conf("top_gear_ratio") double topGearRatio();
    @Conf("bottom_gear_ratio") double bottomGearRatio();

    @Conf("ff_ks") double ff_ks();
    @Conf("ff_kv") double ff_kv();

}
