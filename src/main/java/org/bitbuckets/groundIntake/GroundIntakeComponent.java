package org.bitbuckets.groundIntake;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;

public interface GroundIntakeComponent extends INetworkedComponent {

    @Conf("top_gear_ratio") double topGearRatio();
    @Conf("bottom_gear_ratio") double bottomGearRatio();

}
