package org.bitbuckets.util;

import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;

public interface DisablerComponent extends INetworkedComponent {
    @Conf("vision_disabled") Boolean vision_disabled();
    @Conf("drive_disabled") Boolean drive_disabled();
    @Conf("shooter_disabled") Boolean shooter_disabled();
    @Conf("odometry_disabled") Boolean odometry_disabled();
    @Conf("climber_disabled") Boolean climber_disabled();
    @Conf("groundIntake_disabled") Boolean groundIntake_disabled();
}
