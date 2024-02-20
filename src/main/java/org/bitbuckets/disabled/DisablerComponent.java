package org.bitbuckets.disabled;

import edu.wpi.first.math.geometry.Pose2d;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Conf;
import xyz.auriium.mattlib2.log.annote.Log;

public interface DisablerComponent extends INetworkedComponent {
    @Conf("vision_disabled") boolean vision_disabled();
    @Conf("drive_disabled") boolean drive_disabled();
    @Conf("flywheel_disabled") boolean flywheel_disabled();
    @Conf("pivot_disabled") boolean pivot_disabled();
    @Conf("odometry_disabled") boolean odometry_disabled();
    @Conf("climber_disabled") boolean climber_disabled();
    @Conf("groundIntake_disabled") boolean groundIntake_disabled();
    @Conf("nms_disabled") boolean nms_disabled();

}
