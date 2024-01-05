package org.bitbuckets.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import org.bitbuckets.Robot;

public class VisionSubsystem {

    static final VisionComponent COMPONENT = Robot.LOG.loadWaiting(VisionComponent.class, "vision");
    static final VisionComponent COMPONENT2 = Robot.LOG.loadWaiting(VisionComponent.class, "vision-2");

    static final int id = COMPONENT.id();

    public void init() {
        int id = COMPONENT.id();
        int result = id*4;
        COMPONENT.x_position(result);
    }

}
