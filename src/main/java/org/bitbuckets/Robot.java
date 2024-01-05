package org.bitbuckets;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.TimedRobot;
import org.bitbuckets.vision.VisionSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import xyz.auriium.mattlib2.log.MattLog;

public class Robot extends TimedRobot {

    public static final MattLog LOG = new MattLog(null);


    @Override
    public void robotInit() {

        LOG.init();
    }
}
