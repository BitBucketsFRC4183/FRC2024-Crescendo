package org.bitbuckets;

import org.bitbuckets.drive.DrivebaseComponent;
import org.bitbuckets.util.Util;
import org.bitbuckets.vision.VisionComponent;
import xyz.auriium.mattlib2.IMattLog;
import xyz.auriium.mattlib2.Mattlib;
import xyz.auriium.mattlib2.MattlibLooper;
import xyz.auriium.mattlib2.hardware.config.*;

import static xyz.auriium.mattlib2.Mattlib.LOG;

/**
 * This looks like an interface, but we are not using it as one. Rather, we are abusing the fact that
 * any variable typed into an interface automatically becomes public static final.
 *
 * This class holds all of the relevant mattlib components that we need to know about
 */
public interface IO {

    VisionComponent VISION = LOG.load(VisionComponent.class, "vision");

    //Drive's components
    DrivebaseComponent SWERVE = LOG.load(DrivebaseComponent.class, "swerve");
    MotorComponent[] DRIVES = MotorComponent.ofRange(
            LOG.load(CommonMotorComponent.class, "swerve/drive"),
            LOG.loadRange(IndividualMotorComponent.class, "swerve/drive", 4, Util.RENAMER)
    );
    MotorComponent[] STEERS = MotorComponent.ofRange(
            LOG.load(CommonMotorComponent.class, "swerve/steer"),
            LOG.loadRange(IndividualMotorComponent.class, "swerve/steer", 4, Util.RENAMER)
    );
    PIDComponent[] PIDS = PIDComponent.ofRange(
            LOG.load(CommonPIDComponent.class, "swerve/pid"),
            LOG.loadRange(IndividualPIDComponent.class, "swerve/pid", 4, Util.RENAMER)
    );


}
