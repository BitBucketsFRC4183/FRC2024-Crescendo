package org.bitbuckets.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IGyro {

    Rotation2d rotation_initializationRelative();
    boolean isCurrentlyAlive();
    
}
