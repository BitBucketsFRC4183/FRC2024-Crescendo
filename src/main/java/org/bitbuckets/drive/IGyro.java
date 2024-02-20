package org.bitbuckets.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface IGyro {

    Rotation2d initializationRelativeRotation();
    Rotation2d userZeroRelativeRotation();

    void userZero();
    void userForceOffset(Rotation2d beAt);
    
}
