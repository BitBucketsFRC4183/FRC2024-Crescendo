package org.bitbuckets.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IGyro {

    Rotation2d initializationRelativeRotation();
    Rotation2d userZeroRelativeRotation();

    void userZero();

}
