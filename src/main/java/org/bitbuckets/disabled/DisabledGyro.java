package org.bitbuckets.disabled;

import edu.wpi.first.math.geometry.Rotation2d;
import org.bitbuckets.drive.IGyro;

public class DisabledGyro implements IGyro {

    @Override
    public Rotation2d initializationRelativeRotation() {
        return new Rotation2d();
    }

    @Override
    public Rotation2d userZeroRelativeRotation() {
        return new Rotation2d();
    }

    @Override
    public void userZero() {

    }
}
