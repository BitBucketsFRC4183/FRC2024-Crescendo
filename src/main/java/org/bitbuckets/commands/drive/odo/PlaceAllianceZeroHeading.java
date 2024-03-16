package org.bitbuckets.commands.drive.odo;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.Odometry;

/**
 * This accounts for alliance :)
 */
public class PlaceAllianceZeroHeading extends Command {

    final Odometry odometry;
    final Rotation2d heading;

    public PlaceAllianceZeroHeading(Odometry odometry, Rotation2d heading) {
        this.odometry = odometry;
        this.heading = heading;
    }

    @Override public void initialize() {
        Rotation2d headingMod = heading;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            headingMod = headingMod.plus(Rotation2d.fromDegrees(180));
        }


        odometry.forceHeading(headingMod);
    }
}
