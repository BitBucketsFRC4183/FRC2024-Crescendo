package org.bitbuckets.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import org.bitbuckets.drive.DriveSubsystem;

public class ReadyForwardAuto extends ParallelDeadlineGroup {

    public ReadyForwardAuto(DriveSubsystem swerveSubsystem) {
        super(
                Commands.waitSeconds(0.1),
                Commands.runEnd(
                        () -> swerveSubsystem.orderToHeadingOnly(new ChassisSpeeds(1,0,0)),
                        swerveSubsystem::orderToZero
                )
        );
    }

}
