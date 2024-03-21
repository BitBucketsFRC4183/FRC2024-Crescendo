package org.bitbuckets.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.bitbuckets.commands.drive.SitFacingAutoCommand;
import org.bitbuckets.commands.drive.SitFacingCommand;
import org.bitbuckets.commands.drive.SitFacingPositionCommand;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.FlywheelSubsystem;
import org.bitbuckets.util.FieldConstants;
import xyz.auriium.mattlib2.hardware.config.PIDComponent;

public class DistanceShotMakeReadyGroup extends SequentialCommandGroup {

    public DistanceShotMakeReadyGroup(PIDComponent component, DriveSubsystem driveSubsystem, FlywheelSubsystem flywheelSubsystem, NoteManagementSubsystem noteManagementSubsystem, GroundIntakeSubsystem groundIntakeSubsystem) {
        super(
                new SitFacingPositionCommand(component, driveSubsystem, FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d().minus(new Translation2d(0.4, 0)), true),
                new DistanceShot(flywheelSubsystem, driveSubsystem.odometry, noteManagementSubsystem, groundIntakeSubsystem)
        );
    }

}
