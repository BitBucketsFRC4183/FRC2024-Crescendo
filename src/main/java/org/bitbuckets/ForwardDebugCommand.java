package org.bitbuckets;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.DriveSubsystem;

public class ForwardDebugCommand extends Command {

    final DriveSubsystem driveSubsystem;

    public ForwardDebugCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void execute() {
        driveSubsystem.driveUsingChassisSpeed(new ChassisSpeeds(1,0,0), false);
    }

    @Override
    public void end(boolean shit) {
        driveSubsystem.commandWheelsToZero();
    }
}
