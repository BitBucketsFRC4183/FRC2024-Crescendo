package org.bitbuckets.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.drive.DriveSubsystem;

public class DefaultDriveCommand extends Command {

    final DriveSubsystem driveSubsystem;
    final OperatorInput operatorInput;

    public DefaultDriveCommand(DriveSubsystem driveSubsystem, OperatorInput operatorInput) {
        this.driveSubsystem = driveSubsystem;
        this.operatorInput = operatorInput;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.commandWheelsToZero();
    }
}
