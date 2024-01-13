package org.bitbuckets.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.vision.VisionSubsystem;

public class MoveToAlignCommand extends Command {

    final DriveSubsystem driveSubsystem;
    final VisionSubsystem visionSubsystem;
    final OperatorInput operatorInput;


    public MoveToAlignCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, OperatorInput operatorInput) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
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
        super.end(interrupted);
    }
}
