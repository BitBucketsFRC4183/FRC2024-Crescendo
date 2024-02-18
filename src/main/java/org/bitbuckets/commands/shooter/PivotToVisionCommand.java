package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.OdometrySubsystem;
import org.bitbuckets.shooter.ShooterSubsystem;
import org.bitbuckets.vision.VisionSubsystem;

public class PivotToVisionCommand extends Command {

    final VisionSubsystem visionSubsystem;
    final ShooterSubsystem shooterSubsystem;
    final OdometrySubsystem odometrySubsystem;

    public PivotToVisionCommand(VisionSubsystem visionSubsystem, ShooterSubsystem shooterSubsystem, OdometrySubsystem odometrySubsystem) {
        this.visionSubsystem = visionSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.odometrySubsystem = odometrySubsystem;
    }

    @Override
    public void execute() {
        // this shit idk how this works so uh comment bye bye
//        var opt = visionSubsystem.getDesiredTargetAlignPose();
//        if (opt.isEmpty()) return; //TODO please
//
//        Pose3d visionTargetAprilTagPose = opt.get();
//        Pose3d shooterPose = odometrySubsystem.getShooterCentroidPositionVert();

        





    }


}
