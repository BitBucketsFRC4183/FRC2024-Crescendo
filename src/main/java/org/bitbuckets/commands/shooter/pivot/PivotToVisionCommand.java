package org.bitbuckets.commands.shooter.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.OdometrySubsystem;
import org.bitbuckets.shooter.FlywheelSubsystem;
import org.bitbuckets.vision.VisionSubsystem;

public class PivotToVisionCommand extends Command {

    final VisionSubsystem visionSubsystem;
    final FlywheelSubsystem flywheelSubsystem;
    final OdometrySubsystem odometrySubsystem;

    public PivotToVisionCommand(VisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, OdometrySubsystem odometrySubsystem) {
        this.visionSubsystem = visionSubsystem;
        this.flywheelSubsystem = flywheelSubsystem;
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
