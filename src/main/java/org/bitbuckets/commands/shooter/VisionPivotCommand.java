package org.bitbuckets.commands.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.OdometrySubsystem;
import org.bitbuckets.shooter.ShooterSubsystem;
import org.bitbuckets.vision.VisionSubsystem;
import org.bitbuckets.vision.VisionThing;

public class VisionPivotCommand extends Command {

    final VisionSubsystem visionSubsystem;
    final ShooterSubsystem shooterSubsystem;
    final OdometrySubsystem odometrySubsystem;

    public VisionPivotCommand(VisionSubsystem visionSubsystem, ShooterSubsystem shooterSubsystem, OdometrySubsystem odometrySubsystem) {
        this.visionSubsystem = visionSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.odometrySubsystem = odometrySubsystem;
    }

    @Override
    public void execute() {
        var opt = visionSubsystem.estimateAprilTagTargetPose();
        if (opt.isEmpty()) return; //TODO please

        Pose3d visionTargetAprilTagPose = opt.get();
        Pose3d shooterPose = odometrySubsystem.getShooterCentroidPositionVert();

        





    }


}
