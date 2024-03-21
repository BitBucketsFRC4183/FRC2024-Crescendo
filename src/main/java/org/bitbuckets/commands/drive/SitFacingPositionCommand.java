package org.bitbuckets.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.util.FieldConstants;
import xyz.auriium.mattlib2.hardware.config.PIDComponent;

public class SitFacingPositionCommand extends Command {

    final PIDComponent pidBrain;
    final DriveSubsystem swerveSubsystem;
    final boolean isAllianceRelative;

    final PIDController rotationalController = new PIDController(0.02,0,0);


    final Translation2d seedPos;
    Translation2d desiredPos;

    public SitFacingPositionCommand(PIDComponent pidBrain, DriveSubsystem swerveSubsystem, Translation2d seedPos, boolean isAllianceRelative) {
        this.pidBrain = pidBrain;
        this.swerveSubsystem = swerveSubsystem;
        this.seedPos = seedPos;
        this.isAllianceRelative = isAllianceRelative;

        addRequirements(swerveSubsystem);
    }

    @Override public void initialize() {
        rotationalController.reset();
        rotationalController.setPID(1.5, pidBrain.iConstant(), 0.02);
        rotationalController.enableContinuousInput(-Math.PI, Math.PI);
        rotationalController.setTolerance(Math.PI / 45);
        desiredPos = seedPos;


        if (isAllianceRelative) {
            boolean shouldFlip = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
            if (shouldFlip) {
                desiredPos = new Translation2d(
                        FieldConstants.fieldLength - desiredPos.getX(),
                        desiredPos.getY()
                );
            }
        }
    }

    @Override public void execute() {
        Translation2d delta = desiredPos.minus(swerveSubsystem.odometry.getRobotCentroidPosition().getTranslation());
        Rotation2d goal = new Rotation2d( Math.atan2(delta.getY(), delta.getX()) );
        System.out.println(goal + " IS GOAL");


        double controlOut = rotationalController
                .calculate(
                        goal.getRadians(),
                        swerveSubsystem.odometry.getHeading_fieldRelative().getRadians()
                );


        swerveSubsystem.orderToUnfiltered(new ChassisSpeeds(0,0,controlOut));
    }


    @Override public boolean isFinished() {
        Translation2d delta = desiredPos.minus(swerveSubsystem.odometry.getRobotCentroidPosition().getTranslation());
        Rotation2d goal = new Rotation2d( Math.atan2(delta.getY(), delta.getX()) );
        Rotation2d deltaError = goal.minus(swerveSubsystem.odometry.getHeading_fieldRelative());
        //TODO stupid hack

        double appendStuff = 0;

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            appendStuff = 180;
        }

        var num = Math.abs(deltaError.getDegrees() + appendStuff);

        System.out.println("goal is: " + goal + " and angle is: " + swerveSubsystem.odometry.getHeading_fieldRelative().getRadians() + " error is: " + num);



        return  num < 4;
    }

    @Override public void end(boolean interrupted) {
        System.out.println("STOP NOW FUCKING IDIOT");
        swerveSubsystem.orderToZero();
    }
}
