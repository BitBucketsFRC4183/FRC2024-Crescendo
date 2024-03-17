package org.bitbuckets.commands.drive;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.drive.OperatorToSpeeds;
import org.bitbuckets.drive.DriveSubsystem;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Log;
import xyz.auriium.mattlib2.log.annote.Tune;

public class DriveFacingStaticPosCommand extends Command {


    final OperatorToSpeeds operatorToSpeeds;
    final OperatorInput operatorInput;
    final DriveSubsystem swerveSubsystem;

    final Rotation2d goal_fieldOrAlliance;
    final boolean isAllianceRelative;
    final Component component;

    Rotation2d goalHeadingMutable;

    public interface Component extends INetworkedComponent {
        @Tune("p_heading") double pHeading();

        @Log("isScheduled") void reportIsScheduled(boolean isScheduled);
        @Log("isAtGoal") void reportAtGoal(boolean atGoal);
        @Log("goal") void reportGoal(double goalDeg);
    }

    public DriveFacingStaticPosCommand(OperatorToSpeeds operatorToSpeeds, OperatorInput operatorInput, DriveSubsystem swerveSubsystem, Rotation2d goal_fieldOrAlliance, boolean isAllianceRelative, Component component) {
        this.operatorToSpeeds = operatorToSpeeds;
        this.operatorInput = operatorInput;
        this.swerveSubsystem = swerveSubsystem;
        this.goal_fieldOrAlliance = goal_fieldOrAlliance;
        this.isAllianceRelative = isAllianceRelative;


        xBad = operatorInput.driver.axisGreaterThan(XboxController.Axis.kLeftX.value, 0.1).or(operatorInput.driver.axisLessThan(XboxController.Axis.kLeftX.value, -0.1)).negate();
        yBad = operatorInput.driver.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.1).or(operatorInput.driver.axisLessThan(XboxController.Axis.kLeftY.value, -0.1)).negate();
        rotBad = operatorInput.driver.axisGreaterThan(XboxController.Axis.kRightX.value, 0.1).or(operatorInput.driver.axisLessThan(XboxController.Axis.kRightX.value, -0.1)).negate();

        this.component = component;

        addRequirements(swerveSubsystem);
    }

    final Trigger xBad;
    final Trigger yBad;
    final Trigger rotBad;

    double timeLast_seconds;


    @Override public void initialize() {


        timeLast_seconds = MathSharedStore.getTimestamp();
        boolean shouldFlip = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        goalHeadingMutable = goal_fieldOrAlliance;
        if (shouldFlip) {
            goalHeadingMutable = goalHeadingMutable.plus(Rotation2d.fromDegrees(180));
        }
        component.reportIsScheduled(true);
    }

    @Override public void execute() {
        ChassisSpeeds limitedSpeeds = operatorToSpeeds.desiredLimitedSpeeds_fieldRelative(
                swerveSubsystem.odometry.getHeading_fieldRelative()
        );

        double timeNow_seconds = MathSharedStore.getTimestamp();
        double dt_seconds = timeNow_seconds - timeLast_seconds;
        double dTheta_radianSeconds = limitedSpeeds.omegaRadiansPerSecond;

        if (xBad.getAsBoolean()
                && yBad.getAsBoolean()
                && rotBad.getAsBoolean()
        ) return;


        goalHeadingMutable = goalHeadingMutable.plus(new Rotation2d(dTheta_radianSeconds * dt_seconds));
        Rotation2d errorHeading = goalHeadingMutable.minus(swerveSubsystem.odometry.getHeading_fieldRelative());

        ChassisSpeeds speedsToUse = new ChassisSpeeds(
                limitedSpeeds.vxMetersPerSecond,
                limitedSpeeds.vyMetersPerSecond,
                errorHeading.getRadians() / dt_seconds * component.pHeading()
        );

        boolean atGoal = false;
        if (Math.abs(errorHeading.getDegrees()) < 2) {
            speedsToUse = limitedSpeeds;
            atGoal = true;
        }

        swerveSubsystem.orderToUnfiltered(speedsToUse);

        timeLast_seconds = timeNow_seconds;

        //handle logs
        component.reportGoal(goalHeadingMutable.getDegrees());
        component.reportAtGoal(atGoal);
    }

    @Override public void end(boolean interrupted) {
        component.reportIsScheduled(false);
        swerveSubsystem.orderToUnfiltered(new ChassisSpeeds(0,0,0));
    }
}
