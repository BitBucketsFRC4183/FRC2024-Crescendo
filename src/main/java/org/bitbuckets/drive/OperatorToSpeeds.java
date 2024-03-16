package org.bitbuckets.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.bitbuckets.OperatorInput;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Tune;

public class OperatorToSpeeds {

    final OperatorInput operatorInput;
    final Component component;

    public interface Component extends INetworkedComponent {
        @Tune("normal_speed_multiplier") double normalSpeedMultiplier();
        @Tune("slow_speed_multiplier") double slowSpeedMultiplier();
        @Tune("turbo_speed_multiplier") double turboSpeedMultiplier();
        @Tune("field_oriented") boolean fieldOriented();
    }

    public OperatorToSpeeds(OperatorInput operatorInput, Component component) {
        this.operatorInput = operatorInput;
        this.component = component;
    }

    ChassisSpeeds lastSpeeds = new ChassisSpeeds();

    public ChassisSpeeds desiredLimitedSpeeds_fieldRelative(Rotation2d heading_fieldRelative) {
        double x = operatorInput.getRobotForwardComponentRaw(); //[-1, 1]
        double y = operatorInput.getDriverRightComponentRaw(); //[-1, 1]
        double theta = operatorInput.getDriverAngularComponentRaw(); //[-1, 1]


        double linearMagnitude = MathUtil.applyDeadband(
                Math.hypot(MathUtil.applyDeadband(x,0.1), MathUtil.applyDeadband(y, 0.1)), 0.05
        );
        Rotation2d linearDirection = new Rotation2d(x, y);
        linearMagnitude = linearMagnitude * linearMagnitude;

        Translation2d linearVelocity =
                new Pose2d(new Translation2d(), linearDirection)
                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                        .getTranslation();


        double speedMultiplier = component.normalSpeedMultiplier();
        if (operatorInput.turboModeHold.getAsBoolean()) speedMultiplier = component.turboSpeedMultiplier();
        if (operatorInput.slowModeHold.getAsBoolean()) speedMultiplier = component.slowSpeedMultiplier();

        ChassisSpeeds speeds_robotOriented =  new ChassisSpeeds(
                        linearVelocity.getX() * speedMultiplier, //4.5 is the experimentally determined max velocity
                        linearVelocity.getY() * speedMultiplier,
                        theta * Math.PI
        );

        if (!component.fieldOriented()) {
            return speeds_robotOriented;
        }

        //handle flipping
        Rotation2d headingForFlippingOnly = heading_fieldRelative;
        boolean shouldFlip = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        if (shouldFlip) {
            headingForFlippingOnly = headingForFlippingOnly.plus(Rotation2d.fromDegrees(180));
        }

        lastSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds_robotOriented, headingForFlippingOnly);

        return lastSpeeds;
    }

}
