package org.bitbuckets.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Subsystem;
import xyz.auriium.mattlib2.hardware.config.PIDComponent;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.loop.IMattlibHooked;
import xyz.auriium.yuukonstants.exception.ExplainedException;

/**
 * Subsystem to contain our pid controllers, because I am lazy
 */
public class AutoSubsystem implements IMattlibHooked, Subsystem {

    public final PIDController xPid_meters = new PIDController(0,0,0);
    public final PIDController yPid_meters = new PIDController(0,0,0);
    public final ProfiledPIDController thetaPid_radians =  new ProfiledPIDController(0,0,0, new TrapezoidProfile.Constraints(3,3));

    public interface Component extends INetworkedComponent {

    }

    final OdometrySubsystem odometrySubsystem;
    final PIDComponent xComponent;
    final PIDComponent yComponent;
    final PIDComponent thetaComponent;
    final Component autoComponent;

    public AutoSubsystem(OdometrySubsystem odometrySubsystem, PIDComponent xComponent, PIDComponent yComponent, PIDComponent thetaComponent, Component autoComponent) {
        this.odometrySubsystem = odometrySubsystem;
        this.xComponent = xComponent;
        this.yComponent = yComponent;
        this.thetaComponent = thetaComponent;
        this.autoComponent = autoComponent;

        register();
        mattRegister();
    }

    @Override
    public ExplainedException[] verifyInit() {

        xPid_meters.setP(xComponent.pConstant());
        xPid_meters.setI(xComponent.iConstant());
        xPid_meters.setD(xComponent.dConstant());
        yPid_meters.setP(yComponent.pConstant());
        yPid_meters.setI(yComponent.iConstant());
        yPid_meters.setD(yComponent.dConstant());
        thetaPid_radians.setP(thetaComponent.pConstant());
        thetaPid_radians.setI(thetaComponent.iConstant());
        thetaPid_radians.setD(thetaComponent.dConstant());

        thetaPid_radians.setConstraints(new TrapezoidProfile.Constraints(3,3));

        thetaPid_radians.enableContinuousInput(-Math.PI, Math.PI);
        thetaPid_radians.setTolerance(Math.PI / 360 / 4.5); //0.5 deg

        return new ExplainedException[0];
    }


    public ChassisSpeeds calculateFeedbackSpeeds(Pose2d referencePose) {

        Pose2d currentPose = odometrySubsystem.getRobotCentroidPosition();


        return new ChassisSpeeds(
                xPid_meters.calculate(currentPose.getX(), referencePose.getX()),
                yPid_meters.calculate(currentPose.getY(), referencePose.getY()),
                thetaPid_radians.calculate(currentPose.getRotation().getRadians(), referencePose.getRotation().getRadians())
        );
    }



    public boolean[] getPidAtSetpoint() {
        return new boolean[] {
                xPid_meters.atSetpoint(),
                yPid_meters.atSetpoint(),
                thetaPid_radians.atSetpoint()
        };
    }

    public boolean isXAtSetpoint() {
        return xPid_meters.atSetpoint();
    }

    public boolean isYAtSetpoint() {
        return yPid_meters.atSetpoint();
    }

    public boolean isThetaAtSetpoint() {
        return thetaPid_radians.atSetpoint();
    }

    public double hypotError() {
        return Math.hypot(xPid_meters.getPositionError(), yPid_meters.getPositionError());
    }

}
