package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import org.bitbuckets.drive.OdometrySubsystem;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.FlywheelSubsystem;
import xyz.auriium.mattlib2.auto.ff.FastPolynomialRegression;

import java.util.HashSet;

public class DistanceShotMakeReadyGroup extends DeferredCommand {

    final static FastPolynomialRegression fastPolynomialRegression = FastPolynomialRegression.loadFullRank(
            new double[] { //distance
                    0
            },
            new double[] { //speeds
                60
            },
            5
    );


    public DistanceShotMakeReadyGroup(FlywheelSubsystem flywheelSubsystem, OdometrySubsystem odometrySubsystem, NoteManagementSubsystem noteManagementSubsystem, GroundIntakeSubsystem groundIntakeSubsystem) {
        super(() -> {
            double flywheelSpeeds = fastPolynomialRegression.predict(odometrySubsystem.distanceFromAllianceSpeaker());

            return new FireMakeReadyGroup(flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, flywheelSpeeds);
        }, new HashSet<>());
    }
}
