package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import org.bitbuckets.drive.Odometry;

import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.FlywheelSubsystem;
import xyz.auriium.mattlib2.auto.ff.FastPolynomialRegression;


import java.util.HashSet;

public class DistanceShot extends DeferredCommand {


    final static FastPolynomialRegression fastPolynomialRegression = FastPolynomialRegression.loadRankDeficient_iterative(
            new double[] { //distance
                    1.11,
                    1.2,
                    1.3,
                    1.4,
                    1.5,
                    1.6

            }, new double[] { //speeds
                60,
                60,
                    35,
                    32.5,
                    27.5,
                    26.8

            },
            40
    );



    public DistanceShot(FlywheelSubsystem flywheelSubsystem, Odometry odometrySubsystem, NoteManagementSubsystem noteManagementSubsystem, GroundIntakeSubsystem groundIntakeSubsystem) {
        super(() -> {
            double flywheelSpeeds = fastPolynomialRegression.predict(odometrySubsystem.distanceFromAllianceSpeaker());

            return new FireMakeReadyGroup(flywheelSubsystem, noteManagementSubsystem, groundIntakeSubsystem, () -> flywheelSpeeds);
        }, new HashSet<>());
    }
}
