package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.ShooterSubsystem;


import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.ShooterSubsystem;

public class SetSpeakerShootingAngleCommand extends Command {
        private final ShooterSubsystem shooterSubsystem;

        public SetSpeakerShootingAngleCommand(ShooterSubsystem shooterSubsystem) {
            this.shooterSubsystem = shooterSubsystem;
        }

        @Override
        public void initialize() {

        }

        @Override
        public void execute() {
            shooterSubsystem.moveToAngle(0); //TODO set correct angle
        }

        @Override
        public void end(boolean interrupted) {
            shooterSubsystem.moveToAngle(0);
        }


}
