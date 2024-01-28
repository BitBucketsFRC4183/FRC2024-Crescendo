package org.bitbuckets.commands.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.SwerveComponent;

/**
 * Using SwerveMAX code to be better
 */
public class AugmentedDriveCommand extends Command {

    final OperatorInput operatorInput;
    final DriveSubsystem driveSubsystem;
    final SwerveComponent swerveComponent;

    final SlewRateLimiter magnitudeLimit;
    final SlewRateLimiter rotationalLimit;

    //state
    double currentTranslationMagnitude = 0;
    double currentPolar_radians = 0;
    double lastTime = WPIUtilJNI.now() * 1e-6;

    public AugmentedDriveCommand(OperatorInput operatorInput, DriveSubsystem driveSubsystem, SwerveComponent swerveComponent) {
        this.operatorInput = operatorInput;
        this.driveSubsystem = driveSubsystem;
        this.swerveComponent = swerveComponent;
        magnitudeLimit = new SlewRateLimiter(swerveComponent.magnitudeSlew());
        rotationalLimit = new SlewRateLimiter(swerveComponent.rotationalSlew());
    }

    @Override
    public void execute() {

        double fieldX = 3d * operatorInput.getRobotForwardComponent(); //-3 to 3 m/s
        double fieldY = 3d * operatorInput.getDriverRightComponent();
        double rot_radians = 2d * operatorInput.getDriverRightStickX();

        // Convert XY to polar for rate limiting
        double polar_radians = Math.atan2(fieldY, fieldX);
        double translationalMagnitude = Math.sqrt(Math.pow(fieldX, 2) + Math.pow(fieldY, 2));

        // Calculate the direction slew rate based on an estimate of the lateral acceleration
        double directionSlewRate;

        if (translationalMagnitude != 0.0) {
            directionSlewRate = Math.abs(swerveComponent.magnitudeSlew() / currentTranslationMagnitude);
        } else {
            directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
        }


        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - lastTime;
        double angleDif = AngleDifference(polar_radians, polar_radians);

        if (angleDif < 0.45*Math.PI) {
            currentPolar_radians = StepTowardsCircular(currentTranslationMagnitude, translationalMagnitude, directionSlewRate * elapsedTime);
            currentTranslationMagnitude = magnitudeLimit.calculate(translationalMagnitude);
        }
        else if (angleDif > 0.85*Math.PI) {
            if (currentTranslationMagnitude > 1e-4) { //some small number to avoid floating-point errors with equality checking
                // keep currentTranslationDir unchanged
                currentTranslationMagnitude = magnitudeLimit.calculate(0.0);
            }
            else {
                currentPolar_radians = WrapAngle(currentPolar_radians + Math.PI);
                currentTranslationMagnitude = magnitudeLimit.calculate(translationalMagnitude);
            }
        }
        else {
            currentPolar_radians = StepTowardsCircular(currentPolar_radians, polar_radians, directionSlewRate * elapsedTime);
            currentTranslationMagnitude = magnitudeLimit.calculate(0.0);
        }

        lastTime = currentTime;

        double xSpeedCommanded = currentTranslationMagnitude * Math.cos(currentPolar_radians);
        double ySpeedCommanded = currentTranslationMagnitude * Math.sin(currentPolar_radians);
        double rotationCommanded = rotationalLimit.calculate(rot_radians);



    }

    /**
     * Steps a value towards a target with a specified step size.
     * @param _current The current or starting value.  Can be positive or negative.
     * @param _target The target value the algorithm will step towards.  Can be positive or negative.
     * @param _stepsize The maximum step size that can be taken.
     * @return The new value for {@code _current} after performing the specified step towards the specified target.
     */
    public static double StepTowards(double _current, double _target, double _stepsize) {
        if (Math.abs(_current - _target) <= _stepsize) {
            return _target;
        }
        else if (_target < _current) {
            return _current - _stepsize;
        }
        else {
            return _current + _stepsize;
        }
    }

    /**
     * Steps a value (angle) towards a target (angle) taking the shortest path with a specified step size.
     * @param _current The current or starting angle (in radians).  Can lie outside the 0 to 2*PI range.
     * @param _target The target angle (in radians) the algorithm will step towards.  Can lie outside the 0 to 2*PI range.
     * @param _stepsize The maximum step size that can be taken (in radians).
     * @return The new angle (in radians) for {@code _current} after performing the specified step towards the specified target.
     * This value will always lie in the range 0 to 2*PI (exclusive).
     */
    public static double StepTowardsCircular(double _current, double _target, double _stepsize) {
        _current = WrapAngle(_current);
        _target = WrapAngle(_target);

        double stepDirection = Math.signum(_target - _current);
        double difference = Math.abs(_current - _target);

        if (difference <= _stepsize) {
            return _target;
        }
        else if (difference > Math.PI) { //does the system need to wrap over eventually?
            //handle the special case where you can reach the target in one step while also wrapping
            if (_current + 2*Math.PI - _target < _stepsize || _target + 2*Math.PI - _current < _stepsize) {
                return _target;
            }
            else {
                return WrapAngle(_current - stepDirection * _stepsize); //this will handle wrapping gracefully
            }

        }
        else {
            return _current + stepDirection * _stepsize;
        }
    }

    /**
     * Finds the (unsigned) minimum difference between two angles including calculating across 0.
     * @param _angleA An angle (in radians).
     * @param _angleB An angle (in radians).
     * @return The (unsigned) minimum difference between the two angles (in radians).
     */
    public static double AngleDifference(double _angleA, double _angleB) {
        double difference = Math.abs(_angleA - _angleB);
        return difference > Math.PI? (2 * Math.PI) - difference : difference;
    }

    /**
     * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
     * @param _angle The angle (in radians) to wrap.  Can be positive or negative and can lie multiple wraps outside the output range.
     * @return An angle (in radians) from 0 and 2*PI (exclusive).
     */
    public static double WrapAngle(double _angle) {
        double twoPi = 2*Math.PI;

        if (_angle == twoPi) { // Handle this case separately to avoid floating point errors with the floor after the division in the case below
            return 0.0;
        }
        else if (_angle > twoPi) {
            return _angle - twoPi*Math.floor(_angle / twoPi);
        }
        else if (_angle < 0.0) {
            return _angle + twoPi*(Math.floor((-_angle) / twoPi)+1);
        }
        else {
            return _angle;
        }
    }
}
