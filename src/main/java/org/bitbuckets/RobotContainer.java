package org.bitbuckets;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.bitbuckets.climber.ClimberComponent;
import org.bitbuckets.climber.ClimberSubsystem;
import org.bitbuckets.commands.climber.MoveClimberCommand;
import org.bitbuckets.commands.drive.DefaultDriveCommand;
import org.bitbuckets.commands.drive.FollowTrajectoryCommand;
import org.bitbuckets.commands.drive.MoveToAlignCommand;
import org.bitbuckets.commands.groundIntake.GroundIntakeCommand;
import org.bitbuckets.commands.groundIntake.GroundOuttakeCommand;
import org.bitbuckets.commands.shooter.*;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.DrivebaseComponent;
import org.bitbuckets.drive.OdometrySubsystem;
import org.bitbuckets.drive.SwerveModule;
import org.bitbuckets.groundIntake.GroundIntakeComponent;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.shooter.ShooterComponent;
import org.bitbuckets.shooter.ShooterSubsystem;
import org.bitbuckets.util.DisablerComponent;
import org.bitbuckets.util.EncoderComponent;
import org.bitbuckets.util.ThriftyAbsoluteEncoder;
import org.bitbuckets.util.Util;
import org.bitbuckets.vision.CamerasComponent;
import org.bitbuckets.vision.VisionComponent;
import org.bitbuckets.vision.VisionSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import xyz.auriium.mattlib2.CTowerCommands;
import xyz.auriium.mattlib2.Mattlib;
import xyz.auriium.mattlib2.MattlibSettings;
import xyz.auriium.mattlib2.auto.controls.ff.FFGenComponent;
import xyz.auriium.mattlib2.auto.controls.ff.LinearFFGenRoutine;
import xyz.auriium.mattlib2.hardware.*;
import xyz.auriium.mattlib2.hardware.config.*;
import xyz.auriium.mattlib2.rev.HardwareREV;
import xyz.auriium.mattlib2.sim.HardwareSIM;
import xyz.auriium.mattlib2.sim.SimComponent;
import xyz.auriium.mattlib2.utils.MockingUtil;

import java.io.IOException;

import static xyz.auriium.mattlib2.Mattlib.LOG;

public class RobotContainer {


    public final DriveSubsystem driveSubsystem;
    public final OperatorInput operatorInput;
    public final ShooterSubsystem shooterSubsystem;
    public final OdometrySubsystem odometrySubsystem;
    public final VisionSubsystem visionSubsystem;
    public final ClimberSubsystem climberSubsystem;
    public final GroundIntakeSubsystem groundIntakeSubsystem;
    public final SwerveDriveKinematics kinematics;

    public final MockCamera mockCamera;

    public RobotContainer() {

        CommandScheduler.getInstance().enable();

        //mattlib stuff
        Mattlib.LOOPER.runPreInit();
        Mattlib.LOOPER.runPostInit();
        MattlibSettings.USE_LOGGING = true;

        System.out.println(DRIVE.ff_kv());

        this.operatorInput = new OperatorInput();
        this.kinematics = loadKinematics();
        this.driveSubsystem = loadDriveSubsystem();
        this.shooterSubsystem = loadShooterSubsystem();
        this.visionSubsystem = loadVisionSubsystem();
        this.odometrySubsystem = loadOdometrySubsystem();
        this.climberSubsystem = loadClimberSubsystem();
        this.groundIntakeSubsystem = loadGroundIntakeSubsystem();
        this.mockCamera = loadMockCamera();

        loadCommands();
    }



    public void autonomousInit() {

        ChoreoTrajectory trajectory = Choreo.getTrajectory("MVPTaxi");
        HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
                new PIDController(DRIVE_X_PID.pConstant(),DRIVE_X_PID.iConstant(),DRIVE_X_PID.dConstant()),
                new PIDController(DRIVE_Y_PID.pConstant(), DRIVE_Y_PID.iConstant(), DRIVE_Y_PID.dConstant()),
                new ProfiledPIDController(DRIVE_T_PID.pConstant(), DRIVE_T_PID.iConstant(), DRIVE_T_PID.dConstant(),
                        new TrapezoidProfile.Constraints(1,2)) //TODO
        );

        new SequentialCommandGroup(
                new SetSpeakerShootingAngleCommand(shooterSubsystem),
                new ShootNoteCommand(shooterSubsystem),
                new FollowTrajectoryCommand(trajectory, driveSubsystem, odometrySubsystem, holonomicDriveController)
        ).schedule();

    }

    public void testInit() {

        LinearFFGenRoutine groundTopFFRoutine = new LinearFFGenRoutine(TOP_GROUND_FFGEN, groundIntakeSubsystem.topMotor, groundIntakeSubsystem.topMotor);
        //LinearFFGenRoutine groundBottomFFRoutine = new LinearFFGenRoutine(BOTTOM_GROUND_FFGEN, groundIntakeSubsystem.bottomMotor, groundIntakeSubsystem.bottomMotor);
        CTowerCommands.wrapRoutine(groundTopFFRoutine).schedule();
        //CTowerCommands.wrapRoutine(groundBottomFFRoutine).schedule();

    }

    void loadCommands() {


        //When driver
        Trigger xGreaterThan = operatorInput.driver.axisGreaterThan(XboxController.Axis.kLeftX.value, 0.1);
        Trigger yGreaterThan = operatorInput.driver.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.1);
        Trigger rotGreaterThan = operatorInput.driver.axisGreaterThan(XboxController.Axis.kRightX.value, 0.1);
        Trigger climberThreshold = operatorInput.operatorControl.axisGreaterThan(XboxController.Axis.kRightY.value, 0.1);

        operatorInput.isTeleop.and(xGreaterThan.or(yGreaterThan).or(rotGreaterThan)).whileTrue(new DefaultDriveCommand(driveSubsystem, odometrySubsystem, operatorInput));

        // Trigger things
        operatorInput.ampSetpoint_hold.whileTrue(new SetAmpShootingAngleCommand(shooterSubsystem));
        operatorInput.speakerSetpoint_hold.whileTrue(new SetSpeakerShootingAngleCommand(shooterSubsystem));
        // .andThen(new ShootNoteCommand(shooterSubsystem))
        operatorInput.shootManually.onTrue(new ShootNoteCommand(shooterSubsystem));
        operatorInput.sourceIntake_hold.whileTrue(new IntakeCommand(shooterSubsystem));
        operatorInput.setShooterAngleManually.onTrue(new SetShootingAngleManuallyCommand(operatorInput, shooterSubsystem));

        HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
                new PIDController(DRIVE_X_PID.pConstant(),DRIVE_X_PID.iConstant(),DRIVE_X_PID.dConstant()),
                new PIDController(DRIVE_Y_PID.pConstant(), DRIVE_Y_PID.iConstant(), DRIVE_Y_PID.dConstant()),
                new ProfiledPIDController(DRIVE_T_PID.pConstant(), DRIVE_T_PID.iConstant(), DRIVE_T_PID.dConstant(),
                        new TrapezoidProfile.Constraints(1,2)) //TODO
        );

        operatorInput.autoAlignHold.whileTrue(new MoveToAlignCommand(driveSubsystem, visionSubsystem, holonomicDriveController, odometrySubsystem, operatorInput));
        operatorInput.isTeleop.and(climberThreshold).whileTrue(new MoveClimberCommand(climberSubsystem, operatorInput));

        operatorInput.groundIntakeHold.whileTrue(new GroundIntakeCommand(groundIntakeSubsystem, operatorInput));
        operatorInput.groundOuttakeHold.whileTrue(new GroundOuttakeCommand(groundIntakeSubsystem, operatorInput));

    }


    SwerveDriveKinematics loadKinematics() {
        return new SwerveDriveKinematics(
                new Translation2d(DRIVE.halfWidth_meters(), DRIVE.halfBase_meters()), // FL
                new Translation2d(DRIVE.halfWidth_meters(), -DRIVE.halfBase_meters()), // FR
                new Translation2d(-DRIVE.halfWidth_meters(), DRIVE.halfBase_meters()), // BL
                new Translation2d(-DRIVE.halfWidth_meters(), -DRIVE.halfBase_meters()) // BR
        );
    }

    DriveSubsystem loadDriveSubsystem() {
        SwerveModule[] modules = loadSwerveModules();
        SimpleMotorFeedforward ff = new SimpleMotorFeedforward(DRIVE.ff_ks(), DRIVE.ff_kv());
        return new DriveSubsystem(modules,kinematics,ff);
    }

    SwerveModule[] loadSwerveModules() {
        SwerveModule[] modules = new SwerveModule[4];
        for (int i = 0; i < modules.length; i++) {
            ILinearMotor driveMotor;
            IRotationalController steerController;
            IRotationEncoder absoluteEncoder;

            if (DISABLER.drive_disabled()) {
                //TODO;
            }
            else if (Robot.isSimulation()) {
                driveMotor = HardwareSIM.linearSIM_noPID(DRIVES[i], DRIVE_SIM, DCMotor.getNEO(1));
                steerController = HardwareSIM.rotationalSIM_pid(STEERS[i], STEER_SIM, PIDS[i], DCMotor.getNEO(1));
                absoluteEncoder = steerController; //TODO silly hack
            } else {
                driveMotor = HardwareREV.linearSpark_noPID(DRIVES[i]);
                steerController = HardwareREV.rotationalSpark_builtInPID(STEERS[i], PIDS[i]);
                absoluteEncoder = new ThriftyAbsoluteEncoder(null, null);
            }
            /*
            if (disabled) {
                driveMotor = HardwareDisabled.linearMotor_disabled();
            }
*/
            modules[i] = new SwerveModule(driveMotor, steerController, absoluteEncoder);
        }

        return modules;
    }

    ShooterSubsystem loadShooterSubsystem() {
        IRotationalMotor leftMotor;
        IRotationalMotor rightMotor;
        IRotationalController angleMotor;
        IRotationEncoder absoluteEncoder;

        if (DISABLER.shooter_disabled()) {
            //TODO
        } else {
            leftMotor = HardwareREV.rotationalSpark_noPID(SHOOTER_WHEEL_1);
            rightMotor = HardwareREV.rotationalSpark_noPID(SHOOTER_WHEEL_2);
            angleMotor = HardwareREV.rotationalSpark_builtInPID(ANGLE_SHOOTER_MOTOR, ANGLE_PID);
            absoluteEncoder = new ThriftyAbsoluteEncoder(new AnalogInput(SHOOTER.channel()), ABSOLUTE);
        }

       return new ShooterSubsystem(
               leftMotor,
               rightMotor,
               angleMotor,
               absoluteEncoder,
               SHOOTER,
               ABSOLUTE
       );

    }
    OdometrySubsystem loadOdometrySubsystem() {
        Pigeon2 pigeon2;

        pigeon2 = new Pigeon2(DRIVE.pigeonCanId());
        // TODO implement swappable version per condition (sim, disable, enable)

        return new OdometrySubsystem(
                driveSubsystem,
                visionSubsystem,
                new SwerveDrivePoseEstimator( //The auto path will reset all of this data anyways
                        kinematics,
                        new Rotation2d(),
                        driveSubsystem.currentPositions(),
                        new Pose2d()
                ),
                pigeon2
        ); //TODO
    }
    VisionSubsystem loadVisionSubsystem() {

        //TODO SWAPPABLE VERSION
        PhotonCamera camera1 = new PhotonCamera(CAMERAS.camera1Name());
        if(DISABLER.vision_disabled()) {
            camera1 = MockingUtil.buddy(PhotonCamera.class);
        }
        PhotonCamera camera2 = new PhotonCamera(CAMERAS.camera2Name());
        if(DISABLER.vision_disabled()){
            camera2 = MockingUtil.buddy(PhotonCamera.class);
        }

        AprilTagFieldLayout aprilTagFieldLayout;

        // better error catching later ig
        try {
            aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            throw new IllegalStateException("something awful happened");
        }


        // USE MATTLIB FOR CONSTANTS HERE IM JUST LAZY TODO
        Transform3d robotToCam1 = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0,0,0));
        // using multi tag localization
        PhotonPoseEstimator photonPoseEstimator1 = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera1,
                robotToCam1
                );

        Transform3d robotToCam2 = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0,0,0));
        PhotonPoseEstimator photonPoseEstimator2 = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera2,
                robotToCam2
        );

        return new VisionSubsystem(
                camera1,
                camera2,
                aprilTagFieldLayout,
                photonPoseEstimator1,
                photonPoseEstimator2,
                new AprilTagDetector()
        );
    }

    ClimberSubsystem loadClimberSubsystem() {
        ILinearController leftClimber;
        ILinearController rightClimber;
        SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(CLIMBER.ff_ks(), CLIMBER.ff_kv());
        if (DISABLER.climber_disabled()) {
            // TODO
        } else {
            leftClimber = HardwareREV.linearSpark_builtInPID(LEFT_CLIMBER, CLIMBER_PID);
            rightClimber = HardwareREV.linearSpark_builtInPID(RIGHT_CLIMBER, CLIMBER_PID);
        }

        return new ClimberSubsystem(
                leftClimber,
                rightClimber,
                feedForward
        ); // TODO

    }


    GroundIntakeSubsystem loadGroundIntakeSubsystem() {
        ILinearController leftGroundIntake;
        ILinearController rightGroundIntake;
        SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(GROUNDINTAKE.ff_ks(), GROUNDINTAKE.ff_kv());

        if (DISABLER.groundIntake_disabled()) {
            // TODO
        } else {
            leftGroundIntake = HardwareREV.linearSpark_builtInPID(TOP_GROUNDINTAKE, TOP_GROUND_PID);
            rightGroundIntake = HardwareREV.linearSpark_builtInPID(BOTTOM_GROUNDINTAKE, BOTTOM_GROUND_PID);
        }

        return new ClimberSubsystem(
                leftGroundIntake,
                rightGroundIntake,
                feedForward
        );
    }



    //Components MUST be created in the Robot class (because of how static bs works)
    //config shit

    public static final VisionComponent VISION = LOG.load(VisionComponent.class, "vision");

    public static final ClimberComponent CLIMBER = LOG.load(ClimberComponent.class, "climber");
    public static final PIDComponent CLIMBER_PID = LOG.load(PIDComponent.class, "climber/climber_pid");
    public static final CommonMotorComponent CLIMBER_COMMON = LOG.load(CommonMotorComponent.class, "climber/common");
    public static final MotorComponent LEFT_CLIMBER = MotorComponent.ofSpecific(CLIMBER_COMMON, LOG.load(IndividualMotorComponent.class, "climber/left"));
    public static final MotorComponent RIGHT_CLIMBER = MotorComponent.ofSpecific(CLIMBER_COMMON, LOG.load(IndividualMotorComponent.class, "climber/right"));

    public static final CommonMotorComponent GROUNDINTAKE_COMMON = LOG.load(CommonMotorComponent.class, "groundintake/common");
    public static final MotorComponent TOP_GROUNDINTAKE = MotorComponent.ofSpecific(GROUNDINTAKE_COMMON, LOG.load(IndividualMotorComponent.class, "groundintake/top"));
    public static final MotorComponent BOTTOM_GROUNDINTAKE = MotorComponent.ofSpecific(GROUNDINTAKE_COMMON, LOG.load(IndividualMotorComponent.class, "groundintake/bottom"));
    public static final PIDComponent TOP_GROUND_PID = LOG.load(PIDComponent.class, "groundintake/top_pid");
    public static final PIDComponent BOTTOM_GROUND_PID = LOG.load(PIDComponent.class, "groundintake/bottom_pid");
    public static final FFGenComponent TOP_GROUND_FFGEN = LOG.load(FFGenComponent.class, "groundintake/ff_top");
    public static final FFGenComponent BOTTOM_GROUND_FFGEN = LOG.load(FFGenComponent.class, "groundintake/ffgen_bottom");
    public static final GroundIntakeComponent GROUNDINTAKE = LOG.load(GroundIntakeComponent.class, "groundintake");
    public static final ShooterComponent SHOOTER = LOG.load(ShooterComponent.class, "shooter");
    public static final CommonMotorComponent SHOOTER_COMMON = LOG.load(CommonMotorComponent.class, "shooter/common");
    public static final MotorComponent SHOOTER_WHEEL_1 = MotorComponent.ofSpecific(SHOOTER_COMMON, LOG.load(IndividualMotorComponent.class, "shooter/wheel_1"));
    public static final MotorComponent SHOOTER_WHEEL_2 = MotorComponent.ofSpecific(SHOOTER_COMMON, LOG.load(IndividualMotorComponent.class, "shooter/wheel_2"));
    public static final MotorComponent ANGLE_SHOOTER_MOTOR = LOG.load(MotorComponent.class,"shooter/angle_motor");
    public static final PIDComponent ANGLE_PID = LOG.load(PIDComponent.class,"shooter/angle/pid");

    public static final DrivebaseComponent DRIVE = LOG.load(DrivebaseComponent.class, "swerve");
    public static final CommonMotorComponent DRIVE_COMMON = LOG.load(CommonMotorComponent.class, "swerve/drive_common");
    public static final CommonMotorComponent STEER_COMMON = LOG.load(CommonMotorComponent.class, "swerve/steer_common");
    public static final CommonPIDComponent PID_COMMON = LOG.load(CommonPIDComponent.class, "swerve/steer_pid_common");
    public static final MotorComponent[] DRIVES = MotorComponent.ofRange(DRIVE_COMMON, LOG.loadRange(IndividualMotorComponent.class, "swerve/drive", 4, Util.RENAMER));
    public static final MotorComponent[] STEERS = MotorComponent.ofRange(STEER_COMMON, LOG.loadRange(IndividualMotorComponent.class, "swerve/steer", 4, Util.RENAMER));
    public static final PIDComponent[] PIDS = PIDComponent.ofRange(PID_COMMON, LOG.loadRange(IndividualPIDComponent.class, "swerve/pid", 4, Util.RENAMER));

    public static final SimComponent DRIVE_SIM = LOG.load(SimComponent.class, "swerve/drive_sim");
    public static final SimComponent STEER_SIM = LOG.load(SimComponent.class, "swerve/steer_sim");

    public static final PIDComponent DRIVE_X_PID = LOG.load(PIDComponent.class, "swerve/x_holonomic_pid");
    public static final PIDComponent DRIVE_Y_PID = LOG.load(PIDComponent.class, "swerve/y_holonomic_pid");
    public static final PIDComponent DRIVE_T_PID = LOG.load(PIDComponent.class, "swerve/t_holonomic_pid");

    public static final EncoderComponent ABSOLUTE = LOG.load(EncoderComponent.class, "absolute");
    public static final CamerasComponent CAMERAS = LOG.load(CamerasComponent.class, "cameras");

    public static final DisablerComponent DISABLER = LOG.load(DisablerComponent.class, "disabler");


}
