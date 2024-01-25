package org.bitbuckets;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
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
import org.bitbuckets.disabled.DisabledGyro;
import org.bitbuckets.disabled.DisablerComponent;
import org.bitbuckets.drive.*;
import org.bitbuckets.groundIntake.GroundIntakeComponent;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementComponent;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.ShooterComponent;
import org.bitbuckets.shooter.ShooterSubsystem;
import org.bitbuckets.util.*;
import org.bitbuckets.vision.CamerasComponent;
import org.bitbuckets.vision.VisionComponent;
import org.bitbuckets.vision.VisionSimContainer;
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

import java.io.IOException;

import static xyz.auriium.mattlib2.Mattlib.LOG;

public class RobotContainer {


    public final DriveSubsystem driveSubsystem;
    public final OperatorInput operatorInput;
    public final ShooterSubsystem shooterSubsystem;
    public final OdometrySubsystem odometrySubsystem;
    public final VisionSubsystem visionSubsystem;
    public final VisionSimContainer visionSimContainer;
    public final ClimberSubsystem climberSubsystem;
    public final GroundIntakeSubsystem groundIntakeSubsystem;
    public final NoteManagementSubsystem noteManagementSubsystem;
    public final SwerveDriveKinematics kinematics;


    public RobotContainer(NoteManagementSubsystem noteManagementSubsystem) {
        //THIS HAS TO RUN FIRST
        Mattlib.LOOPER.runPreInit();
        MattlibSettings.USE_LOGGING = true;
        CommandScheduler.getInstance().enable();

        // load order matters!!!!!
        this.operatorInput = new OperatorInput();
        this.kinematics = loadKinematics();
        this.driveSubsystem = loadDriveSubsystem();
        this.visionSubsystem = loadVisionSubsystem();
        this.odometrySubsystem = loadOdometrySubsystem();
        this.shooterSubsystem = loadShooterSubsystem();
        this.climberSubsystem = loadClimberSubsystem();
        this.groundIntakeSubsystem = loadGroundIntakeSubsystem();
        this.noteManagementSubsystem = loadNoteManagementSubsystem();

        if (!DISABLER.vision_disabled() && Robot.isSimulation()) {
            PhotonCamera[] cameras = visionSubsystem.getCameras();
            this.visionSimContainer = new VisionSimContainer(visionSubsystem, odometrySubsystem,
                                                            cameras[0], cameras[1], visionSubsystem.layout);
        } else this.visionSimContainer = null;

        loadCommands();

        //THIS HAS TO RUN AT THE END
        Mattlib.LOOPER.runPostInit();
    }

    public void simulationPeriodic() {
        this.visionSimContainer.simulationPeriodic();
    }


    public void autonomousInit() {

        ChoreoTrajectory trajectory = Choreo.getTrajectory("sixPointAutoTraj");
        HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
                new PIDController(DRIVE_X_PID.pConstant(),DRIVE_X_PID.iConstant(),DRIVE_X_PID.dConstant()),
                new PIDController(DRIVE_Y_PID.pConstant(), DRIVE_Y_PID.iConstant(), DRIVE_Y_PID.dConstant()),
                new ProfiledPIDController(DRIVE_T_PID.pConstant(), DRIVE_T_PID.iConstant(), DRIVE_T_PID.dConstant(),
                        new TrapezoidProfile.Constraints(1,2)) //TODO
        );

        /*new SequentialCommandGroup(
                new SetSpeakerShootingAngleCommand(shooterSubsystem),
                new ShootNoteCommand(shooterSubsystem),
                new FollowTrajectoryCommand(trajectory, driveSubsystem, odometrySubsystem, holonomicDriveController)
        ).schedule();*/

        new SequentialCommandGroup(new FollowTrajectoryCommand(trajectory, driveSubsystem, odometrySubsystem, holonomicDriveController)).schedule();

    }

    public void testInit() {

        LinearFFGenRoutine groundTopFFRoutine = new LinearFFGenRoutine(TOP_GROUND_FFGEN, groundIntakeSubsystem.topMotor, groundIntakeSubsystem.topMotor);
        //LinearFFGenRoutine groundBottomFFRoutine = new LinearFFGenRoutine(BOTTOM_GROUND_FFGEN, groundIntakeSubsystem.bottomMotor, groundIntakeSubsystem.bottomMotor);
        CTowerCommands.wrapRoutine(groundTopFFRoutine).schedule();
        //CTowerCommands.wrapRoutine(groundBottomFFRoutine).schedule();

    }

    void loadCommands() {


        //When driver
        Trigger xGreaterThan = operatorInput.driver.axisGreaterThan(XboxController.Axis.kLeftX.value, 0.1).or(operatorInput.driver.axisLessThan(XboxController.Axis.kLeftX.value, -0.1));
        Trigger yGreaterThan = operatorInput.driver.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.1).or(operatorInput.driver.axisLessThan(XboxController.Axis.kLeftY.value, -0.1));;
        Trigger rotGreaterThan = operatorInput.driver.axisGreaterThan(XboxController.Axis.kRightX.value, 0.1).or(operatorInput.driver.axisLessThan(XboxController.Axis.kRightX.value, -0.1));;
        Trigger climberThreshold = operatorInput.operatorControl.axisGreaterThan(XboxController.Axis.kRightY.value, 0.1).or(operatorInput.driver.axisLessThan(XboxController.Axis.kRightY.value, -0.1));

        operatorInput.isTeleop.and(xGreaterThan.or(yGreaterThan).or(rotGreaterThan)).whileTrue(new DefaultDriveCommand(driveSubsystem, odometrySubsystem, operatorInput));

        // Trigger things
        operatorInput.ampSetpoint_hold.whileTrue(new SetAmpShootingAngleCommand(shooterSubsystem).andThen(new ShootNoteCommand(shooterSubsystem)));
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
                new Translation2d(SWERVE.halfWidth_meters(), SWERVE.halfBase_meters()), // FL
                new Translation2d(SWERVE.halfWidth_meters(), -SWERVE.halfBase_meters()), // FR
                new Translation2d(-SWERVE.halfWidth_meters(), SWERVE.halfBase_meters()), // BL
                new Translation2d(-SWERVE.halfWidth_meters(), -SWERVE.halfBase_meters()) // BR
        );
    }

    DriveSubsystem loadDriveSubsystem() {
        SwerveModule[] modules = loadSwerveModules();
        SimpleMotorFeedforward ff = new SimpleMotorFeedforward(SWERVE.ff_ks(), SWERVE.ff_kv());
        return new DriveSubsystem(modules,kinematics,ff);
    }

    SwerveModule[] loadSwerveModules() {
        SwerveModule[] modules = new SwerveModule[4];
        for (int i = 0; i < modules.length; i++) {
            ILinearMotor driveMotor;
            IRotationalController steerController;
            IRotationEncoder absoluteEncoder;

            if (DISABLER.drive_disabled()) {
                driveMotor = HardwareDisabled.linearMotor_disabled();
                steerController = HardwareDisabled.rotationalController_disabled();
                absoluteEncoder = HardwareDisabled.rotationEncoder_disabled();
            }
            else if (Robot.isSimulation()) {
                driveMotor = HardwareSIM.linearSIM_noPID(DRIVES[i], DCMotor.getNEO(1));
                steerController = HardwareSIM.rotationalSIM_pid(STEERS[i], PIDS[i], DCMotor.getNEO(1));
                absoluteEncoder = steerController; //TODO silly hack wtf this is not a hack i have spent two hours on this and i have not found a solution
            } else {
                driveMotor = HardwareREV.linearSpark_noPID(DRIVES[i]);
                steerController = HardwareREV.rotationalSpark_builtInPID(STEERS[i], PIDS[i]);
                absoluteEncoder = HardwareUtil.thriftyEncoder(STEER_ABS_ENCODERS[i]);
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
            leftMotor = HardwareDisabled.rotationalMotor_disabled();
            rightMotor = HardwareDisabled.rotationalMotor_disabled();
            angleMotor = HardwareDisabled.rotationalController_disabled();
            absoluteEncoder = HardwareDisabled.rotationEncoder_disabled();
        }
        else if (Robot.isSimulation()){
            leftMotor = HardwareSIM.rotationalSIM_noPID(SHOOTER_WHEEL_1, DCMotor.getNEO(1) );
            rightMotor = HardwareSIM.rotationalSIM_noPID(SHOOTER_WHEEL_2, DCMotor.getNEO(1));
            angleMotor = HardwareSIM.rotationalSIM_pid(ANGLE_SHOOTER_MOTOR,ANGLE_PID,DCMotor.getNEO(1) );
            absoluteEncoder = angleMotor;

        }
        else {
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

    NoteManagementSubsystem loadNoteManagementSubsystem() {
        ILinearMotor nms_bottomMotor;
        ILinearMotor nms_topMotor;


        if (DISABLER.nms_disabled()) {
            nms_bottomMotor = HardwareDisabled.linearController_disabled();
            nms_topMotor = HardwareDisabled.linearMotor_disabled();
        } else if (Robot.isSimulation()) {
            nms_bottomMotor = HardwareSIM.linearSIM_noPID(NMS_COMPONENT, DCMotor.getNEO(1));
            nms_topMotor = HardwareSIM.linearSIM_noPID(NMS_COMPONENT, DCMotor.getNEO(1));
        }

        else {
            nms_bottomMotor = HardwareREV.linearSpark_builtInPID(NMS_COMPONENT, NMS_PID);
            nms_topMotor = HardwareREV.linearSpark_builtInPID(NMS_COMPONENT, NMS_PID);
        }

        return new NoteManagementSubsystem(
                nms_bottomMotor, nms_topMotor,new DigitalInput(BEAM_BREAK.channel())
        );

    }

    OdometrySubsystem loadOdometrySubsystem() {

        IGyro gyro;

        if (Robot.isSimulation() || DISABLER.odometry_disabled()) {
            gyro = new DisabledGyro();
        } else {
            Pigeon2 pigeon2 = new Pigeon2(SWERVE.pigeonCanId());

            gyro = new Pigeon2Gyro(pigeon2);
        }

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
                gyro,
                kinematics
        ); //TODO
    }
    VisionSubsystem loadVisionSubsystem() {

        PhotonCamera camera1;
        PhotonCamera camera2;
        camera1 = new PhotonCamera(CAMERAS.camera1Name());
        camera2 = new PhotonCamera(CAMERAS.camera2Name());

        AprilTagFieldLayout aprilTagFieldLayout;
        String filepath = Filesystem.getDeployDirectory().getPath() + "/2024-crescendo.json";

        // better error catching later ig
        try {
            aprilTagFieldLayout = new AprilTagFieldLayout(filepath);
        } catch (IOException e) {
            throw new IllegalStateException(e.getMessage() + " here is why");
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

        photonPoseEstimator1.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        photonPoseEstimator2.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);


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
            leftClimber = HardwareDisabled.linearController_disabled();
            rightClimber = HardwareDisabled.linearController_disabled();
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
        ILinearController topGroundIntake;
        ILinearController bottomGroundIntake;
        SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(GROUND_INTAKE.ff_ks(), GROUND_INTAKE.ff_kv());

        if (DISABLER.groundIntake_disabled()) {
            topGroundIntake = HardwareDisabled.linearController_disabled();
            bottomGroundIntake = HardwareDisabled.linearController_disabled();
        } else {
            topGroundIntake = HardwareREV.linearSpark_builtInPID(GROUND_INTAKE_TOP, GROUND_INTAKE_PID);
            bottomGroundIntake = HardwareREV.linearSpark_builtInPID(GROUND_INTAKE_BOTTOM, GROUND_INTAKE_PID);
        }

        return new GroundIntakeSubsystem(
                topGroundIntake,
                bottomGroundIntake,
                feedForward
        );
    }


    //Components MUST be created in the Robot class (because of how static bs works)
    //config shit

    public static final VisionComponent VISION = LOG.load(VisionComponent.class, "vision");

    // NMS: Note Management System
    public static final MotorComponent NMS = LOG.load(MotorComponent.class, "note_management/NMS");

    public static final ClimberComponent CLIMBER = LOG.load(ClimberComponent.class, "climber");
    public static final PIDComponent CLIMBER_PID = LOG.load(PIDComponent.class, "climber/climber_pid");
    public static final MotorComponent LEFT_CLIMBER = LOG.load(MotorComponent.class, "climber/left");
    public static final MotorComponent RIGHT_CLIMBER = LOG.load(MotorComponent.class, "climber/right");

    public static final GroundIntakeComponent GROUND_INTAKE = LOG.load(GroundIntakeComponent.class, "ground_intake");
    public static final MotorComponent GROUND_INTAKE_TOP = LOG.load(MotorComponent.class,"ground_intake/top");
    public static final MotorComponent GROUND_INTAKE_BOTTOM = LOG.load(MotorComponent.class, "ground_intake/bottom");
    public static final PIDComponent GROUND_INTAKE_PID = LOG.load(PIDComponent.class, "ground_intake/pid");
    public static final FFGenComponent TOP_GROUND_FFGEN = LOG.load(FFGenComponent.class, "ground_intake/ff_top");

    public static final ShooterComponent SHOOTER = LOG.load(ShooterComponent.class, "shooter");
    public static final MotorComponent SHOOTER_WHEEL_1 = LOG.load(MotorComponent.class, "shooter/wheel_1");
    public static final MotorComponent SHOOTER_WHEEL_2 = LOG.load(MotorComponent.class, "shooter/wheel_2");
    public static final MotorComponent ANGLE_SHOOTER_MOTOR = LOG.load(MotorComponent.class,"shooter/angle_motor");
    public static final PIDComponent ANGLE_PID = LOG.load(PIDComponent.class,"shooter/angle/pid");

    public static final MotorComponent NMS_COMPONENT = LOG.load(MotorComponent.class, "NMS");
    public static final PIDComponent NMS_PID = LOG.load(PIDComponent.class, "NMS/pid");

    public static final SwerveComponent SWERVE = LOG.load(SwerveComponent.class, "swerve");
    public static final CommonMotorComponent DRIVE_COMMON = LOG.load(CommonMotorComponent.class, "swerve/drive_common");
    public static final CommonMotorComponent STEER_COMMON = LOG.load(CommonMotorComponent.class, "swerve/steer_common");
    public static final CommonPIDComponent PID_COMMON = LOG.load(CommonPIDComponent.class, "swerve/steer_pid_common");
    public static final MotorComponent[] DRIVES = MotorComponent.ofRange(DRIVE_COMMON, LOG.loadRange(IndividualMotorComponent.class, "swerve/drive", 4, Util.RENAMER));
    public static final MotorComponent[] STEERS = MotorComponent.ofRange(STEER_COMMON, LOG.loadRange(IndividualMotorComponent.class, "swerve/steer", 4, Util.RENAMER));
    public static final PIDComponent[] PIDS = PIDComponent.ofRange(PID_COMMON, LOG.loadRange(IndividualPIDComponent.class, "swerve/pid", 4, Util.RENAMER));

    public static final CommonEncoderComponent[] STEER_ABS_ENCODERS = LOG.loadRange(CommonEncoderComponent.class, "swerve/abs", 4, Util.RENAMER);

    public static final PIDComponent DRIVE_X_PID = LOG.load(PIDComponent.class, "swerve/x_holonomic_pid");
    public static final PIDComponent DRIVE_Y_PID = LOG.load(PIDComponent.class, "swerve/y_holonomic_pid");
    public static final PIDComponent DRIVE_T_PID = LOG.load(PIDComponent.class, "swerve/t_holonomic_pid");

    public static final CommonEncoderComponent ABSOLUTE = LOG.load(CommonEncoderComponent.class, "absolute");
    public static final CamerasComponent CAMERAS = LOG.load(CamerasComponent.class, "cameras");

    public static final DisablerComponent DISABLER = LOG.load(DisablerComponent.class, "disabler");

    public static final NoteManagementComponent BEAM_BREAK = LOG.load(NoteManagementComponent.class, "beam breaker");

}
