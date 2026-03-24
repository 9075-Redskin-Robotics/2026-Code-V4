// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private static final double kSnapHeadingToleranceDegrees = 3.0;
    private static final Translation2d kBlueHopperPosition = new Translation2d(
        Units.inchesToMeters(182.11),
        Units.inchesToMeters(158.85));
    private static final double kFieldLengthMeters = Units.inchesToMeters(690.876);

    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final LiftSubsystem lift = new LiftSubsystem();
    private final Limelight shooterLimelight = new Limelight();
    private final CommandXboxController driver = new CommandXboxController(1);
    private final CommandXboxController opperator = new CommandXboxController(0);

    private final double maxSpeed = 0.35 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); //Swerve Speed Ajuster
    private final double boostedSpeed = 0.8 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // Swerve Boosted Speed Ajuster
    private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.4).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle faceAwayFromHopper = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(maxSpeed * 0.4)
            .withRotationalDeadband(maxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(maxSpeed);

    public final CommandSwerveDrivetrain drivetrain=TunerConstants.createDrivetrain();;

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        faceAwayFromHopper.HeadingController.setPID(7, 0, 0);
        faceAwayFromHopper.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        registerCommands();
        
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        Shuffleboard.getTab("Autonomous").add("Auto Mode", autoChooser);

        configureBindings();
        registerdefaultCommands();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }
    private void configureBindings() {
    // Auto shooter speed based on Limelight distance
        opperator.leftTrigger()
            .whileTrue(Commands.run(() -> {
                if (shooterLimelight.hasTarget()) {
                    shooter.runForDistance(shooterLimelight.getTargetDistanceMeters());
                } else {
                    shooter.stop();
                }
            }, shooter))
            .onFalse(Commands.runOnce(shooter::stop, shooter));
    // Fast Shooter
        opperator.leftBumper()
            .whileTrue(Commands.run(() -> shooter.run(.75), shooter))
            .onFalse(Commands.runOnce(shooter::stop, shooter));
    // Slow Shooter
         opperator.y()
            .whileTrue(Commands.run(() -> shooter.run(0.5), shooter))
            .onFalse(Commands.runOnce(shooter::stop, shooter));
    // Unjam Control
        opperator.a()
            .whileTrue(Commands.run(() -> shooter.run(-0.5), shooter))
            .onFalse(Commands.runOnce(shooter::stop, shooter));
    // Intake controls
        opperator.rightTrigger()
            .whileTrue(Commands.run(() -> intake.run(1), intake))
            .onFalse(Commands.runOnce(intake::stop, intake));
        opperator.rightTrigger()
            .whileTrue(Commands.run(() -> shooter.run(0.4), shooter))
            .onFalse(Commands.runOnce(shooter::stop, shooter));
    // Feeder
        opperator.rightBumper()
            .whileTrue(Commands.run(() -> intake.run(-1), intake))
            .onFalse(Commands.runOnce(intake::stop, intake));
    // Lift Down
        opperator.povUp()
            .onTrue(new InstantCommand(() -> lift.climbUp(-0.8)))
            .onFalse(new InstantCommand(() -> lift.stop()));
    // Lift Up
        opperator.povDown()
            .onTrue(new InstantCommand(() -> lift.climbUp(1)))
            .onFalse(new InstantCommand(() -> lift.stop()));
    // Snap to face away from hopper
        opperator.x().onTrue(createSnapAwayFromHopperCommand());
        
    // Drivetrain Speed Control with Boost
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                double triggerAmount = driver.getRightTriggerAxis();
                double driveSpeed = maxSpeed + ((boostedSpeed - maxSpeed) * triggerAmount);
                return drive.withVelocityX(-driver.getLeftY() * driveSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * driveSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * maxAngularRate) // Drive counterclockwise with negative X (left)
                ;
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading without conflicting with intake controls.
        driver.povLeft().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void registerCommands() {
        NamedCommands.registerCommand("Intake", (Commands.run(() -> intake.run(-1), intake)));
        NamedCommands.registerCommand("Shoot", (Commands.run(() -> shooter.run(0.6), shooter)));
        NamedCommands.registerCommand("AutoShoot", Commands.run(() -> {
            if (shooterLimelight.hasTarget()) {
                shooter.runForDistance(shooterLimelight.getTargetDistanceMeters());
            } else {
                shooter.stop();
            }
        }, shooter));
        NamedCommands.registerCommand("SlowShoot", (Commands.run(() -> shooter.run(0.3), shooter)));
        NamedCommands.registerCommand("StopShoot", (Commands.run(() -> shooter.run(0), shooter)));
        NamedCommands.registerCommand("IntakeStop", (Commands.run(() -> intake.run(0), intake)));
        NamedCommands.registerCommand("LiftUp", (Commands.run(() -> lift.climbUp(0.8), lift)));
        NamedCommands.registerCommand("LiftDown", (Commands.run(() -> lift.climbUp(-1), lift)));
    }
// Shooter test code
    private void registerdefaultCommands() {
        shooter.setDefaultCommand(Commands.run(() -> shooter.run(0), shooter));
        shooter.setDefaultCommand(
            Commands.run(
                () -> {
                    double shooterSpeed = -opperator.getRightY();
                    if (Math.abs(shooterSpeed) < 0.1) {
                        shooterSpeed = 0;
                    }
                    shooter.run(shooterSpeed);
                },
                shooter));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    private Rotation2d getHeadingAwayFromHopper() {
        Translation2d hopperPosition = getHopperFieldPosition();
        Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
        Translation2d robotToHopper = hopperPosition.minus(robotPosition);

        if (robotToHopper.getNorm() < 1e-6) {
            return drivetrain.getState().Pose.getRotation();
        }

        return robotToHopper.getAngle().plus(Rotation2d.k180deg);
    }

    private Command createSnapAwayFromHopperCommand() {
        Rotation2d targetHeading = getHeadingAwayFromHopper();

        return drivetrain.applyRequest(() -> faceAwayFromHopper
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withTargetDirection(targetHeading))
            .until(() -> Math.abs(
                drivetrain.getState().Pose.getRotation().minus(targetHeading).getDegrees())
                < kSnapHeadingToleranceDegrees);
    }

    private Translation2d getHopperFieldPosition() {
        boolean isRedAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red;

        if (!isRedAlliance) {
            return kBlueHopperPosition;
        }

        return new Translation2d(
            kFieldLengthMeters - kBlueHopperPosition.getX(),
            kBlueHopperPosition.getY());
    }
}
