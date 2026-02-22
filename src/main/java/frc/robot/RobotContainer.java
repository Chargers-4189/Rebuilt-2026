// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.commands.IntakeRotate;
import frc.robot.commands.MoveHood;
import frc.robot.commands.MoveIndexer;
import frc.robot.commands.RunIntakeWheels;
import frc.robot.commands.Score;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.util.NetworkTables.HoodTable;
import frc.robot.util.NetworkTables.IntakeTable;
import frc.robot.util.NetworkTables.ShooterTable;
import frc.robot.util.NetworkTables.SwerveTable;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hopper;

public class RobotContainer {
    private final CommandXboxController primaryController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
      private final CommandXboxController testController =
      new CommandXboxController(OperatorConstants.kTestControllerPort);

    //Subsystem declaration
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Indexer indexer = new Indexer();
    private final Intake intake = new Intake();

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    private final SwerveRequest.FieldCentricFacingAngle driveWithAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final SwerveSubsystem drivetrain = TunerConstants.createDrivetrain();
    
    private final Vision vision = new Vision(drivetrain);

    private final Hopper hopper = new Hopper();

    public RobotContainer() {
        configureBindings();
        configureSwerveBindings();
    }

    private void configureBindings() {

        //Deploy Intake
        intake.setDefaultCommand(Commands.run(() -> {
            System.out.println(primaryController.getRightTriggerAxis() - primaryController.getLeftTriggerAxis());
            
            intake.setExtensionSpeed(primaryController.getRightTriggerAxis() - primaryController.getLeftTriggerAxis());
        }, intake));

        //Intake Fuel
        primaryController.leftBumper().whileTrue(new RunIntakeWheels(intake, IntakeTable.kIntakeSpeed));

        //Hopper & Shooter
        primaryController.povLeft().onTrue(Commands.parallel(
            Commands.run(() -> hopper.setSpeed(0.4), hopper),
            new MoveIndexer(indexer, shooter))
        );
        primaryController.povRight().onTrue(Commands.run(() -> {
            hopper.setSpeed(0);
            indexer.setIndexerPower(0);
        }, indexer, hopper));

        //Manual Hood
        primaryController.povDown().whileTrue(new MoveHood(hood, () -> -HoodTable.kManualPower.get()));
        primaryController.povUp().whileTrue(new MoveHood(hood, () -> HoodTable.kManualPower.get()));

        hood.setDefaultCommand(Commands.run(() -> {
            hood.setHoodAngle(HoodTable.kDefaultAngle);
        }, hood));

        //Calibration Shoot
        primaryController.x().whileTrue(new Shoot(shooter, ShooterTable.kPower));
        
        //Shoot
        primaryController.rightBumper().whileTrue(new Score(hood, shooter, vision));




        /*
        //Remove later
        primaryController.leftTrigger().whileTrue(new RunIntakeWheels(intake, () -> Constants.IntakeConstants.kIntakeSpeed));
        primaryController.rightTrigger().whileTrue(new RunIntakeWheels(intake, () -> -Constants.IntakeConstants.kIntakeSpeed));
        primaryController.leftBumper().whileTrue(new IntakeRotate(intake, true)); //Doesnt work
        primaryController.rightBumper().whileTrue(new IntakeRotate(intake, false)); //Doesnt work
        */
    }

    private void configureSwerveBindings() {

        primaryController.rightBumper().whileTrue(
            drivetrain.applyRequest(() ->
                driveWithAngle.withVelocityX(-primaryController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-primaryController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withTargetDirection(vision.getRotationFromHub())
                    .withHeadingPID(SwerveTable.kP.get(), SwerveTable.kI.get(), SwerveTable.kD.get())
                    .withMaxAbsRotationalRate(MaxAngularRate)
            )
        );
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-primaryController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-primaryController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-primaryController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        
        /*
        
        primaryController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        primaryController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-primaryController.getLeftY(), -primaryController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        primaryController.back().and(primaryController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        primaryController.back().and(primaryController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        primaryController.start().and(primaryController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        primaryController.start().and(primaryController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        primaryController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        */
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(/*Rotation2d.kZero*/)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
