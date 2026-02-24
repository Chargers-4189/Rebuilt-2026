// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.commands.IntakeRotate;
import frc.robot.commands.MoveHood;
import frc.robot.commands.LoadFuel;
import frc.robot.commands.RunIntakeWheels;
import frc.robot.commands.Score;
import frc.robot.commands.AlignShooter;
import frc.robot.commands.ExampleAutoScore;
import frc.robot.commands.AlignShooter;
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
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    
    private final SwerveRequest.FieldCentricFacingAngle driveWithAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    //Disabled Telemetry:
    //private final Telemetry logger = new Telemetry(MaxSpeed);

    public final SwerveSubsystem swerve = TunerConstants.createDrivetrain();
    
    private final Vision vision = new Vision(swerve);

    private final Hopper hopper = new Hopper();

    public RobotContainer() {
        //configureSystemIdBindings();
        configureBindings();
        configureSwerveBindings();
    }

    private void configureSystemIdBindings() {
        primaryController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        primaryController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        /*
        * Joystick Y = quasistatic forward
        * Joystick A = quasistatic reverse
        * Joystick B = dynamic forward
        * Joystick X = dyanmic reverse
        */
        primaryController.y().whileTrue(intake.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        primaryController.a().whileTrue(intake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        primaryController.b().whileTrue(intake.sysIdDynamic(SysIdRoutine.Direction.kForward));
        primaryController.x().whileTrue(intake.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        //Deploy Intake
        intake.setDefaultCommand(Commands.run(() -> {            
            intake.setExtensionSpeed(primaryController.getRightTriggerAxis() - primaryController.getLeftTriggerAxis());
        }, intake));
    }

    private void configureBindings() {

        //Deploy Intake
        intake.setDefaultCommand(Commands.run(() -> {
            //System.out.println(primaryController.getRightTriggerAxis() - primaryController.getLeftTriggerAxis());
            
            intake.setExtensionSpeed(primaryController.getRightTriggerAxis() - primaryController.getLeftTriggerAxis());
        }, intake));

        primaryController.a().whileTrue(Commands.run(() -> {
            intake.setExtensionAngle(IntakeTable.kDefaultAngle.get());
        }));

        //Intake Fuel
        primaryController.leftBumper().whileTrue(new RunIntakeWheels(intake, IntakeTable.kIntakePower));

        //Hopper & Shooter

        /*
        primaryController.povLeft().onTrue(Commands.parallel(
            Commands.run(() -> hopper.setSpeed(0.4), hopper),
            new MoveIndexer(indexer, shooter, swerve))
        );*/

        /*
        primaryController.povRight().onTrue(Commands.run(() -> {
            hopper.setSpeed(0);
            indexer.setIndexerPower(0);
        }, indexer, hopper));
        */

        //Manual Hood
        primaryController.povDown().whileTrue(new MoveHood(hood, () -> -HoodTable.kManualPower.get()));
        primaryController.povUp().whileTrue(new MoveHood(hood, () -> HoodTable.kManualPower.get()));

        hood.setDefaultCommand(Commands.run(() -> {
            hood.setHoodAngle(HoodTable.kDefaultAngle);
        }, hood));

        //Calibration Shoot
        primaryController.x().whileTrue(new Shoot(shooter, ShooterTable.kPower));
        
        //Score
        primaryController.rightBumper().whileTrue(new Score(shooter, hood, indexer, swerve, vision, hopper));




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
            swerve.applyRequest(() ->
                driveWithAngle.withVelocityX(-primaryController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-primaryController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withTargetDirection(vision.getRotationFromHub())
                    .withHeadingPID(SwerveTable.kP.get(), SwerveTable.kI.get(), SwerveTable.kD.get())
                    .withMaxAbsRotationalRate(MaxAngularRate * SwerveTable.kMaxPower.get())
                    .withTargetRateFeedforward(swerve.calculateFeedForward(vision.getRotationFromHub()))
            )
        );
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        swerve.setDefaultCommand(
            // Drivetrain will execute this command periodically
            swerve.applyRequest(() ->
                drive.withVelocityX(-primaryController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-primaryController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-primaryController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            swerve.applyRequest(() -> idle).ignoringDisable(true)
        );

        //Disabled Telemetry:
        //swerve.registerTelemetry(logger::telemeterize);

        
        /*
        
        primaryController.a().whileTrue(swerve.applyRequest(() -> brake));
        primaryController.b().whileTrue(swerve.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-primaryController.getLeftY(), -primaryController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        primaryController.back().and(primaryController.y()).whileTrue(swerve.sysIdDynamic(Direction.kForward));
        primaryController.back().and(primaryController.x()).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
        primaryController.start().and(primaryController.y()).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
        primaryController.start().and(primaryController.x()).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        primaryController.leftBumper().onTrue(swerve.runOnce(swerve::seedFieldCentric));

        */
    }

    public Command getAutonomousCommand() { /*
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }*/

  Pose2d currentRobotPose = swerve.getState().Pose;
    Pose2d twoFeetForward = currentRobotPose.plus(new Transform2d(1,0,currentRobotPose.getRotation())); 

    PathConstraints constraints = PathConstraints.unlimitedConstraints(12);
    // try {
    //         return AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("test.path"),constraints);
    // } catch (Exception e) {
    //     return Commands.none();
    // }
    // return AutoBuilder.pathfindToPose(twoFeetForward, constraints);

        return new ExampleAutoScore(shooter, hood, indexer, swerve, vision, hopper);
    }
}
