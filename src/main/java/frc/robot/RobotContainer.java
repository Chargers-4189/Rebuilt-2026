// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathConstraints;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.commands.StopAll;
import frc.robot.commands.autos.AlignPosition;
import frc.robot.commands.autos.AutoCenterCollectAndShoot;
import frc.robot.commands.autos.AutoCenterCollectAndShootFullPath;
import frc.robot.commands.autos.AutoCenterCollectWOInterferance;
import frc.robot.commands.autos.AutoShootOurSide;
import frc.robot.commands.autos.ChoreoCenterCollect1;
import frc.robot.commands.hood.MoveHood;
import frc.robot.commands.intake.IntakeRotate;
import frc.robot.commands.intake.IntakeRunAndRotate2;
import frc.robot.commands.intake.OuttakeFuel;
import frc.robot.commands.intake.RunIntakeWheels;
import frc.robot.commands.passing.Pass;
import frc.robot.commands.scoring.AlignAngle;
import frc.robot.commands.scoring.FixedDistanceScore;
import frc.robot.commands.scoring.Score;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.util.NetworkTables;
import frc.robot.util.NetworkTables.HoodTable;
import frc.robot.util.NetworkTables.IntakeTable;
import frc.robot.util.NetworkTables.ShooterTable;
import frc.robot.util.NetworkTables.SwerveTable;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hopper;

public class RobotContainer {
    private final CommandXboxController primaryController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
      private final CommandXboxController secondaryController =
      new CommandXboxController(OperatorConstants.kTestControllerPort);


    //Subsystem declaration
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Indexer indexer = new Indexer();
    private final Intake intake = new Intake();

    private static AutoChooser autoChooser = new AutoChooser();
    //Disabled Telemetry:

    public final SwerveSubsystem swerve = TunerConstants.createDrivetrain();
    
    private final Vision vision = new Vision(swerve);

    private final Hopper hopper = new Hopper();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            //.withDeadband(swerve.MaxSpeed * 0.1).withRotationalDeadband(swerve.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    public RobotContainer() {
        configureBindings();
        configureSwerveBindings();
        //intakeSystemId();
        //swerveSystemId();
        configureAutoChooser();
        NetworkTables.initialize(primaryController);
    }

    private void configureBindings() {

        //Deploy Intake
        primaryController.rightTrigger(.1).or(primaryController.leftTrigger(.1)).whileTrue(
            Commands.run(() -> {            
                intake.setExtensionPower(IntakeTable.kManualExtensionPower.get() * (primaryController.getRightTriggerAxis() - primaryController.getLeftTriggerAxis()));
            }, intake)
            .finallyDo(() -> intake.setExtensionPower(0))
            .withName("Manual Intake")
        );

        primaryController.b().onTrue(new IntakeRotate(intake, true));
        primaryController.y().onTrue(new IntakeRotate(intake, false));

        //Intake Fuel
        primaryController.rightBumper().toggleOnTrue(new IntakeRunAndRotate2(intake, IntakeTable.kWheelPower));

        primaryController.x().whileTrue(new OuttakeFuel(intake, hopper));

        //Manual Hood
        primaryController.povDown().whileTrue(new MoveHood(hood, () -> -HoodTable.kManualPower.get()));
        primaryController.povUp().whileTrue(new MoveHood(hood, () -> HoodTable.kManualPower.get()));

        hood.setDefaultCommand(Commands.run(() -> {
            hood.setHoodAngle(HoodTable.kDefaultAngle);
        }, hood).withName("Hood Default Angle"));

        //Fixed-Distance Shoot
        primaryController.a().whileTrue(new FixedDistanceScore(shooter, hood, indexer, swerve, vision, hopper, intake, primaryController, ShooterTable.kFixedShootDistance));
        
        //Score
        primaryController.leftBumper().whileTrue(
            new Score(shooter, hood, indexer, swerve, vision, hopper, intake, primaryController)
        );

        //primaryController.povLeft().onTrue(new AlignPosition(swerve, vision, new Pose2d(14, 4.4, new Rotation2d())));
        primaryController.povRight().whileTrue(new AlignAngle(swerve, primaryController, () -> 0, true));
        primaryController.povLeft().whileTrue(new Pass(shooter, hood, indexer, hopper, intake, swerve));
    }

    private void configureSwerveBindings() {
        
        //Stop All
        primaryController.start().whileTrue(new StopAll(hood, hopper, indexer, intake, shooter, swerve));

        //Reset Gyro
        primaryController.back().onTrue(swerve.resetGyro());

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        swerve.setDefaultCommand(
            // Drivetrain will execute this command periodically
            swerve.applyRequest(() ->
                drive.withVelocityX(MathUtil.copyDirectionPow(-primaryController.getLeftY(), SwerveTable.kDriveExponent.get()) * SwerveSubsystem.MaxSpeed)
                    .withVelocityY(MathUtil.copyDirectionPow(-primaryController.getLeftX(), SwerveTable.kDriveExponent.get()) * SwerveSubsystem.MaxSpeed)
                    .withRotationalRate(MathUtil.copyDirectionPow(-primaryController.getRightX(), SwerveTable.kRotationalExponent.get()) * SwerveSubsystem.MaxAngularRate)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            swerve.applyRequest(() -> idle).ignoringDisable(true)
        );
    }

    public void configureAutoChooser() {
        autoChooser.addCmd("quarterCenter", () -> new ChoreoCenterCollect1(shooter, hood, indexer, swerve, vision, hopper, intake));
        SmartDashboard.putData(autoChooser);
    }

    
    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
    }

    private void swerveSystemId() {
        // Run SysId routines when holding back/start and X/Y.

        // Note that each routine should be run exactly once in a single log.
        primaryController.y().whileTrue(swerve.sysIdDynamic(Direction.kForward));
        primaryController.x().whileTrue(swerve.sysIdDynamic(Direction.kReverse));
        primaryController.a().whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
        primaryController.b().whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        primaryController.leftBumper().onTrue(swerve.runOnce(swerve::seedFieldCentric));

        //Log data
        Telemetry logger = new Telemetry(swerve.MaxSpeed);
        swerve.registerTelemetry(logger::telemeterize);
    }

    private void intakeSystemId() {
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
            intake.setExtensionPower(primaryController.getRightTriggerAxis() - primaryController.getLeftTriggerAxis());
        }, intake).withName("Manual Intake"));
    }
}