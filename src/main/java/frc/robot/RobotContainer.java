// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.choreo.ChoreoTraj;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.commands.StopAll;
import frc.robot.commands.autos.DepotThenOutpost;
import frc.robot.commands.autos.OutpostOnly;
import frc.robot.commands.autos.OutpostThenDepot;
import frc.robot.commands.autos.SimpleCollectThenShoot;
import frc.robot.commands.intake.IntakeRotate;
import frc.robot.commands.intake.IntakeRunAndRotate;
import frc.robot.commands.intake.IntakeTaunt;
import frc.robot.commands.intake.OuttakeFuel;
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
import frc.robot.subsystems.IntakeExtender;
import frc.robot.subsystems.IntakeWheels;
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
    private final IntakeWheels intakeWheels = new IntakeWheels();
    private final IntakeExtender intakeExtender = new IntakeExtender();

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
        primaryController.b().whileTrue(intakeExtender.setPowerCommand(() -> -IntakeTable.kManualExtensionPower.get()));
        primaryController.y().whileTrue(intakeExtender.setPowerCommand(() -> IntakeTable.kManualExtensionPower.get()));

        //Intake Fuel
        primaryController.rightBumper().toggleOnTrue(new IntakeRunAndRotate(intakeWheels, intakeExtender, IntakeTable.kWheelPower));

        secondaryController.a().whileTrue(intakeWheels.runWheelsCommand(IntakeTable.kWheelPower));

        primaryController.povUp().whileTrue(new OuttakeFuel(intakeWheels, hopper));

        //Taunt
        primaryController.povLeft().whileTrue(new IntakeTaunt(intakeWheels, intakeExtender));

        //Manual Intake

        //primaryController.povUp().whileTrue(intakeExtender.manualExtensionCommand(() -> IntakeTable.kManualExtensionPower.get()));
        //primaryController.povDown().whileTrue(intakeExtender.manualExtensionCommand(() -> -IntakeTable.kManualExtensionPower.get()));

        //Manual Hood
        //primaryController.povDown().whileTrue(new MoveHood(hood, () -> -HoodTable.kManualPower.get()));
        //primaryController.povUp().whileTrue(new MoveHood(hood, () -> HoodTable.kManualPower.get()));

        hood.setDefaultCommand(Commands.run(() -> {
            hood.setHoodAngle(HoodTable.kDefaultAngle);
        }, hood).withName("Hood Default Angle"));

        //Fixed-Distance Shoot
        primaryController.x().whileTrue(new FixedDistanceScore(shooter, hood, indexer, swerve, vision, hopper, primaryController, ShooterTable.kFixedShootDistance));
        
        //Score
        primaryController.leftBumper().whileTrue(
            new Score(shooter, hood, indexer, swerve, vision, hopper, primaryController)
        );

        //primaryController.povLeft().onTrue(new AlignPosition(swerve, vision, new Pose2d(14, 4.4, new Rotation2d())));
        primaryController.a().whileTrue(new AlignAngle(swerve, primaryController, () -> 0, true));
        primaryController.povDown().whileTrue(new Pass(shooter, hood, indexer, hopper, vision, swerve, primaryController));
        //Auto Intake Buttons
        primaryController.leftTrigger(.5).onTrue(new IntakeRotate(intakeExtender, false));
        primaryController.rightTrigger(.5).onTrue(new IntakeRotate(intakeExtender, true));
    }

    private void configureSwerveBindings() {
        
        //Stop All
        primaryController.start().whileTrue(new StopAll(hood, hopper, indexer, intakeWheels, intakeExtender, shooter, swerve));
        secondaryController.start().whileTrue(new StopAll(hood, hopper, indexer, intakeWheels, intakeExtender, shooter, swerve));

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
        autoChooser.addCmd("Quarter Center (Trench, Single)", () -> new SimpleCollectThenShoot(shooter, hood, indexer, swerve, vision, hopper, intakeWheels, intakeExtender, ChoreoTraj.quarterCenter, false));
        autoChooser.addCmd("Quarter Center (Trench, Double)", () -> new SimpleCollectThenShoot(shooter, hood, indexer, swerve, vision, hopper, intakeWheels, intakeExtender, ChoreoTraj.quarterCenter, true));
        autoChooser.addCmd("Quarter Center (Bump, Single)", () -> new SimpleCollectThenShoot(shooter, hood, indexer, swerve, vision, hopper, intakeWheels, intakeExtender, ChoreoTraj.bumpQuarterCenter, false));
        autoChooser.addCmd("Quarter Center (Bump, Double)", () -> new SimpleCollectThenShoot(shooter, hood, indexer, swerve, vision, hopper, intakeWheels, intakeExtender, ChoreoTraj.bumpQuarterCenter, true));

        autoChooser.addCmd("Steal Center (Trench, Single)", () -> new SimpleCollectThenShoot(shooter, hood, indexer, swerve, vision, hopper, intakeWheels, intakeExtender, ChoreoTraj.stealCenter, false));
        autoChooser.addCmd("Steal Center (Trench, Double)", () -> new SimpleCollectThenShoot(shooter, hood, indexer, swerve, vision, hopper, intakeWheels, intakeExtender, ChoreoTraj.stealCenter, true));
        autoChooser.addCmd("Steal Center (Bump, Single)", () -> new SimpleCollectThenShoot(shooter, hood, indexer, swerve, vision, hopper, intakeWheels, intakeExtender, ChoreoTraj.bumpStealCenter, false));
        autoChooser.addCmd("Steal Center (Bump, Double)", () -> new SimpleCollectThenShoot(shooter, hood, indexer, swerve, vision, hopper, intakeWheels, intakeExtender, ChoreoTraj.bumpStealCenter, true));
        
        autoChooser.addCmd("Depot Then Outpost", () -> new DepotThenOutpost(shooter, hood, indexer, swerve, vision, hopper, intakeWheels, intakeExtender));
        autoChooser.addCmd("Outpost Then Depot", () -> new OutpostThenDepot(shooter, hood, indexer, swerve, vision, hopper, intakeWheels, intakeExtender));
        autoChooser.addCmd("Depot Only", () -> new SimpleCollectThenShoot(shooter, hood, indexer, swerve, vision, hopper, intakeWheels, intakeExtender, ChoreoTraj.depotOnly, false));
        autoChooser.addCmd("Outpost Only", () -> new OutpostOnly(shooter, hood, indexer, swerve, vision, hopper, intakeWheels, intakeExtender));

        autoChooser.addCmd("Shoot Preload", () -> new Score(shooter, hood, indexer, swerve, vision, hopper).withTimeout(6));
        SmartDashboard.putData("Auto Chooser", autoChooser);
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
        Telemetry logger = new Telemetry(SwerveSubsystem.MaxSpeed);
        swerve.registerTelemetry(logger::telemeterize);
    }
}