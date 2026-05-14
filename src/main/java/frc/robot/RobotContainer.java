// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.Faces;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.commands.StopAll;
import frc.robot.commands.autos.AlignPosition;
import frc.robot.commands.autos.DepotThenOutpost;
import frc.robot.commands.autos.OutpostOnly;
import frc.robot.commands.autos.OutpostThenDepot;
import frc.robot.commands.autos.ScoreWithTaunt;
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
      new CommandXboxController(OperatorConstants.kPrimaryControllerPort);
      private final CommandXboxController secondaryController =
      new CommandXboxController(OperatorConstants.kSecondaryControllerPort);
      private final CommandXboxController faceController =
      new CommandXboxController(OperatorConstants.kFaceControllerPort);

    //Subsystem declaration
    private final SwerveSubsystem swerve = TunerConstants.createDrivetrain();
    private final Vision vision = new Vision(swerve);

    private final IntakeExtender intakeExtender = new IntakeExtender();
    private final IntakeWheels intakeWheels = new IntakeWheels();
    private final Hopper hopper = new Hopper();
    private final Indexer indexer = new Indexer();
    private final Hood hood = new Hood();
    private final Shooter shooter = new Shooter();

    private final Faces face = new Faces();

    private static AutoChooser autoChooser = new AutoChooser();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            //.withDeadband(swerve.MaxSpeed * 0.1).withRotationalDeadband(swerve.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    
    public RobotContainer() {
        configureBindings();
        //swerveSystemId();
        configureAutoChooser();
        NetworkTables.initialize(primaryController);
        configureFaces();
    }

    private void configureBindings() {

        //Hood Default Command
        hood.setDefaultCommand(Commands.run(() -> {
            hood.setHoodAngle(HoodTable.kDefaultAngle);
        }, hood).withName("Hood Default Angle"));

        /*---------- Primary Controls ----------*/
        
        //Cancel All
        primaryController.start().whileTrue(new StopAll(hood, hopper, indexer, intakeWheels, intakeExtender, shooter, swerve));

        //Rotate Intake
        primaryController.leftTrigger(.5).onTrue(new IntakeRotate(intakeExtender, true));
        primaryController.rightTrigger(.5).onTrue(new IntakeRotate(intakeExtender, false));

        //Outtake Fuel
        primaryController.povUp().whileTrue(new OuttakeFuel(intakeWheels, hopper));

        //Intake Fuel
        primaryController.rightBumper().toggleOnTrue(new IntakeRunAndRotate(intakeWheels, intakeExtender, IntakeTable.kWheelPower));
        
        //Score
        primaryController.leftBumper().whileTrue(
            new Score(shooter, hood, indexer, swerve, vision, hopper, primaryController)
        );

        //Intake Wooble (Taunt)
        primaryController.x().whileTrue(new IntakeTaunt(intakeWheels, intakeExtender));

        //Fixed Distance Score
        primaryController.b().whileTrue(new FixedDistanceScore(shooter, hood, indexer, swerve, vision, hopper, primaryController, ShooterTable.kFixedShootDistance));
        
        //Align to Trench
        primaryController.a().whileTrue(new AlignAngle(swerve, primaryController, () -> 0, true, false));

        //Shuttle
        primaryController.y().whileTrue(new Pass(shooter, hood, indexer, hopper, vision, swerve, primaryController));

        /*---------- Drivetrain ----------*/

        //Reset Gyro
        primaryController.back().onTrue(swerve.resetGyro());

        //Drive
        swerve.setDefaultCommand(
            swerve.applyRequest(() ->
                drive.withVelocityX(MathUtil.copyDirectionPow(-primaryController.getLeftY(), SwerveTable.kDriveExponent.get()) * SwerveSubsystem.MaxSpeed)
                    .withVelocityY(MathUtil.copyDirectionPow(-primaryController.getLeftX(), SwerveTable.kDriveExponent.get()) * SwerveSubsystem.MaxSpeed)
                    .withRotationalRate(MathUtil.copyDirectionPow(-primaryController.getRightX(), SwerveTable.kRotationalExponent.get()) * SwerveSubsystem.MaxAngularRate)
            )
        );

        //Idle When Disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            swerve.applyRequest(() -> idle).ignoringDisable(true)
        );

        /*---------- Secondary Controls ----------*/

        //Cancel All
        secondaryController.start().whileTrue(new StopAll(hood, hopper, indexer, intakeWheels, intakeExtender, shooter, swerve));
        secondaryController.back().whileTrue(new StopAll(hood, hopper, indexer, intakeWheels, intakeExtender, shooter, swerve));
        
        //Intake Extension
        secondaryController.leftTrigger(.5).whileTrue(intakeExtender.setPowerCommand(() -> -IntakeTable.kManualExtensionPower.get()));
        secondaryController.rightTrigger(.5).whileTrue(intakeExtender.setPowerCommand(() -> IntakeTable.kManualExtensionPower.get()));

        //Intake Wheels
        secondaryController.a().whileTrue(intakeWheels.runWheelsCommand(IntakeTable.kWheelPower));

        //secondaryController.y().whileTrue(new AlignPosition(swerve, vision, new Pose2d(14, 4, Rotation2d.kZero)));
    }

    public void configureAutoChooser() {

        boolean resetOdom = true;

        //This is the main auto we ran at the state competition. Previously Titled "Closer Center V2"
        autoChooser.addCmd("DCMP Auto", () -> new SimpleCollectThenShoot(shooter, hood, indexer, swerve, vision, hopper, intakeWheels, intakeExtender, ChoreoTraj.closerCenterV2, ChoreoTraj.secondPassCopy1, ChoreoTraj.thirdPassB, 3, resetOdom));
        
        //This auto tries has an optimized second pass, trying to score slightly more fuel.
        autoChooser.addCmd("Worlds Auto", () -> new SimpleCollectThenShoot(shooter, hood, indexer, swerve, vision, hopper, intakeWheels, intakeExtender, ChoreoTraj.closerCenterV2, ChoreoTraj.secondPassV3, ChoreoTraj.thirdPassB, 3, resetOdom));
        
        //This auto shouldn't cross the center line
        autoChooser.addCmd("Fear the Center Line", () -> new SimpleCollectThenShoot(shooter, hood, indexer, swerve, vision, hopper, intakeWheels, intakeExtender, ChoreoTraj.closerCenterV2, ChoreoTraj.secondPassV3, ChoreoTraj.thirdPassB, 3, resetOdom));
        
        //Third Wheel
        autoChooser.addCmd("Third Wheel", () -> Commands.sequence(
            Commands.waitSeconds(5),
            new SimpleCollectThenShoot(shooter, hood, indexer, swerve, vision, hopper, intakeWheels, intakeExtender, ChoreoTraj.shootPreload, ChoreoTraj.secondPassV3, ChoreoTraj.thirdPassB, 3, resetOdom)
        ));

        //Preload Only.
        autoChooser.addCmd("Shoot Preload", () -> new ScoreWithTaunt(shooter, hood, indexer, swerve, vision, hopper, intakeWheels, intakeExtender).withTimeout(6));
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    
    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
    }

    public Command getTeleopInitCommand() {
        return new IntakeRotate(intakeExtender, true);
    }

    public void activateVision() {
        vision.activate();
    }
    public void deactivateVision() {
        vision.activate();
    }

    public void setFilterOppositeSide(boolean filterOppositeSide) {
        vision.setFilterOppositeSide(filterOppositeSide);
    }

    private void configureFaces() {
        faceController.button(1).onTrue(Commands.runOnce(() -> {
            face.defaultanimation(0);
        },face).ignoringDisable(true));
        
        faceController.button(2).onTrue(Commands.runOnce(() -> {
            face.happy();
        },face).ignoringDisable(true));

        faceController.button(
            3).onTrue(Commands.runOnce(() -> {
            face.sad(0);
        },face).ignoringDisable(true));

        faceController.button(4).onTrue(Commands.runOnce(() -> {
            face.dead();
        },face).ignoringDisable(true));

        faceController.button(5).onTrue(Commands.runOnce(() -> {
            face.love();
        },face).ignoringDisable(true));

        faceController.button(7).onTrue(Commands.runOnce(() -> {
            face.scared(0);
        },face).ignoringDisable(true));

        /*
        faceController.button(6).onTrue(Commands.runOnce(() -> {
            face.pirate();
        },face).ignoringDisable(true));
        */

        faceController.button(8).onTrue(Commands.runOnce(() -> {
            face.mad();
        },face).ignoringDisable(true));

        faceController.button(9).onTrue(Commands.runOnce(() -> {
            face.lookleft();
        },face).ignoringDisable(true));
        
        faceController.button(10).onTrue(Commands.runOnce(() -> {
            face.lookright();
        },face).ignoringDisable(true));

        faceController.button(11).onTrue(Commands.runOnce(() -> {
            face.sleepy();
        },face).ignoringDisable(true));

        faceController.button(12).onTrue(Commands.runOnce(() -> {
            face.confusedanimation(0);
        },face).ignoringDisable(true));

        faceController.button(13).onTrue(Commands.runOnce(() -> {
            face.monster();
        },face).ignoringDisable(true));
        faceController.button(14).onTrue(Commands.runOnce(() -> {
            face.alien();
        },face).ignoringDisable(true));

        faceController.button(15).onTrue(Commands.runOnce(() -> {
            face.pirate(); // used to be money animation
        },face).ignoringDisable(true));
        
    }

    /*
    private void swerveSystemId() {

        //Stop All
        primaryController.start().whileTrue(new StopAll(hood, hopper, indexer, intakeWheels, intakeExtender, shooter, swerve));

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

        // Run SysId routines when holding back/start and X/Y.

        // Note that each routine should be run exactly once in a single log.
        primaryController.y().whileTrue(swerve.sysIdDynamic(Direction.kForward));
        primaryController.x().whileTrue(swerve.sysIdDynamic(Direction.kReverse));
        primaryController.a().whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
        primaryController.b().whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        primaryController.back().onTrue(swerve.runOnce(swerve::seedFieldCentric));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            swerve.applyRequest(() -> idle).ignoringDisable(true)
        );

        //Log data
        Telemetry logger = new Telemetry(SwerveSubsystem.MaxSpeed);
        swerve.registerTelemetry(logger::telemeterize);
    }
    */
}