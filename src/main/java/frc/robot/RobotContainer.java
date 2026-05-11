// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.choreo.ChoreoTraj;
import frc.robot.commands.AlignAngle;
import frc.robot.commands.autos.ScoreWithTaunt;
import frc.robot.commands.autos.SimpleCollectThenShoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Faces;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotState;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.shooter.Shooter.ShootingType;
import frc.robot.util.NetworkTables;
import frc.robot.util.NetworkTables.SwerveTable;

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

    private final Superstructure superstructure = new Superstructure(
        vision,
        swerve
    );

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
        
        //Cancel All
        superstructure.bindStopAll(primaryController.start().or(secondaryController.start()).or(secondaryController.back()));

        /*---------- Primary Controls ----------*/

        //Rotate Intake
        superstructure.onTrue(primaryController.leftTrigger(.5), RobotState.EXTENDING);
        superstructure.onTrue(primaryController.rightTrigger(.5), RobotState.TUCKED);

        //Outtake Fuel
        superstructure.whileTrue(primaryController.povUp(), RobotState.OUTTAKING);

        //Intake Fuel
        superstructure.whileTrue(primaryController.rightBumper(), RobotState.INTAKING);
        
        //Score
        superstructure.whileTrue(primaryController.leftBumper(), RobotState.ALIGNING, ShootingType.SCORE);

        //Intake Wooble (Taunt)
        primaryController.x().whileTrue(Commands.startEnd(
            () -> superstructure.setTaunting(true), 
            () -> superstructure.setTaunting(false)
        ));

        //Fixed Distance Score        
        superstructure.whileTrue(primaryController.b(), RobotState.ALIGNING, ShootingType.STATIC);

        //Align to Trench
        primaryController.a().whileTrue(
            new AlignAngle(swerve, primaryController, () -> 0, true, false)
        );

        //Shuttle
        superstructure.whileTrue(primaryController.y(), RobotState.ALIGNING, ShootingType.PASS);

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

        //Manual Intake Extension
        //secondaryController.leftTrigger(.5).whileTrue(intakeExtender.setPowerCommand(() -> -IntakeTable.kManualExtensionPower.get()));
        //secondaryController.rightTrigger(.5).whileTrue(intakeExtender.setPowerCommand(() -> IntakeTable.kManualExtensionPower.get()));

        //Manual Intake Wheels
        superstructure.whileTrue(secondaryController.a(), RobotState.UNJAMMING);

        //secondaryController.y().whileTrue(new AlignPosition(swerve, vision, new Pose2d(14, 4, Rotation2d.kZero)));
    }

    public void configureAutoChooser() {

        boolean resetOdom = true;

        //This is the main auto we ran at the state competition. Previously Titled "Closer Center V2"
        autoChooser.addCmd("DCMP Auto", () -> new SimpleCollectThenShoot(superstructure, swerve, vision, ChoreoTraj.closerCenterV2, ChoreoTraj.secondPassCopy1, ChoreoTraj.thirdPassB, 3, resetOdom));
        
        //This auto tries has an optimized second pass, trying to score slightly more fuel.
        autoChooser.addCmd("Worlds Auto", () -> new SimpleCollectThenShoot(superstructure, swerve, vision, ChoreoTraj.closerCenterV2, ChoreoTraj.secondPassV3, ChoreoTraj.thirdPassB, 3, resetOdom));
        
        //This auto shouldn't cross the center line
        autoChooser.addCmd("Fear the Center Line", () -> new SimpleCollectThenShoot(superstructure, swerve, vision, ChoreoTraj.closerCenterV2, ChoreoTraj.secondPassV3, ChoreoTraj.thirdPassB, 3, resetOdom));
        
        //Third Wheel
        autoChooser.addCmd("Third Wheel", () -> Commands.sequence(
            Commands.waitSeconds(5),
            new SimpleCollectThenShoot(superstructure, swerve, vision, ChoreoTraj.shootPreload, ChoreoTraj.secondPassV3, ChoreoTraj.thirdPassB, 3, resetOdom)
        ));

        //Preload Only.
        autoChooser.addCmd("Shoot Preload", () -> new ScoreWithTaunt(superstructure, swerve, vision).withTimeout(6));
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    
    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
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