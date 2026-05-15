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
import frc.robot.commands.ScoreWithTaunt;
import frc.robot.commands.SimpleCollectThenShoot;
import frc.robot.handlers.Manager;
import frc.robot.handlers.Manager.RobotState;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.util.NetworkTables;
import frc.robot.util.NetworkTables.SwerveTable;

public class RobotContainer {
    private final CommandXboxController primaryController =
      new CommandXboxController(OperatorConstants.kPrimaryControllerPort);
      private final CommandXboxController secondaryController =
      new CommandXboxController(OperatorConstants.kSecondaryControllerPort);

    //Subsystem declaration
    private final SwerveSubsystem swerve = TunerConstants.createDrivetrain();
    private final Vision vision = new Vision(swerve);

    private final Manager manager = new Manager(vision, swerve);

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
    }

    private void configureBindings() {
        
        //Cancel All
        manager.bindStopAll(primaryController.start().or(secondaryController.start()).or(secondaryController.back()));

        /*---------- Primary Controls ----------*/

        //Rotate Intake
        manager.onTrue(primaryController.leftTrigger(.5), RobotState.EXTENDING);
        manager.onTrue(primaryController.rightTrigger(.5), RobotState.TUCKING);

        //Outtake Fuel
        manager.whileTrue(primaryController.povUp(), RobotState.OUTTAKING);

        //Intake Fuel
        manager.whileTrue(primaryController.rightBumper(), RobotState.INTAKING);
        
        //Score
        manager.whileTrue(primaryController.leftBumper(), RobotState.SCORING);

        //Intake Wooble (Taunt)
        primaryController.x().whileTrue(Commands.startEnd(
            () -> manager.setTaunting(true), 
            () -> manager.setTaunting(false)
        ));

        //Fixed Distance Score        
        manager.whileTrue(primaryController.b(), RobotState.STATIC_SHOOTING);

        //Align to Trench
        primaryController.a().whileTrue(
            new AlignAngle(swerve, primaryController, () -> 0, true, false)
        );

        //Shuttle
        manager.whileTrue(primaryController.y(), RobotState.PASSING);

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
        //secondaryController.leftTrigger(.5).whileTrue(extender.setPowerCommand(() -> -IntakeTable.kManualExtensionPower.get()));
        //secondaryController.rightTrigger(.5).whileTrue(extender.setPowerCommand(() -> IntakeTable.kManualExtensionPower.get()));

        //Manual Intake Wheels
        manager.whileTrue(secondaryController.a(), RobotState.UNJAMMING);

        //secondaryController.y().whileTrue(new AlignPosition(swerve, vision, new Pose2d(14, 4, Rotation2d.kZero)));
    }

    public void configureAutoChooser() {

        boolean resetOdom = true;

        //This is the main auto we ran at the state competition. Previously Titled "Closer Center V2"
        autoChooser.addCmd("DCMP Auto", () -> new SimpleCollectThenShoot(manager, swerve, vision, ChoreoTraj.closerCenterV2, ChoreoTraj.secondPassCopy1, ChoreoTraj.thirdPassB, 3, resetOdom));
        
        //This auto tries has an optimized second pass, trying to score slightly more fuel.
        autoChooser.addCmd("Worlds Auto", () -> new SimpleCollectThenShoot(manager, swerve, vision, ChoreoTraj.closerCenterV2, ChoreoTraj.secondPassV3, ChoreoTraj.thirdPassB, 3, resetOdom));
        
        //This auto shouldn't cross the center line
        autoChooser.addCmd("Fear the Center Line", () -> new SimpleCollectThenShoot(manager, swerve, vision, ChoreoTraj.closerCenterV2, ChoreoTraj.secondPassV3, ChoreoTraj.thirdPassB, 3, resetOdom));
        
        //Third Wheel
        autoChooser.addCmd("Third Wheel", () -> Commands.sequence(
            Commands.waitSeconds(5),
            new SimpleCollectThenShoot(manager, swerve, vision, ChoreoTraj.shootPreload, ChoreoTraj.secondPassV3, ChoreoTraj.thirdPassB, 3, resetOdom)
        ));

        //Preload Only.
        autoChooser.addCmd("Shoot Preload", () -> new ScoreWithTaunt(manager, swerve, vision).withTimeout(6));
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

    /*
    private void swerveSystemId() {

        //Stop All
        primaryController.start().whileTrue(new StopAll(hood, hopper, indexer, wheels, extender, shooter, swerve));

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