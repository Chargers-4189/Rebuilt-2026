// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignAngle;
import frc.robot.handlers.Hopper.HopperState;
import frc.robot.handlers.Intake.IntakeState;
import frc.robot.handlers.Shooter.ShooterState;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.util.Stopwatch;

public class Manager extends SubsystemBase {

  /** Vision Subsystem */
  private Vision vision;
  /** Swerve Subsystem */
  private SwerveSubsystem swerve;
  /** Hopper Subsystem - Fully controlled by the Manager.*/
  private Hopper hopper;
  /** Intake Subsystem - Fully controlled by the Manager.*/
  private Intake intake;
  /** Shooter Subsystem - Fully controlled by the Manager.*/
  private Shooter shooter;

  /** Driving Power in the X Direction.*/
  private DoubleSupplier driveX;
  
  /** Driving Power in the Y Direction.*/
  private DoubleSupplier driveY;

  /** Current State of the Robot.*/
  private RobotState currentState = RobotState.DEFAULT;

  /** Specifies whether the intake should be taunting. */
  private boolean isTaunting = false;

  /** The Event Loop for State-Activated Triggers. */
  private EventLoop stateEventLoop = new EventLoop();

  /** Requests that the robot be stopped. Replaces the StopAll command. */
  private BooleanSupplier reqStop = () -> false;

  /** Triggers when the robot is attempting to shoot. */
  private Trigger shootingTrigger = new Trigger(stateEventLoop, () -> currentState == RobotState.SCORING || currentState == RobotState.PASSING || currentState == RobotState.STATIC_SHOOTING);
  /** Triggers when the robot is actually firing fuel. */
  private Trigger firingTrigger = shootingTrigger.and(new Trigger(stateEventLoop, () -> shooter.isAligned()));
  /** Triggers when the robot should align to the hub. */
  private Trigger scoringTrigger = new Trigger(stateEventLoop, () -> currentState == RobotState.SCORING);
  /** Triggers when the robot should align for passing. */
  private Trigger passingTrigger = new Trigger(stateEventLoop, () -> currentState == RobotState.PASSING);
  /** Triggers when the robot should stop */
  private Trigger stopTrigger = new Trigger(stateEventLoop, reqStop);

  /** Used to measure how long it takes for the flywheel to spin up. */
  private Stopwatch spinupTime = new Stopwatch();

  /** Creates a new Manager. 
   * 
   * @param vision the vision subsystem
   * @param swerve the swerve subsystem
  */
  public Manager(Vision vision, SwerveSubsystem swerve) {
    this.vision = vision;
    this.swerve = swerve;
    this.hopper = new Hopper();
    this.intake = new Intake();
    this.shooter = new Shooter(vision);

    configureStateTriggers();
  }

  /** An enum for representing states the robot could be in. */
  public enum RobotState {
    /** Just driving around. */
    DEFAULT,
    /** Retracting the intake. */
    TUCKING,
    /** Extending the intake. */
    EXTENDING,
    /** Extending the intake and intaking fuel. */
    INTAKING,
    /** Scoring in the hub. */
    SCORING,
    /** Fixed distance scoring in the hub. */
    STATIC_SHOOTING,
    /** Passing fuel to our alliance zone. */
    PASSING,
    /** Extending the intake and outtaking fuel. */
    OUTTAKING,
    /** Forcing the intake to outtake fuel. */
    UNJAMMING,
    /** Deactivating all motors. */
    STOPPED,
    /** Spinning up the shooter to fixed velocity. */
    PRESPIN
  }

  /**
   * Sets the robot to the given state when the condition changes to `true` and resets the state
   * to `RobotState.DEFAULT` when the condition changes to `false`.
   * 
   * @param trigger the trigger
   * @param robotState the state to activate
   */
  public void whileTrue(Trigger trigger, RobotState robotState) {
    trigger.whileTrue(Commands.startEnd(
      () -> setState(robotState),
      () -> setState(RobotState.DEFAULT),
      this
    ));
  }

  /**
   * Sets the robot to the given state when the condition changes to `true`.
   * 
   * @param trigger the trigger
   * @param robotState the state to activate
   */
  public void onTrue(Trigger trigger, RobotState onTrue) {
    trigger.onTrue(Commands.runOnce(
      () -> setState(onTrue),
      this
    ));
  }

  /**
   * Toggles a state when the condition changes from `false` to `true`.
   * 
   * @param trigger the trigger
   * @param robotState the state to toggle
   */
  public void toggleOnTrue(Trigger trigger, RobotState onTrue) {
    trigger.toggleOnTrue(Commands.startEnd(
      () -> setState(onTrue),
      () -> {
        if (this.currentState == onTrue) {
          setState(RobotState.DEFAULT);
        }
      },
      this
    ));
  }

  /**
   * Sets the current state of the robot to the given state.
   * 
   * @param robotState the new state
   */
  public void setState(RobotState robotState) {
    this.currentState = robotState;
  }

  /**
   * Returns the robot's current state.
   * 
   * @return the current robot state
   */
  public RobotState getState() {
    return currentState;
  }

  /**
   * Specifies whether the intake should be taunting.
   * 
   * @param reqTaunt whether the intake should taunt
   */
  public void setTaunting(boolean reqTaunt) {
    this.isTaunting = reqTaunt;
  }

  /**
   * Binds driving controls for the swerve drive, which will be active when the
   * Manager has control over the swerve subsystem.
   * 
   * @param driveX the power to drive in the X direction
   * @param driveY power to drive in the Y direction
   */
  public void bindDrivePower(DoubleSupplier driveX, DoubleSupplier driveY) {
    this.driveX = driveX;
    this.driveY = driveY;
  }

  /**
   * Binds stopAll for the robot.
   * 
   * @param stopAll supplies whether the robot should be stopped.
   */
  public void bindStopAll(BooleanSupplier stopAll) {
    reqStop = stopAll;
  }

  /**
   * Configures any state triggers that should activate when the robot switches
   * between states. Helpful for integrating Commands into the state machine.
   */
  public void configureStateTriggers() {
    shootingTrigger.onTrue(Commands.runOnce(() -> {
      spinupTime.start();
    }));
    firingTrigger.onTrue(Commands.runOnce(() -> {
        System.out.println(spinupTime.getElapsedTime());
    }));

    scoringTrigger.whileTrue(
      new AlignAngle(swerve, driveX, driveY, () -> vision.getRotationFromHub(), false, false)
    );
    passingTrigger.whileTrue(
      new AlignAngle(swerve, driveX, driveY, () -> Vision.convertFieldRotation(Rotation2d.k180deg).getRotations(), false, false)
    );
    stopTrigger.whileTrue(
      Commands.run(() -> swerve.applyRequest(() -> SwerveSubsystem.idle), swerve)
    );
  }

  @Override
  public void periodic() {
    stateEventLoop.poll();

    if (reqStop.getAsBoolean()) {
      currentState = RobotState.STOPPED;
      swerve.applyRequest(() -> SwerveSubsystem.idle);
    }

    switch (currentState) {
      case DEFAULT:
        hopper.setState(HopperState.STOPPED);
        intake.setState(isTaunting ? IntakeState.TAUNTING : IntakeState.STOPPED);
        shooter.setState(ShooterState.DEFAULT);
        break;
      case TUCKING:
        hopper.setState(HopperState.STOPPED);
        intake.setState(IntakeState.RETRACTED);
        shooter.setState(ShooterState.DEFAULT);
        if (isTaunting || intake.isAligned()) {
          setState(RobotState.DEFAULT);
        }
        break;
      case EXTENDING:
        hopper.setState(HopperState.STOPPED);
        intake.setState(IntakeState.RETRACTED);
        shooter.setState(ShooterState.DEFAULT);
        if (isTaunting || intake.isAligned()) {
          setState(RobotState.DEFAULT);
        }
        break;
      case INTAKING:
        hopper.setState(HopperState.STOPPED);
        intake.setState(IntakeState.INTAKING);
        shooter.setState(ShooterState.DEFAULT);
        if (isTaunting) {
          setState(RobotState.DEFAULT);
        }
        break;
      case SCORING:
        hopper.setState(shooter.isAligned() ? HopperState.INDEXING : HopperState.OUTDEXING);
        intake.setState(isTaunting ? IntakeState.TAUNTING : IntakeState.EXTENDED);
        shooter.setState(ShooterState.SCORING);;
        break;
      case STATIC_SHOOTING:
        hopper.setState(shooter.isAligned() ? HopperState.INDEXING : HopperState.OUTDEXING);
        intake.setState(isTaunting ? IntakeState.TAUNTING : IntakeState.EXTENDED);
        shooter.setState(ShooterState.STATIC_SHOOTING);;
        break;
      case PASSING:
        hopper.setState(shooter.isAligned() ? HopperState.INDEXING : HopperState.OUTDEXING);
        intake.setState(isTaunting ? IntakeState.TAUNTING : IntakeState.EXTENDED);
        shooter.setState(ShooterState.PASSING);;
        break;
      case OUTTAKING:
        hopper.setState(HopperState.OUTTAKING);
        intake.setState(IntakeState.OUTTAKING);
        shooter.setState(ShooterState.DEFAULT);
        break;
      case UNJAMMING:
        hopper.setState(HopperState.STOPPED);
        intake.setState(IntakeState.FORCE_OUTTAKE);
        shooter.setState(ShooterState.DEFAULT);
        break;
      case STOPPED:
        hopper.setState(HopperState.STOPPED);
        intake.setState(IntakeState.STOPPED);
        shooter.setState(ShooterState.DEFAULT);
        break;
      case PRESPIN:
        hopper.setState(HopperState.STOPPED);
        intake.setState(IntakeState.STOPPED);
        shooter.setState(ShooterState.PRESPIN);
        break;
    }
  }
}
