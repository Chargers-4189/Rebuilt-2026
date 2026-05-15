// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignAngle;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.shooter.Shooter.ShootingType;
import frc.robot.util.Stopwatch;

public class Manager extends SubsystemBase {

  private Vision vision;
  private SwerveSubsystem swerve;
  private Hopper hopper;
  private Intake intake;
  private Shooter shooter;

  private DoubleSupplier driveX;
  private DoubleSupplier driveY;
  private DoubleSupplier unjamPower;

  private RobotState currentState = RobotState.DEFAULT;
  private boolean isTaunting = false;

  private EventLoop stateEventLoop = new EventLoop();

  //private Trigger defaultTrigger = new Trigger(stateEventLoop, () -> robotState == RobotState.DEFAULT);
  //private Trigger tuckedTrigger = new Trigger(stateEventLoop, () -> robotState == RobotState.TUCKED);
  //private Trigger extendedTrigger = new Trigger(stateEventLoop, () -> robotState == RobotState.EXTENDED);
  //private Trigger intakingTrigger = new Trigger(stateEventLoop, () -> robotState == RobotState.INTAKING);
  
  private Trigger shootingTrigger = new Trigger(stateEventLoop, () -> currentState == RobotState.SCORING || currentState == RobotState.PASSING || currentState == RobotState.STATIC_SHOOTING);
  
  private Trigger aligningTrigger = shootingTrigger.and(new Trigger(stateEventLoop, () -> !shooter.isAligned()));
  private Trigger firingTrigger = shootingTrigger.and(new Trigger(stateEventLoop, () -> shooter.isAligned()));
  
  //private Trigger outtakingTrigger = new Trigger(stateEventLoop, () -> robotState == RobotState.OUTTAKING);
  //private Trigger unjamTrigger = new Trigger(stateEventLoop, () -> robotState == RobotState.UNJAMMING);
  //private Trigger stoppedTrigger = new Trigger(stateEventLoop, () -> robotState == RobotState.STOPPED);
  //private Trigger prespinTrigger = new Trigger(stateEventLoop, () -> robotState == RobotState.PRESPIN);

  private Trigger scoringTrigger = new Trigger(stateEventLoop, () -> {
    return currentState == RobotState.SCORING;
  });

  private Trigger passingTrigger = new Trigger(stateEventLoop, () -> {
    return currentState == RobotState.PASSING;
  });

  //private Trigger staticShootingTrigger = new Trigger(stateEventLoop, () -> {
  //  return currentState == RobotState.STATIC_SHOOTING;
  //});

  private Trigger reqStop;

  private Stopwatch spinupTime = new Stopwatch();

  /** Creates a new Manager. */
  public Manager(Vision vision, SwerveSubsystem swerve) {
    this.vision = vision;
    this.swerve = swerve;
    this.hopper = new Hopper();
    this.intake = new Intake();
    this.shooter = new Shooter(vision);

    configureStateTriggers();
  }

  public enum RobotState {
    DEFAULT,
    TUCKING,
    EXTENDING,
    INTAKING,
    SCORING,
    STATIC_SHOOTING,
    PASSING,
    OUTTAKING,
    UNJAMMING,
    STOPPED,
    PRESPIN
  }

  public void stateLoop() {
    stateEventLoop.poll();

    if (reqStop.getAsBoolean()) {
      currentState = RobotState.STOPPED;
      swerve.applyRequest(() -> SwerveSubsystem.idle);
    }

    switch (currentState) {
      case DEFAULT:
        hopper.stop();
        intake.setState(isTaunting ? IntakeState.TAUNTING : IntakeState.STOPPED);
        shooter.setState(ShooterState.STOPPED);
        break;
      case TUCKING:
        hopper.stop();
        intake.setState(IntakeState.RETRACTED);
        shooter.setState(ShooterState.STOPPED);
        if (isTaunting || intake.extender.isAligned()) {
          setState(RobotState.DEFAULT);
        }
        break;
      case EXTENDING:
        hopper.stop();
        intake.setState(IntakeState.RETRACTED);
        shooter.setState(ShooterState.STOPPED);
        if (isTaunting || intake.extender.isAligned()) {
          setState(RobotState.DEFAULT);
        }
        break;
      case INTAKING:
        hopper.stop();
        intake.setState(IntakeState.INTAKING);
        shooter.setState(ShooterState.STOPPED);
        if (isTaunting) {
          setState(RobotState.DEFAULT);
        }
        break;
      case SCORING:
        if (shooter.isAligned()) {
          hopper.index(); 
        } else {
          hopper.outdex();
        }
        intake.setState(isTaunting ? IntakeState.TAUNTING : IntakeState.EXTENDED);
        shooter.setState(ShooterState.SCORING);;
        break;
      case STATIC_SHOOTING:
        if (shooter.isAligned()) {
          hopper.index(); 
        } else {
          hopper.outdex();
        }
        intake.setState(isTaunting ? IntakeState.TAUNTING : IntakeState.EXTENDED);
        shooter.setState(ShooterState.STATIC_SHOOTING);;
        break;
      case PASSING:
        if (shooter.isAligned()) {
          hopper.index(); 
        } else {
          hopper.outdex();
        }
        intake.setState(isTaunting ? IntakeState.TAUNTING : IntakeState.EXTENDED);
        shooter.setState(ShooterState.PASSING);;
        break;
      case OUTTAKING:
        hopper.outtake();
        intake.setState(IntakeState.OUTTAKING);
        shooter.setState(ShooterState.STOPPED);
        break;
      case UNJAMMING:
        hopper.stop();
        intake.setState(IntakeState.FORCE_OUTTAKE);
        shooter.setState(ShooterState.STOPPED);
        break;
      case STOPPED:
        hopper.stop();
        intake.setState(IntakeState.STOPPED);
        shooter.setState(ShooterState.STOPPED);
        break;
      case PRESPIN:
        hopper.stop();
        intake.setState(IntakeState.STOPPED);
        shooter.setState(ShooterState.PRESPIN);
        break;
    }
  }

  public void whileTrue(Trigger trigger, RobotState onTrue) {
    trigger.whileTrue(Commands.startEnd(
      () -> setState(onTrue),
      () -> setState(RobotState.DEFAULT),
      this
    ));
  }

  public void onTrue(Trigger trigger, RobotState onTrue) {
    trigger.onTrue(Commands.runOnce(
      () -> setState(onTrue),
      this
    ));
  }

  public void toggleOnTrue(Trigger trigger, RobotState onTrue) {
    trigger.toggleOnTrue(Commands.startEnd(
      () -> setState(onTrue),
      () -> setState(RobotState.DEFAULT),
      this
    ));
  }

  public void setState(RobotState robotState) {
    this.currentState = robotState;
  }

  public void setTaunting(boolean isTaunting) {
    this.isTaunting = isTaunting;
  }

  public void bindDrivePower(DoubleSupplier driveX, DoubleSupplier driveY) {
    this.driveX = driveX;
    this.driveY = driveY;
  }

  public void bindUnjamPower(DoubleSupplier unjamPower) {
    this.unjamPower = unjamPower;
  }

  public void bindStopAll(Trigger trigger) {
    reqStop = trigger;
  }

  public void configureStateTriggers() {
    aligningTrigger.onTrue(Commands.runOnce(() -> {
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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    stateLoop();
  }
}
