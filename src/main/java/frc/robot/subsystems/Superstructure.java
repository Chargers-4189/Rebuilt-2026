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
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShootingType;
import frc.robot.util.Stopwatch;

public class Superstructure extends SubsystemBase {

  private Vision vision;
  private SwerveSubsystem swerve;
  private Hopper hopper;
  private Intake intake;
  private Shooter shooter;

  private DoubleSupplier driveX;
  private DoubleSupplier driveY;
  private DoubleSupplier unjamPower;

  private RobotState currentState = RobotState.DEFAULT;
  private ShootingType shootingType = ShootingType.SCORE;
  private boolean isTaunting = false;

  private EventLoop stateEventLoop = new EventLoop();

  //private Trigger defaultTrigger = new Trigger(stateEventLoop, () -> robotState == RobotState.DEFAULT);
  //private Trigger tuckedTrigger = new Trigger(stateEventLoop, () -> robotState == RobotState.TUCKED);
  //private Trigger extendedTrigger = new Trigger(stateEventLoop, () -> robotState == RobotState.EXTENDED);
  //private Trigger intakingTrigger = new Trigger(stateEventLoop, () -> robotState == RobotState.INTAKING);
  private Trigger aligningTrigger = new Trigger(stateEventLoop, () -> currentState == RobotState.ALIGNING);
  private Trigger firingTrigger = new Trigger(stateEventLoop, () -> currentState == RobotState.FIRING);
  //private Trigger outtakingTrigger = new Trigger(stateEventLoop, () -> robotState == RobotState.OUTTAKING);
  //private Trigger unjamTrigger = new Trigger(stateEventLoop, () -> robotState == RobotState.UNJAMMING);
  //private Trigger stoppedTrigger = new Trigger(stateEventLoop, () -> robotState == RobotState.STOPPED);
  //private Trigger prespinTrigger = new Trigger(stateEventLoop, () -> robotState == RobotState.PRESPIN);

  private Trigger scoringTrigger = new Trigger(stateEventLoop, () -> {
    return (shootingType == ShootingType.SCORE) && (currentState == RobotState.ALIGNING || currentState == RobotState.FIRING);
  });
  private Trigger passingTrigger = new Trigger(stateEventLoop, () -> {
    return (shootingType == ShootingType.PASS) && (currentState == RobotState.ALIGNING || currentState == RobotState.FIRING);
  });
  //private Trigger staticScoringTrigger = new Trigger(stateEventLoop, () -> {
  //  return (shootingType == ShootingType.STATIC) && (robotState == RobotState.ALIGNING || robotState == RobotState.FIRING);
  //});

  private Trigger reqStop;

  private Stopwatch spinupTime = new Stopwatch();

  /** Creates a new Superstructure. */
  public Superstructure(Vision vision, SwerveSubsystem swerve) {
    this.vision = vision;
    this.swerve = swerve;
    this.hopper = new Hopper();
    this.intake = new Intake();
    this.shooter = new Shooter(vision);

    configureStateTriggers();
  }

  public enum RobotState {
    DEFAULT,
    TUCKED,
    EXTENDING,
    INTAKING,
    ALIGNING,
    FIRING,
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
        if (isTaunting) {
          intake.taunt();
        } else {
          intake.stop();
        }
        shooter.stop();
        break;
      case TUCKED:
        hopper.stop();
        intake.idleIn();
        shooter.stop();
        if (isTaunting) {
          setState(RobotState.DEFAULT);
        }
      case EXTENDING:
        hopper.stop();
        intake.idleOut();
        shooter.stop();
        if (isTaunting || intake.extender.isAligned()) {
          setState(RobotState.DEFAULT);
        }
      case INTAKING:
        hopper.stop();
        intake.intake();
        shooter.stop();
        if (isTaunting) {
          setState(RobotState.DEFAULT);
        }
        break;
      case ALIGNING:
        hopper.outdex();
        if (isTaunting) {
          intake.taunt();
        } else {
          intake.idleOut();
        }
        shooter.align(shootingType);
        if (shooter.flywheel.isAligned()) {
          setState(RobotState.FIRING);
        }
        break;
      case FIRING:
        hopper.index();
        if (isTaunting) {
          intake.taunt();
        } else {
          intake.idleOut();
        }
        shooter.align(shootingType);
        break;
      case OUTTAKING:
        hopper.outtake();
        intake.outtake();
        shooter.stop();
        break;
      case UNJAMMING:
        hopper.stop();
        intake.unjam(unjamPower);
        shooter.stop();
        break;
      case STOPPED:
        hopper.stop();
        intake.stop();
        shooter.stop();
        break;
      case PRESPIN:
        hopper.stop();
        intake.stop();
        shooter.prespin();
        break;
    }
  }

  public void whileTrue(Trigger trigger, RobotState onTrue) {
    trigger.whileTrue(Commands.startEnd(
      () -> setState(onTrue),
      () -> setState(RobotState.DEFAULT)
    ));
  }

  public void whileTrue(Trigger trigger, RobotState onTrue, ShootingType shootingType) {
    trigger.whileTrue(Commands.startEnd(
      () -> setState(onTrue, shootingType),
      () -> setState(RobotState.DEFAULT)
    ));
  }

  public void onTrue(Trigger trigger, RobotState onTrue) {
    trigger.onTrue(Commands.runOnce(
      () -> setState(onTrue)
    ));
  }

  public void toggleOnTrue(Trigger trigger, RobotState onTrue) {
    trigger.toggleOnTrue(Commands.startEnd(
      () -> setState(onTrue),
      () -> setState(RobotState.DEFAULT)
    ));
  }

  public void setState(RobotState robotState) {
    if (robotState == RobotState.ALIGNING || robotState == RobotState.FIRING) {
      System.out.println("WARNING: Shooting Type should be specified.");
    }
    this.currentState = robotState;
  }

  public void setState(RobotState robotState, ShootingType shootingType) {
    if (this.currentState != RobotState.ALIGNING && robotState == RobotState.FIRING) {
      System.out.println("WARNING: Innapropriate state jump to FIRING.");
    }
    this.currentState = robotState;
    this.shootingType = shootingType;
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
