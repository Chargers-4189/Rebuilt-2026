// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.NetworkTables.IntakeTable;
import frc.robot.subsystems.IntakeExtender;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.util.Stopwatch;

public class Intake extends SubsystemBase {

  /** Extender Subsystem - Fully controlled by the Intake. */
  private IntakeExtender extender;
  /** Wheels Subsystem - Fully controlled by the Intake. */
  private IntakeWheels wheels;

  /** Current State of the Intake.*/
  private IntakeState intakeState = IntakeState.STOPPED;

  /** The Event Loop for State-Activated Triggers. */
  private EventLoop stateEventLoop;

  /** Triggers when the intake is taunting. */
  private Trigger tauntTrigger = new Trigger(stateEventLoop, () -> intakeState == IntakeState.TAUNTING);
  private Stopwatch tauntTimer = new Stopwatch();

  /** Creates a new Intake. */
  public Intake() {
    extender = new IntakeExtender();
    wheels = new IntakeWheels();
    configureStateTriggers();
  }

  /** An enum for representing states the intake could be in. */
  public enum IntakeState {
    INTAKING,
    OUTTAKING,
    TAUNTING,
    EXTENDED,
    RETRACTED,
    FORCE_OUTTAKE,
    FORCE_INTAKE,
    STOPPED
  }

  /**
   * Configures any state triggers that should activate when the intake switches
   * between states. Helpful for integrating Commands into the state machine.
   */
  private void configureStateTriggers() {
    tauntTrigger.onTrue(Commands.runOnce(() -> tauntTimer.start()));
  }

  /**
   * Sets the state of the intake to the given state.
   * 
   * @param intakeState the new state
   */
  public void setState(IntakeState intakeState) {
    this.intakeState = intakeState;
  }

  /**
   * Returns the intake's current state.
   * 
   * @return the current state
   */
  public IntakeState getState() {
    return intakeState;
  }

  /**
   * Calculates the appropriate angle to taunt the intake.
   *
   * @return the appropriate angle setpoint
   */
  private double calculateTauntAngle() {
    return IntakeTable.kTauntMagnitude.getAsDouble() * Math.sin(2 * Math.PI * tauntTimer.getElapsedTime() * IntakeTable.kTauntFrequency.getAsDouble());
  }

  /**
   * Returns true if the intake is aligned to its setpoint.
   *
   * @return whether the intake is aligned
   */
  public boolean isAligned() {
    return extender.isAligned();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    stateEventLoop.poll();
    switch (intakeState) {
      case INTAKING:
        if (extender.isAligned()) {
          extender.stop();
          wheels.setPower(IntakeTable.kWheelPower.get());
        } else {
          extender.setAngle(IntakeTable.kOuterExtensionLimit.get());
          wheels.stop();
        }
        break;
      case OUTTAKING:
        if (extender.isAligned()) {
          extender.stop();
          wheels.setPower(-IntakeTable.kWheelPower.get());
        } else {
          extender.setAngle(IntakeTable.kOuterExtensionLimit.get());
          wheels.stop();
        }
        break;
      case TAUNTING:
        extender.setAngle(calculateTauntAngle());
        wheels.setPower(IntakeTable.kLowWheelPower.get());
        break;
      case EXTENDED:
        if (!extender.isAligned()) {
          extender.setAngle(IntakeTable.kOuterExtensionLimit.get());
        }
        wheels.stop();
        break;
      case RETRACTED:
        if (!extender.isAligned()) {
          extender.setAngle(IntakeTable.kInnerExtensionLimit.get());
        }
        wheels.stop();
        break;
      case FORCE_OUTTAKE:
        extender.stop();
        wheels.setPower(-IntakeTable.kWheelPower.getAsDouble());
        break;
      case FORCE_INTAKE:
        extender.stop();
        wheels.setPower(IntakeTable.kWheelPower.getAsDouble());
        break;
      case STOPPED:
        extender.stop();
        wheels.stop();
        break;
    }
  }
}
