// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.NetworkTables.IntakeTable;
import frc.robot.util.Stopwatch;

public class Intake extends SubsystemBase {

  public Extender extender;
  public Wheels wheels;

  private IntakeState intakeState = IntakeState.STOPPED;

  private EventLoop stateEventLoop;

  private DoubleSupplier unjamPower;

  private Trigger tauntTrigger = new Trigger(stateEventLoop, () -> intakeState == IntakeState.TAUNTING);
  private Stopwatch tauntTimer = new Stopwatch();
  /** Creates a new Intake. */
  public Intake() {
    extender = new Extender();
    wheels = new Wheels();
    configureStateTriggers();
  }

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

  private void configureStateTriggers() {
    tauntTrigger.onTrue(Commands.runOnce(() -> tauntTimer.start()));
  }

  public void setState(IntakeState intakeState) {
    this.intakeState = intakeState;
  }

  public void getState(IntakeState intakeState) {
    this.intakeState = intakeState;
  }

  private double calculateTauntAngle() {
    return IntakeTable.kTauntMagnitude.getAsDouble() * Math.sin(2 * Math.PI * tauntTimer.getElapsedTime() * IntakeTable.kTauntFrequency.getAsDouble());
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
