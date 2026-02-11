// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX AXIS_MOTOR = new TalonFX(Constants.IntakeConstants.kIntakeAxisMotor);
  private final TalonFX INTAKE_MOTOR = new TalonFX(Constants.IntakeConstants.kIntakeMotor); //Needs to be inverted

  public Intake() {}

  //+: Goes Out, -: Goes in
  public void setAxisSpeed(double speed) {
    AXIS_MOTOR.set(speed);
  }

  //+: Intake, -: Outtake
  public void setIntakeSpeed(double speed) {
    INTAKE_MOTOR.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
