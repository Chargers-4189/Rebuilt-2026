// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFXS AXIS_MOTOR = new TalonFXS(Constants.IntakeConstants.kIntakeAxisMotor);
  private TalonFXS INTAKE_MOTOR = new TalonFXS(Constants.IntakeConstants.kIntakeMotor); //Needs to be inverted

  public Intake() {}

  //+: Goes Out, -: Goes in
  public void setAxisSpeed(double speed) {
    AXIS_MOTOR.set(speed);
  }

  //+: Intake, -: Outtake
  public void setIntakeSpeed(double speed) {
    INTAKE_MOTOR.set(-speed); //Change once inverted
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
