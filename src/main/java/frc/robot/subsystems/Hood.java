// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  private TalonFXS HOOD_MOTOR = new TalonFXS(Constants.HoodConstants.kHoodMotor);
  private DutyCycleEncoder HOOD_ENCODER = new DutyCycleEncoder(Constants.HoodConstants.kHoodEncoder);

  public Hood() {}

  public void setSpeed(double speed) {
    HOOD_MOTOR.set(speed);
  }

  public double getEncoder() {
    return HOOD_ENCODER.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
