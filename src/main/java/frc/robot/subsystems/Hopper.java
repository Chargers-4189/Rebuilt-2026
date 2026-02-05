// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */
  //Hopper has 2 motors and 0 sensors
  TalonFX RIGHT_MOTOR = new TalonFX(Constants.HopperConstants.kMOTOR_ID_RIGHT);
  TalonFX LEFT_MOTOR = new TalonFX(Constants.HopperConstants.kMOTOR_ID_LEFT);

  public Hopper() {}

  //Positive: Feed Into Robot (EAT)
  //Negative: Feed Out of Robot (VOMIT)
  public void setSpeed(double speed) {
    RIGHT_MOTOR.set(speed);
    LEFT_MOTOR.set(-speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
