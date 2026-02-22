// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */
  //Hopper has 2 motors and 0 sensors
  //private TalonFXS LEFT_MOTOR = new TalonFXS(Constants.HopperConstants.kHopperMotorLeft); //Leader
  private TalonFXS RIGHT_MOTOR = new TalonFXS(Constants.HopperConstants.kHopperMotorRight); //Follower (Set inverted)

  public Hopper() {
    
  }

  //Positive: Feed Into Robot (EAT)
  //Negative: Feed Out of Robot (VOMIT)
  public void setSpeed(double speed) {
    //LEFT_MOTOR.set(speed);
    RIGHT_MOTOR.set(-speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
