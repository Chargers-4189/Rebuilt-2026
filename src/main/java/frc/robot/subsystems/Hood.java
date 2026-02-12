// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
 


  private final TalonFXS hoodMotor = new TalonFXS(
    HoodConstants.kMotorCanID
  );

  private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(HoodConstants.kEncoderDIO);
  
  public Hood() {}

  public void setHoodPower(double hoodMotorPower) {
    hoodMotor.set(hoodMotorPower);  // DIRECTION UNTESTED
  }
  public double getHoodPosition() {
    return (hoodEncoder.get()/HoodConstants.kGearRatio);
  }
  public void zeroEncoder() {
    System.out.println("ERROR: Use Rev Software to reset this.");
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
