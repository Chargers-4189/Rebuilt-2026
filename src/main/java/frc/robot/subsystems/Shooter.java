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

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  //Shooter motor shoots balls upwards from the hopper and indexer motor
  private final TalonFXS leftShooterMotor = new TalonFXS( 
    ShooterConstants.kLeftMotorCanID 
  );
  private final TalonFXS rightShooterMotor = new TalonFXS( 
    ShooterConstants.kRightMotorCanID 
  );
 
  public Shooter() {}


  public void setShooterPower(double shooterMotorPower) {
    leftShooterMotor.set(shooterMotorPower);
    rightShooterMotor.set(-shooterMotorPower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
