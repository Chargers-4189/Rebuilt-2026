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

public class Indexer extends SubsystemBase {

  //Indexer motor gets ball from hopper and ready to shoot
  private final TalonFXS indexerMotor = new TalonFXS(
    -1 //CHANGE ID HERE, cement ID then add to constants file
  );  



  public Indexer() {}


  public void setIndexerPower(double indexerMotorPower) {
    indexerMotor.set(indexerMotorPower);  // DIRECTION UNTESTED
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
