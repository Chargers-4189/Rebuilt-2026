// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  //Shooter motor shoots balls upwards from the hopper and indexer motor
  private final SparkMax shooterMotor = new SparkMax(
    -1, //CHANGE ID HERE, cement ID then add to constants file
    MotorType.     //motor type currently unknown
  );
  //Indexer motor gets ball from hopper and ready to shoot
  private final SparkMax indexerMotor = new SparkMax(
    -1, //CHANGE ID HERE, cement ID then add to constants file
    MotorType.    //motor type currently unknown
  );
  //Hood Motor rotates the shooting direction vertically
  private final SparkMax hoodMotor = new SparkMax(
    -1, //CHANGE ID HERE, cement ID then add to constants file
    MotorType.    //motor type currently unknown
  );

  private AbsoluteEncoder hoodEncoder = hoodMotor.getAbsoluteEncoder();
  



  public Shooter() {}

  //Hood
  public void setHoodPower(double hoodMotorPower) {
    indexerMotor.set(hoodMotorPower);  // DIRECTION UNTESTED
  }
  public double getHoodPosition() {
    return hoodEncoder.getPosition(); 
  }

  //Indexer
  public void setIndexerPower(double indexerMotorPower) {
    indexerMotor.set(indexerMotorPower);  // DIRECTION UNTESTED
  }

  //Shooter
  public void setShooterPower(double shooterMotorPower) {
    shooterMotor.set(shooterMotorPower);  // DIRECTION UNTESTED
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
