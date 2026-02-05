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
  private final SparkMax ShooterMotor = new SparkMax(
    -1, //CHANGE ID HERE, cement ID then add to constants file
    MotorType.     //motor type currently unknown
  );
  //Indexer motor gets ball from hopper and ready to shoot
  private final SparkMax IndexerMotor = new SparkMax(
    -1, //CHANGE ID HERE, cement ID then add to constants file
    MotorType.    //motor type currently unknown
  );
  //Hood Motor rotates the shooting direction vertically
  private final SparkMax HoodMotor = new SparkMax(
    -1, //CHANGE ID HERE, cement ID then add to constants file
    MotorType.    //motor type currently unknown
  );

  private AbsoluteEncoder HoodEncoder = HoodMotor.getAbsoluteEncoder();
  



  public Shooter() {}

  //Hood
  public void setHoodPower(double hoodMotorPower) {
    IndexerMotor.set(hoodMotorPower);  // DIRECTION UNTESTED
  }
  public double getHoodPosition() {
    return HoodEncoder.getPosition(); 
  }

  //Indexer
  public void setIndexerPower(double indexerMotorPower) {
    IndexerMotor.set(indexerMotorPower);  // DIRECTION UNTESTED
  }

  //Shooter
  public void setShooterPower(double shooterMotorPower) {
    ShooterMotor.set(shooterMotorPower);  // DIRECTION UNTESTED
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
