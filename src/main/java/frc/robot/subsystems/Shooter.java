// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;
import frc.robot.util.NetworkTables.ShooterTable;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  //Shooter motor shoots balls upwards from the hopper and indexer motor
  private final TalonFXS leftShooterMotor = new TalonFXS( 
    ShooterConstants.kLeftMotorCanID 
  );
  private final TalonFXS rightShooterMotor = new TalonFXS( 
    ShooterConstants.kRightMotorCanID 
  );

  private TalonFXSConfiguration talonFXSConfigs;
  private Slot0Configs slot0Configs;
  private MotionMagicConfigs motionMagicConfigs;
  private final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

  private double targetVelocity;
 
  public Shooter() {
    ConfigureMotor();
  }

  public void ConfigureMotor() {
    talonFXSConfigs = new TalonFXSConfiguration();
    talonFXSConfigs.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
    // set slot 0 gainss
    slot0Configs = talonFXSConfigs.Slot0;
    slot0Configs.kS = ShooterTable.kS.get(); // Add 0.25 V output to overcome static friction
    slot0Configs.kV = ShooterTable.kV.get(); // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = ShooterTable.kA.get(); // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = ShooterTable.kP.get(); // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = ShooterTable.kI.get(); // no output for integrated error
    slot0Configs.kD = ShooterTable.kD.get(); // no output for error derivative
    // set Motion Magic settings
    motionMagicConfigs = talonFXSConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = ShooterTable.kMotionMagicAcceleration.get(); // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = ShooterTable.kMotionMagicJerk.get(); // Target jerk of 4000 rps/s/s (0.1 seconds)
    leftShooterMotor.getConfigurator().apply(talonFXSConfigs);
    rightShooterMotor.getConfigurator().apply(talonFXSConfigs);
  }

  public void setShooterPower(double shooterMotorPower) {
    //leftShooterMotor.setControl(m_request.withVelocity(shooterMotorPower));
    targetVelocity = shooterMotorPower;
    leftShooterMotor.setControl(m_request.withVelocity(targetVelocity));
    rightShooterMotor.setControl(m_request.withVelocity(-targetVelocity));
  }

  public double getVelocity() {
    //leftShooterMotor.setControl(m_request.withVelocity(shooterMotorPower));
    return (leftShooterMotor.getVelocity().getValueAsDouble());
    //rightShooterMotor.setControl(m_request.withVelocity(-shooterMotorPower));
  }

  public double getTargetVelocity() {
    return targetVelocity;
  }

  public void setShooterPowerNoPID(double shooterMotorPower) {
    leftShooterMotor.set(shooterMotorPower);
    rightShooterMotor.set(-shooterMotorPower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ShooterTable.velocity.set(leftShooterMotor.getVelocity().getValueAsDouble());
    ShooterTable.powerGoal.set(targetVelocity);
  }
}
