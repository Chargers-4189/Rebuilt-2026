// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {

  /** Indexer Motor */
  private final TalonFXS indexerMotor = new TalonFXS(IndexerConstants.kMotorCanID);  

  /** Creates a new Indexer. */
  public Indexer() {
    TalonFXSConfiguration talonFXSConfigs = new TalonFXSConfiguration();
    talonFXSConfigs.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
    indexerMotor.getConfigurator().apply(talonFXSConfigs);
  }

  /**
   * Sets the Indexer to the given power. Positive is indexing towards the shooter.
   * 
   * @param power the power to run the Indexer.
   */
  public void setPower(double power) {
    indexerMotor.set(-power);
  }

  /** Deactivates the indexer motors. */
  public void stop() {
    setPower(0);
  }

  @Override
  public void periodic() {}
}
