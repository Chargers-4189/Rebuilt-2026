// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {

  //Indexer motor gets ball from hopper and ready to shoot
  private final TalonFXS indexerMotor = new TalonFXS(
    IndexerConstants.kMotorCanID
  );  

  public Indexer() {}

  public void setIndexerPower(double indexerMotorPower) {
    indexerMotor.set(-indexerMotorPower);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
