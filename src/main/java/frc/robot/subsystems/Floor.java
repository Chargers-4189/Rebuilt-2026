// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Floor extends SubsystemBase {

  /** Floor Motor */
  private TalonFXS floorMotor = new TalonFXS(Constants.FloorConstants.kMotorRight);

  /** Creates a new Floor. */
  public Floor() {
    TalonFXSConfiguration talonFXSConfigs = new TalonFXSConfiguration();
    talonFXSConfigs.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
    floorMotor.getConfigurator().apply(talonFXSConfigs);
  }

  /**
   * Sets the floor to the given power. Positive is indexing towards the shooter.
   * 
   * @param power the power to run the Floor.
   */
  public void setPower(double power) {
    floorMotor.set(-power);
  }

  /** Deactivates the floor motors. */
  public void stop() {
    setPower(0);
  }

  @Override
  public void periodic() {}
}
