// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Floor extends SubsystemBase {
  private TalonFXS rightMotor = new TalonFXS(Constants.FloorConstants.kMotorRight);

  /** Creates a new Floor. */
  public Floor() {
    TalonFXSConfiguration talonFXSConfigs = new TalonFXSConfiguration();
    talonFXSConfigs.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
    rightMotor.getConfigurator().apply(talonFXSConfigs);
  }

  //Positive: Feed Into Robot (EAT)
  //Negative: Feed Out of Robot (VOMIT)
  public void setPower(double power) {
    rightMotor.set(-power);
  }

  public void stop() {
    setPower(0);
  }

  public Command setPowerCommand(DoubleSupplier power) {
    return Commands.runEnd(
      () -> setPower(power.getAsDouble()),
      () -> stop(),
      this
    ).withName("Set Floor Power");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
