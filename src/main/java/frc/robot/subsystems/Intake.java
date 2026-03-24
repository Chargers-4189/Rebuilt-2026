// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.NetworkTables.IntakeTable;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFXS wheelMotor = new TalonFXS(Constants.IntakeConstants.kWheelMotor); //Needs to be inverted
  private TalonFXS extensionMotor = new TalonFXS(Constants.IntakeConstants.kExtenderMotor);
  private CANcoder encoder = new CANcoder(Constants.IntakeConstants.kIntakeEncoder);


  public Intake() {}

  //+: Fuel go in robot, -: Fuel go out robot
  public void setWheelPower(double power) {
    wheelMotor.set(-power);
  }

  //+: Rotates Clockwise (Out), -: Rotates Counterclockwise (In)
  public void setExtensionPower(double power) {
    extensionMotor.set(power);
  }

  public double getEncoder() {
    return encoder.getAbsolutePosition().getValueAsDouble();
  }

  public boolean encoderConnected() {
    return encoder.isConnected();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    IntakeTable.rawEncoder.set(getEncoder());
  }
}
