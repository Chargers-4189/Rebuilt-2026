// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.NetworkTables.IntakeTable;

public class IntakeWheels extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFXS wheelMotor = new TalonFXS(Constants.IntakeConstants.kWheelMotor); //Needs to be inverted


  public IntakeWheels() {}

  //+: Fuel go in robot, -: Fuel go out robot
  public void setWheelPower(double power) {
    wheelMotor.set(-power);
  }

  public Command runWheelsCommand(DoubleSupplier power) {
    return Commands.run(
      () -> setWheelPower(power.getAsDouble()), this
    ).finallyDo(
      () -> setWheelPower(0)
    ).withName(
      "RunIntakeWheels"
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
