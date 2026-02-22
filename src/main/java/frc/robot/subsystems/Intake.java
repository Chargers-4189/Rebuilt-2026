// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFXS WheelMotor = new TalonFXS(Constants.IntakeConstants.kIntakeMotor); //Needs to be inverted
  private DutyCycleEncoder Encoder = new DutyCycleEncoder(Constants.IntakeConstants.kIntakeEncoder);
  private TalonFXS ExtensionMotor = new TalonFXS(Constants.IntakeConstants.kIntakeAxisMotor);

  public Intake() {}

  //+: BALLS GO INTO ROBOT, -: BALLS GO OUT OF ROBOT
  public void setWheelSpeed(double speed) {
    WheelMotor.set(-speed); //Change once inverted
  }

  //+: ROTATES CW (OUT), -: ROTATES CCW (IN)
  public void setExtensionSpeed(double speed) {
    ExtensionMotor.set(speed);
  }

  public double getEncoder() {
    return Encoder.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(getEncoder());
  }
}
