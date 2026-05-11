// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeWheels extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFXS wheelMotor = new TalonFXS(Constants.IntakeConstants.kWheelMotor); //Needs to be inverted
  private VictorSPX LEDone = new VictorSPX(Constants.IntakeConstants.kLEDONECANID);
  private TalonSRX LEDtwo = new TalonSRX(Constants.IntakeConstants.kLEDTWOCANID);
  private int timer = 0;

  private double currentPowerGoal = 0;

  public IntakeWheels() {
    TalonFXSConfiguration talonFXSConfigs = new TalonFXSConfiguration();
    talonFXSConfigs.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
    wheelMotor.getConfigurator().apply(talonFXSConfigs);
  }

  //+: Fuel go in robot, -: Fuel go out robot
  public void setPower(double power) {
    this.currentPowerGoal = -power;
    wheelMotor.set(currentPowerGoal);
  }

  public void stop() {
    setPower(0);
  }

  public Command setPowerCommand(DoubleSupplier power) {
    return Commands.runEnd(
      () -> setPower(power.getAsDouble()),
      () -> stop(),
      this
    ).withName(
      "Set Intake Wheel Power"
    );
  }

  public double getVelocity() {
    if (!wheelMotor.isConnected()) {
      return 0;
    }
    return -wheelMotor.getVelocity().getValueAsDouble();
  }

  private void setLeds(double power) {
    if (power < 0) {
      System.out.println("ERROR: Don't Set LEDS to absorb light!!!");
    } else {
      LEDone.set(ControlMode.PercentOutput,power);
      LEDtwo.set(ControlMode.PercentOutput,power);
    }
  }

  public void updateLeds() {
    if(currentPowerGoal != 0){
      if (Math.abs(getVelocity()) > .5) {
        //Flash
        if ((timer % 25) < 12.5) { // 1/2 seconds flash
          setLeds(1);
        } else {
          setLeds(0);
        }

      } else {
        //Solid
        setLeds(1);
      }
    } else {
      //Off
      setLeds(0);
    }
    timer++;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateLeds();
  }
}
