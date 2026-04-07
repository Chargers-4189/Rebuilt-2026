// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;

import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.NetworkTables.IntakeTable;

public class IntakeWheels extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFXS wheelMotor = new TalonFXS(Constants.IntakeConstants.kWheelMotor); //Needs to be inverted
  private VictorSP LEDone = new VictorSP(Constants.IntakeConstants.kLEDONECANID);
  private PWMTalonSRX LEDtwo = new PWMTalonSRX(Constants.IntakeConstants.kLEDTWOCANID);
  private int timer = 0;



  public IntakeWheels() {
    TalonFXSConfiguration talonFXSConfigs = new TalonFXSConfiguration();
    talonFXSConfigs.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
    wheelMotor.getConfigurator().apply(talonFXSConfigs);
  }

  //+: Fuel go in robot, -: Fuel go out robot
  public void setWheelPower(double power) {
    wheelMotor.set(-power);
  }

  public double getWheelPower() {
    return wheelMotor.get();
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


  public void setDriverLight() {
    if(getWheelPower() > 0.1){
      if ((timer % 25) < 12.5) { // 1/2 seconds flash
        LEDone.set(1);
        LEDtwo.set(1);
      } else {
        LEDone.set(0);
        LEDtwo.set(0);
      }
    }else{
      LEDone.set(0);
      LEDtwo.set(0);    
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setDriverLight();
    timer++;
  }
}
