// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeWheels extends SubsystemBase {

  /** IntakeWheels Motor */
  private TalonFXS wheelMotor = new TalonFXS(Constants.IntakeConstants.kWheelMotor);

  /** LED Strip 1 */
  private VictorSPX LEDone = new VictorSPX(Constants.IntakeConstants.kLEDONECANID);

  /** LED Strip 2 */
  private TalonSRX LEDtwo = new TalonSRX(Constants.IntakeConstants.kLEDTWOCANID);

  /** Incremented each frame in order to time led flashing. */
  private int frameCounter = 0;

  /** Stores the most recent power setpoint applied to the IntakeWheels.*/
  private double currentPowerGoal = 0;

  /** Creates a new Intake. */
  public IntakeWheels() {
    TalonFXSConfiguration talonFXSConfigs = new TalonFXSConfiguration();
    talonFXSConfigs.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
    wheelMotor.getConfigurator().apply(talonFXSConfigs);
  }

  /**
   * Sets the intake wheels to the given power. Positive is intaking fuel.
   * 
   * @param power the power to run the intake wheels.
   */
  public void setPower(double power) {
    this.currentPowerGoal = -power;
    wheelMotor.set(currentPowerGoal);
  }

  /** Deactivates the intake wheels motor. */
  public void stop() {
    setPower(0);
  }

  /**
   * Gets the current velocity of the intake wheels via the sensors on its motor.
   * Positive is intaking fuel. Returns zero if the sensors are disconected.
   * 
   * @return the current velocity (rotations per second)
   */
  public double getVelocity() {
    if (!wheelMotor.isConnected()) {
      return 0;
    }
    return -wheelMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Sets both LED strips to the given power.
   *
   * @param power the power to set the LED strips
   */
  private void setLeds(double power) {
    if (power < 0) {
      System.out.println("ERROR: Don't Set LEDS to absorb light!!!");
    } else {
      LEDone.set(ControlMode.PercentOutput, power);
      LEDtwo.set(ControlMode.PercentOutput, power);
    }
  }

  /**
   * Updates the LED indicators based on the current power setpoint and the
   * current velocity of the intake wheels.
   */
  public void updateLeds() {
    if(currentPowerGoal != 0){
      if (Math.abs(getVelocity()) > .5) {
        //Flash
        if ((frameCounter % 25) < 12.5) { // 1/2 seconds flash
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
    frameCounter++;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateLeds();
  }
}
