// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.util.NetworkTables.HoodTable;
import edu.wpi.first.math.MathUtil;

public class Hood extends SubsystemBase {

  /** Hood Motor */
  private final TalonFXS hoodMotor = new TalonFXS(HoodConstants.kMotorCanID);

  /** Hood Encoder */
  private final CANcoder hoodEncoder = new CANcoder(HoodConstants.kEncoderID);

  /** PID Controller for the Hood. */
  private final PIDController hoodController = new PIDController(
    HoodTable.kP.get(),
    HoodTable.kI.get(),
    HoodTable.kD.get()
  );

  /** Stores the most recent angle setpoint applied to the hood.*/
  private double hoodGoal = 0;

  /** Creates a new Hood. */
  public Hood() {
    TalonFXSConfiguration talonFXSConfigs = new TalonFXSConfiguration();
    talonFXSConfigs.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
    talonFXSConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    hoodMotor.getConfigurator().apply(talonFXSConfigs);
    
    hoodEncoder.getConfigurator().apply(HoodConstants.kHoodEncoderConfigs);
  }

  /**
   * Sets the hood motor to the given power. Positive is shooting out of the robot.
   * 
   * @param power the power to run the flywheel.
   */
  public void setPower(double power) {
    hoodMotor.set(-power);
  }

  /** Returns the hood to its default angle. */
  public void home() {
    setAngle(HoodTable.kDefaultAngle.get());
  }

  /** Deactivates the hood motor. */
  public void stop() {
    setPower(0);
  }

  /**
   * Sets the hood to the given angle via PID. Positive is moving the hood
   * upwards. Zero is the lowest possible position of the hood.
   * 
   * @param velocity the new angle (rotations)
   */
  public void setAngle(double angle) {
    hoodGoal = angle;
    if (hoodEncoder.isConnected()) {
      hoodMotor.set(
        -MathUtil.clamp(hoodController.calculate(
          getPosition(),
          angle
        ),
        -HoodConstants.kAutoPower,
        HoodConstants.kAutoPower)
      );
    } else {
      setPower(0);
      System.out.println("ERROR: Hood Encoder Disconnected");
    }
  }

  /**
   * Gets the current angle of the hood via the hood encoder. Positive is moving
   * the hood upwards. Zero is the lowest possible position of the hood.
   * 
   * @return the current angle (rotations)
   */
  public double getPosition() {
    return hoodEncoder.getAbsolutePosition().getValueAsDouble();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    HoodTable.hoodGoal.set(hoodGoal);
    HoodTable.hoodEncoder.set(getPosition());
    
    hoodController.setPID(
      HoodTable.kP.get(),
      HoodTable.kI.get(),
      HoodTable.kD.get()
    );
  }
}