// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

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

  private final TalonFXS hoodMotor = new TalonFXS(
    HoodConstants.kMotorCanID
  );

  private final CANcoder hoodEncoder = new CANcoder(HoodConstants.kEncoderID);

  private final PIDController hoodController = new PIDController(
    HoodTable.kP.get(),
    HoodTable.kI.get(),
    HoodTable.kD.get()
  );

  private double hoodGoal = 0;

  public Hood() {
    TalonFXSConfiguration talonFXSConfigs = new TalonFXSConfiguration();
    talonFXSConfigs.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
    talonFXSConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    hoodMotor.getConfigurator().apply(talonFXSConfigs);
    
    hoodEncoder.getConfigurator().apply(HoodConstants.kHoodEncoderConfigs);
  }

  public void setPower(double power) {
    hoodMotor.set(-power);
  }

  public void home() {
    setAngle(HoodTable.kDefaultAngle.get());
  }

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