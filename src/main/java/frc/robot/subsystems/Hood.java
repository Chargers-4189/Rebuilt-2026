// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.NetworkTables;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import edu.wpi.first.math.MathUtil;

public class Hood extends SubsystemBase {
 
  public double offset = 0;

  private final TalonFXS hoodMotor = new TalonFXS(
    HoodConstants.kMotorCanID
  );

  private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(0);   //Change channel after looking at wiring later
  private final PIDController m_hoodFeedback = new PIDController(
    NetworkTables.HoodTable.kP.get(),
    NetworkTables.HoodTable.kI.get(),
    NetworkTables.HoodTable.kD.get()
  );


  public Hood() {}

  public void setHoodPower(double hoodMotorPower) {
    hoodMotor.set(hoodMotorPower);  // DIRECTION UNTESTED
  }

  public double getHoodPosition() {
    return (hoodEncoder.get() - offset);
  }

  public void zeroEncoder() {
    System.out.println("ERROR: Use Rev Software to reset this.");
  }

  public void offsetEncoder() {
    offset = (hoodEncoder.get() - .676);
  }

  /**
   * Sets the hood angle to the desired hood angle, if the hood angle is within the tolerance we set, then it shoots the fuel
   * @param angle the angle to go to
   */
  // public void setHoodAngle(double angle) {
  //   if(Math.abs(angle - getHoodPosition()) >= HoodConstants.kHoodTolerance){
  //     if(angle > getHoodPosition()){
  //       setHoodPower(HoodConstants.kHoodPower); 
  //     }else if(angle < getHoodPosition()){
  //       setHoodPower(-HoodConstants.kHoodPower);
  //     }
  //   }
  // }

  public Command SetHoodAngle() {
    return Commands.run(
        () -> {
          hoodMotor.set(MathUtil.clamp(
            m_hoodFeedback.calculate(hoodEncoder.get(), MathUtil.clamp(NetworkTables.HoodTable.kANGLE.get(), 0, 0.676)), -.6, 0.6));
        }, this).withName("HoodAlign"); // PID math max clamp at 0.4
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(getHoodPosition());
    //System.out.println(hoodEncoder.isConnected());
    m_hoodFeedback.setPID(
      NetworkTables.HoodTable.kP.get(),
      NetworkTables.HoodTable.kI.get(),
      NetworkTables.HoodTable.kD.get()
    );
  }
}