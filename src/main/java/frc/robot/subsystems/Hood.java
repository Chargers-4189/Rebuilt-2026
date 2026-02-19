// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.util.NetworkTables.HoodTable;
import edu.wpi.first.math.MathUtil;

public class Hood extends SubsystemBase {
 
  public double offset = 0;

  private final TalonFXS hoodMotor = new TalonFXS(
    HoodConstants.kMotorCanID
  );

  private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(0);   //Change channel after looking at wiring later
  private final PIDController m_hoodFeedback = new PIDController(
    HoodTable.kP.get(),
    HoodTable.kI.get(),
    HoodTable.kD.get()
  );

  
  // private TalonFXSConfiguration talonFXSConfigs;
  // private Slot0Configs slot0Configs;
  // private MotionMagicConfigs motionMagicConfigs;
  // private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
 


  public Hood() {
   // m_hoodFeedback.enableContinuousInput(0, 1);
    //ConfigureMotor();
    //m_hoodFeedback.setTolerance(.1);
  }

  // public void ConfigureMotor() {
  //   talonFXSConfigs = new TalonFXSConfiguration();
  //   talonFXSConfigs.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
  //   // set slot 0 gainss
  //   slot0Configs = talonFXSConfigs.Slot0;
  //   slot0Configs.kS = HoodTable.kS.get(); // Add 0.25 V output to overcome static friction
  //   slot0Configs.kP = HoodTable.kP.get(); // An error of 1 rps results in 0.11 V output
  //   slot0Configs.kI = HoodTable.kI.get(); // no output for integrated error
  //   slot0Configs.kD = HoodTable.kD.get(); // no output for error derivative
  //   // set Motion Magic settings
  //   motionMagicConfigs = talonFXSConfigs.MotionMagic;
  //   motionMagicConfigs.MotionMagicAcceleration = HoodTable.MotionMagicAcceleration.get(); // Target acceleration of 400 rps/s (0.25 seconds to max)
  //   motionMagicConfigs.MotionMagicJerk = HoodTable.MotionMagicJerk.get(); // Target jerk of 4000 rps/s/s (0.1 seconds)
  //   hoodMotor.getConfigurator().apply(talonFXSConfigs);
  // }

  public void setHoodPower(double hoodMotorPower) {
    hoodMotor.set(-hoodMotorPower);
  }

  public double getHoodPosition() {
    return 1 - hoodEncoder.get();
  }

  public void zeroEncoder() {
    System.out.println("ERROR: Use Rev Software to reset this.");
  }

  /*
  public void offsetEncoder() {
    offset = hoodEncoder.get();
  }*/

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

  
  public Command setHoodAngleCommand(DoubleSupplier angle) {
    return Commands.run(
        () -> {
          setHoodAngle(angle.getAsDouble());
        }, this).withName("HoodAlign"); // PID math max clamp at 0.4
  }

  public void setHoodAngle(double angle) {
    hoodMotor.set(-MathUtil.clamp(m_hoodFeedback.calculate((getHoodPosition() + .2) % 1, (angle + .2) % 1),-0.4, 0.4));
    //hoodMotor.setControl(m_request.withPosition(3));
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println(hoodEncoder.get() + " " + getHoodPosition() + offset);
    //System.out.println(hoodEncoder.isConnected());
    //System.out.println("encoder " + getHoodPosition());
    
    m_hoodFeedback.setPID(
      HoodTable.kP.get(),
      HoodTable.kI.get(),
      HoodTable.kD.get()
    );
    //ConfigureMotor();
  }
}