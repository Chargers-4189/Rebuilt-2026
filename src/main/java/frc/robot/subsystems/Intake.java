// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.NetworkTables.IntakeTable;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFXS wheelMotor = new TalonFXS(Constants.IntakeConstants.kIntakeMotor); //Needs to be inverted
  private TalonFXS extensionMotor = new TalonFXS(Constants.IntakeConstants.kIntakeAxisMotor);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.IntakeConstants.kIntakeEncoder, 0, 1);

  /*
   *   * @param gearbox The type of and number of motors in the arm gearbox.
   * @param gearing The gearing of the arm (numbers greater than 1 represent reductions).
   * @param jKgMetersSquared The moment of inertia of the arm; can be calculated from CAD software.
   * @param armLengthMeters The length of the arm.
   * @param minAngleRads The minimum angle that the arm is capable of.
   * @param maxAngleRads The maximum angle that the arm is capable of.
   * @param simulateGravity Whether gravity should be simulated or not.
   * @param startingAngleRads The initial position of the Arm simulation in radians.
   * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
   *     noise is desired. If present must have 1 element for position.
   */
  private SingleJointedArmSim sim = new SingleJointedArmSim(
    DCMotor.getNEO(1),
    15,
    40,
    .5,
    0,
    Math.PI,
    true,
    0,
    1
  );

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
    return MathUtil.inputModulus(encoder.get() + IntakeTable.kEncoderOffset.get(), 0, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    IntakeTable.rawEncoder.set(encoder.get());
    IntakeTable.offsetEncoder.set(getEncoder());
  }
}
