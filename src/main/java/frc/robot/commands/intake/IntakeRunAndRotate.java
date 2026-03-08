// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.util.NetworkTables.IntakeTable;
import frc.robot.util.OffsetEncoder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeRunAndRotate extends Command {
  /**
  * @author Jack Koster
  * 
  * @param intake Subsystem
  * @param rotateOut True if its going out of the bot, false if going into the bot
  * 
  * @constants kIntakeAxisOuterLimit, kIntakeAxisInnerLimit, kIntakeAxisSpeed
  * 
  * This is the command that rotates the intake in and out of the bot.
  * The signs (<, >. etc) will need to be changed based on the values of both the Inner and Outer limit.
  */

  private Intake intake;
  private boolean rotateOut;
  private DoubleSupplier angle;
  private DoubleSupplier power;

  private OffsetEncoder offsetEncoder;

  /** Creates a new IntakeRotate. */
  public IntakeRunAndRotate(Intake intake, DoubleSupplier angle, DoubleSupplier power) {
    this.intake = intake;
    this.offsetEncoder = intake.getOffsetEncoder();
    this.angle = angle;
    this.power = power;
    addRequirements(intake);
  }
    /** Creates a new IntakeRotate. */
  public IntakeRunAndRotate(Intake intake, boolean rotateOut, DoubleSupplier power) {
    this(intake, rotateOut ? IntakeTable.kOuterExtensionLimit: IntakeTable.kInnerExtensionLimit, power);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!rotateOut){
      intake.setExtensionAngle(angle.getAsDouble());
    }
    intake.setWheelPower(power.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setExtensionPower(0);
    intake.setWheelPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
