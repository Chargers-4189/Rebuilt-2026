// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.util.OffsetEncoder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeRotate extends Command {
  /** Creates a new IntakeRotate. */

  /**
  * @author Jack Koster
  * 
  * @param Intake Subsystem
  * @param rotateOut True if its going out of the bot, false if going into the bot
  * 
  * @constants kIntakeAxisOuterLimit, kIntakeAxisInnerLimit, kIntakeAxisSpeed
  * 
  * This is the command that rotates the intake in and out of the bot.
  * The signs (<, >. etc) will need to be changed based on the values of both the Inner and Outer limit.
  */

  private Intake Intake;
  private boolean rotateOut;

  public IntakeRotate(Intake Intake, boolean rotateOut) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Intake = Intake;
    addRequirements();
    this.rotateOut = rotateOut;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Moves out of bot
    if(rotateOut == true && Intake.getEncoder() > Constants.IntakeConstants.kIntakeAxisOuterLimit && Intake.getEncoder() <= Constants.IntakeConstants.kIntakeAxisOuterLimit) {
      Intake.setExtensionSpeed(Constants.IntakeConstants.kIntakeAxisSpeed);
    //Moves into Bot
    }else if(rotateOut == false && Intake.getEncoder() < Constants.IntakeConstants.kIntakeAxisInnerLimit && Intake.getEncoder() >= Constants.IntakeConstants.kIntakeAxisInnerLimit) {
      Intake.setExtensionSpeed(-Constants.IntakeConstants.kIntakeAxisSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.setExtensionSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Should end once the Intake hits a certain point in the Encoder which functions as its limit. - Jack
    if(rotateOut == true && Intake.getEncoder() <= Constants.IntakeConstants.kIntakeAxisOuterLimit) {
      return true;
    }else if(rotateOut == false && Intake.getEncoder() >= Constants.IntakeConstants.kIntakeAxisInnerLimit) {
      return true;
    }
    return false;
  }
}
