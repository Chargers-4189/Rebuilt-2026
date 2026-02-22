// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeTaunt extends Command {
  /** Creates a new IntakeTaunt. */

  /**
   * @author Jack Koster
   * 
   * @param Intake Subsystem
   * 
   * @constants kTauntFraction, KTauntAmount 
   * 
   * The this command was made per request of Byran.
   * This is unecessary for the function of the robot but its funny.
   * It is effective a 'taunt' or 'emote', the bot will raise and lower the intake like its putting its hands up and down.
   * This file is untested so please dont use it.
   */

  private Intake Intake;
  private boolean inPositiion = false;
  private int loops;

  public IntakeTaunt(Intake Intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Intake = Intake;
    addRequirements(Intake);
    this.loops = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //This code makes sure its in the correct position before getting 'silly'
    if(Intake.getEncoder() <= Constants.IntakeConstants.kIntakeAxisOuterLimit && inPositiion == false) {
      inPositiion = true;
    }else if(Intake.getEncoder() > Constants.IntakeConstants.kIntakeAxisOuterLimit && inPositiion == false) {
      Intake.setExtensionSpeed(Constants.IntakeConstants.kIntakeAxisSpeed);
    }

    //Make it start from the lower area and go up like a 1/4 or a 1/3 inside then back down 2-3 times
    if(inPositiion == true && loops < (Constants.IntakeConstants.kTauntAmount * 2)) {

      //Going inside robot
      if(loops % 2 == 0) {
        Intake.setExtensionSpeed(1.0);
        //This kerfuffle of a statement is what makes it go only a fraction of the way up, using only the constant file. (CHANGE THE + / - DEPENDING IF THE INNER IS GREATER THEN THE OUTER)
        if(Intake.getEncoder() >= (Constants.IntakeConstants.kIntakeAxisInnerLimit - (Math.floor(Math.abs(Constants.IntakeConstants.kIntakeAxisOuterLimit - Constants.IntakeConstants.kIntakeAxisInnerLimit) / Constants.IntakeConstants.kTauntFraction)))) {
          Intake.setExtensionSpeed(0.0);
          loops++;
        }
      //Going outside the robot
      }else if(loops % 2 == 1) {
        Intake.setExtensionSpeed(-1.0);
        if(Intake.getEncoder() <= Constants.IntakeConstants.kIntakeAxisOuterLimit) {
          Intake.setExtensionSpeed(0.0);
          loops++;
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //if the command is interrupted you need to manually move the intake back into place.
    Intake.setExtensionSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(loops == (Constants.IntakeConstants.kTauntAmount * 2)) {
      return true;
    }else {
      return false;
    }
  }
}
