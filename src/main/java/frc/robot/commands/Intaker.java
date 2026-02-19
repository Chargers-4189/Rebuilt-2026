// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Intaker extends Command {
  /** Creates a new Intaker. */

  /**
   * @author Jack Koster
   * 
   * @param Intake Subsystem
   * @param isIntaking True if the robot is gathering (Intaking) balls, False if its depositing balls (Vomiting).
   * 
   * @constants kIntakeSpeed
   * 
   * This is the command that moves the motors in the Intake that intake / outtake the balls.
   */

  private Intake Intake;
  private boolean isIntaking;
  
  public Intaker(Intake Intake, boolean isIntaking) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Intake = Intake;
    addRequirements(Intake);
    this.isIntaking = isIntaking;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isIntaking == true) {
      Intake.setIntakeSpeed(Constants.IntakeConstants.kIntakeSpeed);
    }else if(isIntaking == false) {
      Intake.setIntakeSpeed(-Constants.IntakeConstants.kIntakeSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.setIntakeSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //I dont know/have a good at the time of making this to end this command - Jack
    return false;
  }
}
