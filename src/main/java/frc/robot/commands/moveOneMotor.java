// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.oneMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class moveOneMotor extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final oneMotor m_subsystem;
  private final CommandXboxController controller;

  /**
   * Creates a new moveOneMotor.
   *
   * @param subsystem The subsystem used by this command.
   */
  public moveOneMotor(oneMotor subsystem, CommandXboxController controller) {
    m_subsystem = subsystem;
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(controller.y().getAsBoolean()){
        m_subsystem.go(0);
      }if(controller.a().getAsBoolean()){
        m_subsystem.go(0.5);
      }if(controller.b().getAsBoolean()){
        m_subsystem.go(0.75);
      }if(controller.x().getAsBoolean()){
        m_subsystem.go(1);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.go(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
