// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.choreo.ChoreoTraj;
import frc.robot.handlers.Manager;
import frc.robot.handlers.Manager.RobotState;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OutpostThenDepot extends SequentialCommandGroup {
  /** Creates a new AutoShootOurSide. */
  public OutpostThenDepot(Manager manager, SwerveSubsystem swerve, Vision vision) {
    // Add your commands in the addCommands() call, e.g.

    addCommands(
      Commands.runOnce(() -> manager.setState(RobotState.INTAKING)),
      swerve.choreoAuto(ChoreoTraj.outpostThenDepot$0, true),
      Commands.waitSeconds(4),
      swerve.choreoAuto(ChoreoTraj.outpostThenDepot$1, false),
      Commands.runOnce(() -> manager.setState(RobotState.DEFAULT)),
      new ScoreWithTaunt(manager, swerve, vision) 
    );
  }
}
