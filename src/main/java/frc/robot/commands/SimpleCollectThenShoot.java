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
public class SimpleCollectThenShoot extends SequentialCommandGroup {
  /** Creates a new AutoCenterCollectAndShootFullPath. */
  public SimpleCollectThenShoot(Manager manager, SwerveSubsystem swerve, Vision vision, ChoreoTraj firstPass, ChoreoTraj secondPass, ChoreoTraj thirdPass, int numPasses, boolean resetOdom) {
    if (numPasses == 1) {
      addCommands(
        new SinglePass(manager, swerve, vision, firstPass, resetOdom, false)
      );
    } else if (numPasses == 2) {
      addCommands(
        new SinglePass(manager, swerve, vision, firstPass, resetOdom, true),
        new SinglePass(manager, swerve, vision, secondPass, false, false)
      );
    } else if (numPasses == 3) {
      addCommands(
        new SinglePass(manager, swerve, vision, firstPass, resetOdom, true),
        new SinglePass(manager, swerve, vision, secondPass, false, true),
        Commands.race(
          Commands.startEnd(
            () -> manager.setState(RobotState.INTAKING), 
            () -> manager.setState(RobotState.DEFAULT)
          ),
          swerve.choreoAuto(thirdPass, false)
        )
      );
    }
  }
}
