// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Superstructure.RobotState;
import frc.robot.util.NetworkTables.IntakeTable;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreWithTaunt extends ParallelCommandGroup {
  /** Creates a new ScoreWithTaunt. */
  public ScoreWithTaunt(Superstructure superstructure, SwerveSubsystem swerve, Vision vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.startEnd(
        () -> superstructure.setState(RobotState.ALIGNING),
        () -> superstructure.setState(RobotState.DEFAULT)
      ),
      Commands.sequence(
        Commands.waitSeconds(IntakeTable.kTauntDelay.get()),
        Commands.startEnd(
          () -> superstructure.setTaunting(true),
          () -> superstructure.setTaunting(false)
        )
      )
    );
  }
}
