// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.intake.IntakeRunAndRotate;
import frc.robot.subsystems.Intake;
import frc.robot.util.NetworkTables.IntakeTable;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPathOurSideAndIntake extends ParallelDeadlineGroup {
  /** Creates a new AutoPathOurSideAndIntake. */
  public AutoPathOurSideAndIntake(Intake intake) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(getDeadlineCommand());
    
    addCommands(new IntakeRunAndRotate(intake, IntakeTable.kOuterExtensionLimit, IntakeTable.kAutoInPower));
    // addCommands(new FooCommand(), new BarCommand());
  }

  private static Command getDeadlineCommand() {
    try {
      return AutoBuilder.followPath(PathPlannerPath.fromPathFile("shootOnlyOurSide"));
    } catch (Exception e) {
      return Commands.none();
    }
  }
}
