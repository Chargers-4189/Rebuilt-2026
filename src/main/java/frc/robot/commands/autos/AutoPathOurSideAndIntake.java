// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.intake.IntakeRunAndRotate2;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.util.NetworkTables.IntakeTable;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPathOurSideAndIntake extends ParallelDeadlineGroup {
  /** Creates a new AutoPathOurSideAndIntake. */
  public AutoPathOurSideAndIntake(Intake intake, SwerveSubsystem swerve, Vision vision) {
    // Add the deadline command in the super() call. Add other commands using addCommands().
    super(
      shootOnlyOnOurSide()
    );
    
    addCommands(
      new IntakeRunAndRotate2(intake, IntakeTable.kWheelPower)
    );
  }

  private static Command shootOnlyOnOurSide() {
    try {
      return AutoBuilder.followPath(PathPlannerPath.fromPathFile("shootOnlyOurSide")).withName("Shoot Only Our Side");
    } catch (Exception e) {
      System.out.println(e.getMessage());
      return Commands.none().withName("Auto Command Not Found");
    }
  }
}
