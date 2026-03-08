// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.IntakeRotate;
import frc.robot.commands.intake.IntakeRunAndRotate;
import frc.robot.commands.intake.RunIntakeWheels;
import frc.robot.commands.scoring.Score;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.util.NetworkTables.IntakeTable;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCenterCollectWOInterferance extends ParallelCommandGroup {
  /** Creates a new AutoCenterCollectAndShootFullPath. */
  public AutoCenterCollectWOInterferance(Shooter shooter, Hood hood, Indexer indexer, SwerveSubsystem swerve, Vision vision, Hopper hopper, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Command path;
    try {
        path = AutoBuilder.followPath(PathPlannerPath.fromPathFile("centerCollectWOInterfereance"));
    } catch(Exception e){
        path = Commands.none();
    }
    addCommands(new IntakeRotate(intake, false), path.withTimeout(1.5), new RunIntakeWheels(intake, IntakeTable.kAutoInPower), new Score(shooter, hood, indexer, swerve, vision, hopper, intake));
  }
}
