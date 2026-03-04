// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCenterCollectAndShoot extends SequentialCommandGroup {
  /** Creates a new AutoCenterCollectAndShoot. */
  public AutoCenterCollectAndShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
            Command path;
    try {
        path = AutoBuilder.followPath(PathPlannerPath.fromPathFile("centerCollectAndShoot"));
    } catch(Exception e){
        path = Commands.none();
    }
    addCommands(path);
  }
}
