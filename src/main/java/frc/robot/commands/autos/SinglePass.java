// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.choreo.ChoreoTraj;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Superstructure.RobotState;
import frc.robot.util.NetworkTables.AutoTable;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SinglePass extends SequentialCommandGroup {
  /** Creates a new AutoCenterCollectAndShootFullPath. */
  public SinglePass(Superstructure superstructure, SwerveSubsystem swerve, Vision vision, ChoreoTraj traj, boolean resetOdom, boolean withShooterTimeout) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(
      Commands.runOnce(() -> superstructure.setState(RobotState.INTAKING)),
      Commands.race(
        swerve.choreoAuto(traj, resetOdom),
        Commands.sequence(
          Commands.waitSeconds(MathUtil.clamp(traj.totalTimeSecs() - AutoTable.kPreSpinDuration.get(), .6, 20)),
          Commands.runOnce(() -> {
            vision.activate();
            superstructure.setState(RobotState.PRESPIN);
          })
        )
      ),
      Commands.runOnce(() -> vision.activate(), vision),
      new ScoreWithTaunt(superstructure, swerve, vision).withTimeout(withShooterTimeout ? AutoTable.kShooterTimeout.get() : 6),
      Commands.runOnce(() -> vision.deactivate(), vision)
    );
  }
}
