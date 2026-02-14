// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.util.NetworkTables.ShooterTable;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Score extends Command {

  private Hood hood;
  private Shooter shooter;
  
  /** Creates a new Score. */
  public Score(Hood hood, Shooter shooter) {
    this.hood = hood;
    this.shooter = shooter;
    addRequirements(hood, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dist = ShooterTable.kDISTANCE.get();
    double angle = .66;
    if (dist > 7) {
      angle -= .04*(dist - 7);
    }

    double power = 0.0222112*dist + .0225543*ShooterTable.kFUEL_NUM.get() + .374194*angle + .147292;

    System.out.println(angle + " " + power);
    hood.setHoodAngle(angle);
    shooter.setShooterPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
