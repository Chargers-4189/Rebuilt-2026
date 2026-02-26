// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.util.ScoringCalculator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignHoodAndFlywheel extends Command {

  private Hood hood;
  private Shooter shooter;
  private Vision vision;

  private double angle;
  private double power;
  
  /** Creates a new AlignHoodAndFlywheel. */
  public AlignHoodAndFlywheel(Hood hood, Shooter shooter, Vision vision) {
    this.hood = hood;
    this.shooter = shooter;
    this.vision = vision;
    addRequirements(hood, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = vision.getDistanceFromHub();
    //double distance = ShooterTable.kDISTANCE.get();
    angle = ScoringCalculator.calculateHoodAngle(distance);
    power = ScoringCalculator.calculateShootingPower(distance);

    //System.out.println(distance + " " + angle + " " + power);

    hood.setHoodAngle(angle);
    shooter.setVelocity(power);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setVelocity(0);
    hood.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
