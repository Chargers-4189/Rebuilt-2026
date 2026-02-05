// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootFuelNoSwerveAlign extends Command {
  private final Shooter shooter;
  double speed; //Speed of shooter
  double angle; //position of hood we currently want
  double currentAngle;
  double hoodTolerance = 0.5;  //This line should eventually be removed and it's applications should be replaced with this number in the constants file
  /** Creates a new ShootFuel. */
  public ShootFuelNoSwerveAlign(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = -1; //Lynn, you will determine this number through vision somehow
    currentAngle = shooter.getHoodPosition();
    angle = -1; //Lynn, you will also determine this number through vision somehow


    //sets the hood angle to the desired hood angle, if the hood angle is within the tolerance we set, then it shoots the fuel
    if(Math.abs(angle - currentAngle) >= hoodTolerance){
      if(angle <= currentAngle){
      shooter.setHoodPower(0.05); // this number (currently 0.05) should probably be iterated then put into constants
    }else if(angle <= currentAngle){
      shooter.setHoodPower(-0.05);
    }}else{
      shooter.setIndexerPower(speed); // I cannot see a time where we have the shooter running and we wouldnt want the indexer running. somebody please challenge this line I am not confident at all
      shooter.setShooterPower(speed);
    }

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
