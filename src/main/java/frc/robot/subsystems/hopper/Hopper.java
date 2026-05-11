// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.NetworkTables.HopperTable;
import frc.robot.util.NetworkTables.IndexerTable;

public class Hopper extends SubsystemBase {

  public Indexer indexer;
  public Floor floor;

  /** Creates a new Belly. */
  public Hopper() {
    this.indexer = new Indexer();
    this.floor = new Floor();
  }

  public void stop() {
    indexer.stop();
    floor.stop();
  }

  public void index() {
    indexer.setPower(IndexerTable.kPower.get());
    floor.setPower(HopperTable.kPower.get());
  }

  public void outdex() {
    indexer.setPower(IndexerTable.kReversePower.get());
    floor.stop();
  }

  public void outtake() {
    indexer.setPower(IndexerTable.kReversePower.get());
    floor.setPower(HopperTable.kReversePower.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
