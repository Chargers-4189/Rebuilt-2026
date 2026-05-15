// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.Floor;
import frc.robot.subsystems.hopper.Indexer;
import frc.robot.util.NetworkTables.HopperTable;
import frc.robot.util.NetworkTables.IndexerTable;

public class Hopper extends SubsystemBase {

  public Indexer indexer;
  public Floor floor;

  public HopperState hopperState;

  /** Creates a new Hopper. */
  public Hopper() {
    this.indexer = new Indexer();
    this.floor = new Floor();
  }

  public enum HopperState {
    STOPPED,
    INDEXING,
    OUTDEXING,
    OUTTAKING
  }

  public void setState(HopperState hopperState) {
    this.hopperState = hopperState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (hopperState) {
      case STOPPED:
        indexer.stop();
        floor.stop();
        break;
      case INDEXING:
        indexer.setPower(IndexerTable.kPower.get());
        floor.setPower(HopperTable.kPower.get());
        break;
      case OUTDEXING:
        indexer.setPower(IndexerTable.kReversePower.get());
        floor.stop();
        break;
      case OUTTAKING:
        indexer.setPower(IndexerTable.kReversePower.get());
        floor.setPower(HopperTable.kReversePower.get());
        break;
    }
  }
}
