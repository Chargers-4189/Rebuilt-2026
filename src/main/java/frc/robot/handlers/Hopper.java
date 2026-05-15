// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Indexer;
import frc.robot.util.NetworkTables.HopperTable;
import frc.robot.util.NetworkTables.IndexerTable;

public class Hopper extends SubsystemBase {

  /** Indexer Subsystem - Fully Controller by the Hopper. */
  private Indexer indexer;
  /** Floor Subsystem - Fully Controller by the Hopper. */
  private Floor floor;

  /** Current State of the Intake.*/
  private HopperState hopperState;

  /** Creates a new Hopper. */
  public Hopper() {
    this.indexer = new Indexer();
    this.floor = new Floor();
  }

  /** An enum for representing states the hopper could be in. */
  public enum HopperState {
    STOPPED,
    INDEXING,
    OUTDEXING,
    OUTTAKING
  }

  /**
   * Sets the state of the hopper to the given state.
   * 
   * @param hopperState the new state
   */
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
