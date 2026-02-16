// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Visionsubsystem extends SubsystemBase {

  // todo: name the camera 
  PhotonCamera cameraL = new PhotonCamera("");
  PhotonCamera cameraR = new PhotonCamera("");
  /** Creates a new Visionsubsystem. */
  //42in
  //
  public Visionsubsystem() {
    
    

  }

  public PhotonPipelineResult rightCamBestResult(){

    
    return cameraR.getAllUnreadResults().get(0);

  }

  public PhotonPipelineResult leftCamBestResult(){

    return cameraL.getAllUnreadResults().get(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
