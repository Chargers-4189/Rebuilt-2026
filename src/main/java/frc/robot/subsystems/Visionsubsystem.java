// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class Visionsubsystem extends SubsystemBase {

  private AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  Pose2d hubPosBlue = new Pose2d(Units.feetToMeters(179.56),Units.feetToMeters(158.32),new Rotation2d(0,0));
  Pose2d humPosRed = new Pose2d(Units.feetToMeters(466.56),Units.feetToMeters(158.32),new Rotation2d(0,0));
  // todo: name the camera 
  PhotonCamera leftcamera = new PhotonCamera("");
  Transform3d leftCamTransform = new Transform3d(0,0,0, new Rotation3d(0,0,0));

  PhotonCamera rightCamera = new PhotonCamera("");
  Transform3d rightCamTransform = new Transform3d(0,0,0, new Rotation3d(0,0,0));
  
  TunerSwerveDrivetrain swerve;
  //42in
  //
  public Visionsubsystem(TunerSwerveDrivetrain swerve) {
    this.swerve = swerve;
  }


  public void addVisionMeasurement(PhotonCamera camera, Transform3d robotTransform){
    PhotonPoseEstimator leftPoseEstimator = new PhotonPoseEstimator(layout, robotTransform); 
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    if(!results.isEmpty()){

       Optional<EstimatedRobotPose> leftEstimatedPoseOptional = leftPoseEstimator.estimateAverageBestTargetsPose(results.get(0));
        
      if(leftEstimatedPoseOptional.isPresent()){
        EstimatedRobotPose leftEstimatedPose = leftEstimatedPoseOptional.get();
        swerve.addVisionMeasurement(leftEstimatedPose.estimatedPose.toPose2d(), leftEstimatedPose.timestampSeconds);
      }
    }
  }
  @Override
  public void periodic() {
    this.addVisionMeasurement(leftcamera, leftCamTransform);
    this.addVisionMeasurement(rightCamera, rightCamTransform);
    // This method will be called once per scheduler run
  }
}
