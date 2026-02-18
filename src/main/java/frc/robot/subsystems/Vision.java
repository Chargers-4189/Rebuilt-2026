// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.opencv.core.Mat;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Vision extends SubsystemBase {

  private AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  Pose2d hubPosBlue = new Pose2d(Units.inchesToMeters(179.56),Units.inchesToMeters(158.32),new Rotation2d(0,0));
  Pose2d hubPosRed = new Pose2d(Units.inchesToMeters(466.56),Units.inchesToMeters(158.32),new Rotation2d(0,0));
  // todo: name the camera 
  PhotonCamera leftcamera = new PhotonCamera("LeftCam");
  Transform3d leftCamTransform = new Transform3d(-Units.inchesToMeters(13),-Units.inchesToMeters(3),Units.inchesToMeters(13), new Rotation3d(0,Units.degreesToRadians(-45),0));

  PhotonCamera rightCamera = new PhotonCamera("RightCam");
  Transform3d rightCamTransform = new Transform3d(-Units.inchesToMeters(13),-Units.inchesToMeters(-5.5),Units.inchesToMeters(13), new Rotation3d(0,Units.degreesToRadians(-45),0));
  
  CommandSwerveDrivetrain swerve;
  //42in
  //return 0.0;
  public Vision(CommandSwerveDrivetrain swerve) {
    this.swerve = swerve;
  }

  public void addVisionMeasurement(PhotonCamera camera, Transform3d robotTransform){
    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(layout, robotTransform); 
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    if(!results.isEmpty()){
       Optional<EstimatedRobotPose> estimatedPoseOptional = poseEstimator.estimateAverageBestTargetsPose(results.get(0));
        
      if(estimatedPoseOptional.isPresent()){
        EstimatedRobotPose estimatedPose = estimatedPoseOptional.get();
        swerve.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
      }
    }
  }

  public double getDistanceFromHub(){
    if(swerve.m_isBlueAlliance){
      return getDistanceFromBlueHub();
    } else {
      return getDistanceFromRedHub();
    }
  }
  private double getDistanceFromBlueHub(){
    return getDistanceFromLocation(hubPosBlue);
  }

  private double getDistanceFromRedHub(){
    return getDistanceFromLocation(hubPosRed);
  }

  private double getDistanceFromLocation(Pose2d hubLocation){
      
      double botX = swerve.getState().Pose.getX();
      double botY = swerve.getState().Pose.getY();

      double hubX = hubLocation.getX();
      double hubY = hubLocation.getY();

      double xDiff = botX - hubX;
      double yDiff = botY - hubY;

      double diff = (xDiff * xDiff) + (yDiff * yDiff);

      return Math.sqrt(diff);
  }

  public double getRotationFromHub(){
    if(swerve.m_isBlueAlliance){
      return getRotationToBlueHub();
    } else {
      return getRotationToRedHub();
    }
  }

  private double getRotationToRedHub(){
    return getRotationToLocation(hubPosRed);
  }

  private double getRotationToBlueHub(){
    return getRotationToLocation(hubPosBlue);
  }
  private double getRotationToLocation(Pose2d location){
    double botX = swerve.getState().Pose.getX();
    double botY = swerve.getState().Pose.getY();

    double hubX = location.getX();
    double hubY = location.getY();

    double xDiff = botX - hubX;
    double yDiff = botY - hubY;

    return Math.atan2(yDiff,xDiff);
  }
  @Override
  public void periodic() {
    this.addVisionMeasurement(leftcamera, leftCamTransform);
    this.addVisionMeasurement(rightCamera, rightCamTransform);
  }
}
