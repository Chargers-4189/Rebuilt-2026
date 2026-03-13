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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.NetworkTables.SwerveTable;

public class Vision extends SubsystemBase {

  private static AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static final Pose2d hubPoseRed = findMidpoint(layout.getTagPose(4).get().toPose2d(), layout.getTagPose(10).get().toPose2d());
  public static final Pose2d hubPoseBlue = findMidpoint(layout.getTagPose(20).get().toPose2d(), layout.getTagPose(26).get().toPose2d());
  public static final Pose2d fieldCenterPose = new Pose2d(layout.getFieldLength() / 2, layout.getFieldWidth() / 2, new Rotation2d());
  public static final Pose2d redOriginPose = new Pose2d(layout.getFieldLength(), layout.getFieldWidth(), Rotation2d.k180deg);

  public static final double redTrenchX = fieldCenterPose.getX() - 143.50;
  public static final double blueTrenchX = fieldCenterPose.getX() - 143.50;

  PhotonCamera leftcamera = new PhotonCamera("LeftCam");
  Transform3d leftCamTransform = new Transform3d(Units.inchesToMeters(12.25),Units.inchesToMeters(2),Units.inchesToMeters(10.75), new Rotation3d(0,Units.degreesToRadians(-30),0));

  PhotonCamera rightCamera = new PhotonCamera("RightCam");
  Transform3d rightCamTransform = new Transform3d(Units.inchesToMeters(12.25),Units.inchesToMeters(-7.5),Units.inchesToMeters(10.75), new Rotation3d(0,Units.degreesToRadians(-50),0));
  
  SwerveSubsystem swerve;

  public Vision(SwerveSubsystem swerve) {
    this.swerve = swerve;
  }

  public void addVisionMeasurement(PhotonCamera camera, Transform3d robotTransform){
    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(layout, robotTransform); 
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    if(!results.isEmpty()){
      for (PhotonPipelineResult result : results) {
        Optional<EstimatedRobotPose> estimatedPoseOptional = poseEstimator.estimateAverageBestTargetsPose(result);
        
        if(estimatedPoseOptional.isPresent()){
          EstimatedRobotPose estimatedPose = estimatedPoseOptional.get();
          swerve.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
        }
      }
    }
  }

  public double getDistanceFromHub(){
    return getDistanceFromLocation(getHubPose());
  }

  private Pose2d getHubPose() {
    if(swerve.m_isBlueAlliance){
      return hubPoseBlue;
    } else {
      return hubPoseRed;
    }
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
    return getRotationToLocation(hubPoseRed);
  }

  private double getRotationToBlueHub(){
    return getRotationToLocation(hubPoseBlue);
  }
  private double getRotationToLocation(Pose2d location){
    double botX = swerve.getState().Pose.getX();
    double botY = swerve.getState().Pose.getY();

    double hubX = location.getX();
    double hubY = location.getY();

    double xDiff = hubX - botX;
    double yDiff = hubY - botY;

    return (new Rotation2d(xDiff, yDiff)).getRotations();
  }

  private static Pose2d findMidpoint(Pose2d pose1, Pose2d pose2) {
    double x1 = pose1.getX();
    double y1 = pose1.getY();
    
    double x2 = pose2.getX();
    double y2 = pose2.getY();

    double xAvg = (x1 + x2) / 2;
    double yAvg = (y1 + y2) / 2;

    return new Pose2d(xAvg, yAvg, new Rotation2d());
  }

  @Override
  public void periodic() {
    this.addVisionMeasurement(leftcamera, leftCamTransform);
    this.addVisionMeasurement(rightCamera, rightCamTransform);

    //System.out.println(getDistanceFromHub());
    SwerveTable.hubRotation.set(getRotationFromHub());
    SwerveTable.hubDistance.set(getDistanceFromHub());
  }

  
  public static Pose2d convertFieldPos(Pose2d bluePerspective) {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)) {
      return bluePerspective.rotateAround(fieldCenterPose.getTranslation(), Rotation2d.k180deg);
    } else {
      return bluePerspective;
    }
  }

  public static Rotation2d convertFieldRotation(Rotation2d bluePerspective) {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)) {
      return bluePerspective.plus(Rotation2d.k180deg);
    } else {
      return bluePerspective;
    }
  }

  public static Pose2d flipFieldPose(Pose2d pose, boolean rightSide) {
    if (rightSide) {
      return flipPose(pose);
    } else {
      return pose;
    }
  }

  public static Pose2d flipPose(Pose2d pose) {
    return new Pose2d(pose.getX(), redOriginPose.getY() - pose.getY(), pose.getRotation().unaryMinus());
  }

  public static Rotation2d convertFieldRotations(Rotation2d bluePerspective) {
    if (isRedAlliance()) {
      return bluePerspective.rotateBy(Rotation2d.k180deg);
    } else {
      return bluePerspective;
    }
  }
  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red);
  }
  
  /*
  public static enum FieldZone {
    OUR_ZONE, OPPONENT_ZONE, NEUTRAL_ZONE, BLUE_ZONE,
  }

  public FieldZone getRobotFieldZone() {
    Pose2d pose = swerve.getPose();

    if (pose.getX() < blueTrenchX) {
      // Blue Zone
      if (isRedAlliance()) {
        return FieldZone.OPPONENT_ZONE;
      } else {
        return FieldZone.OUR_ZONE;
      }
    } else if (pose.getX() < blueTrenchX) {
      // Neutral Zone
      return FieldZone.NEUTRAL_ZONE;
    } else {
      // Red Zone
      if (isRedAlliance()) {
        return FieldZone.OUR_ZONE;
      } else {
        return FieldZone.OPPONENT_ZONE;
      }
    }
  }
  */
}