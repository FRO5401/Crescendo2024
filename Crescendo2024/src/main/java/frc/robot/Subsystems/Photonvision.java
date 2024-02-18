// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// 24 1/2


package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Photonvision extends SubsystemBase{
  /** Creates a new NetworkTables. */
  PhotonCamera camera;


  public Photonvision(String cameraName) {
    camera = new PhotonCamera(cameraName);

    camera.setPipelineIndex(0);
  }

  public PhotonCamera getCamera(){
    return camera;
  }

 
  boolean hasTarget(){
    var result = camera.getLatestResult();
    return result.hasTargets();
   }
 
   double getAprilTagID(){
     var result = camera.getLatestResult();
     if(hasTarget()){
       return result.getBestTarget().getFiducialId();
     }
     return 0;
   }
 
   double getTargetYaw(){
 
     var result = camera.getLatestResult();
     if(hasTarget()){
       return result.getBestTarget().getYaw();
     }
     return 0;
   }
 
   double getTargetArea(){
     var result = camera.getLatestResult();
     if(hasTarget()){
       return result.getBestTarget().getArea();
     }
     return 0;
   }

  /**
  @return returns value in meters
  */
  public double getTargetHeight(int id){
    switch(id){
      case 1: return 122/100;
      case 2: return 122/100;
      case 3: return 1.37;
      case 4: return 1.37;
      case 5: return 122/100;
      case 6: return 122/100;
      case 7: return 132/100;
      case 8: return 132/100;
      case 9: return 122/100;
      case 10: return 122/100;
      case 11: return 121/100;
      case 12: return 121/100;
      case 13: return 121/100;
      case 14: return 121/100;
      case 15: return 121/100;
      case 16: return 121/100;

      default: return 0;
    }
    
  }

  public double distanceToTarget(){
     var result = camera.getLatestResult();

    if (result.hasTargets()) {
      // First calculate range
      double range =
        PhotonUtils.calculateDistanceToTargetMeters(
          Units.inchesToMeters(Constants.PhotonConstants.CAMERA_HEIGHT),
          getTargetHeight(result.getBestTarget().getFiducialId()),
          Units.degreesToRadians(Constants.PhotonConstants.CAMERA_ANGLE),
          Units.degreesToRadians(result.getBestTarget().getPitch())
          );
         return range;
        }
      return 0;
  }

  public void shuffleBoardTabs(){
    var result = camera.getLatestResult();
    SmartDashboard.putNumber("Camera Latency", result.getLatencyMillis());
    SmartDashboard.putNumber("Range", distanceToTarget());
    SmartDashboard.putNumber("Yaw",result.getBestTarget().getYaw());
  }

  @Override
  public void periodic(){
    shuffleBoardTabs();
  }
}
