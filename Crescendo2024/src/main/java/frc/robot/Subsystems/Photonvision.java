// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photonvision extends SubsystemBase{
  /** Creates a new NetworkTables. */
  PhotonCamera camera;


  public Photonvision(String cameraName) {
    camera = new PhotonCamera(cameraName);

    camera.setPipelineIndex(0);
  }

  public PhotonTrackedTarget getBestTarget(){
    var result = camera.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    return target;
  }

  public double getTargetYaw(){
    return getBestTarget().getYaw();
  }

  public double getTargetPitch(){
    return getBestTarget().getPitch();
  }

  public double getAprilTagID(){
    return getBestTarget().getFiducialId();
  }

  public boolean hasTargets(){
    var result = camera.getLatestResult();
    return result.hasTargets();
  }

  public PhotonCamera getCamera(){
    return camera;
  }

  public PhotonPipelineResult getLatestResult(){
    return camera.getLatestResult();
  }

  /**
  @return returns value in meters
  */
  public double getTargetHeight(int id){
    switch(id){
      case 1: return 122/100;
      case 2: return 122/100;
      case 3: return 132/100;
      case 4: return 132/100;
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

  public void shuffleBoardTabs(){
    var result = camera.getLatestResult();
    SmartDashboard.putNumber("Camera Latency", result.getLatencyMillis());
  }
  double test = getTargetHeight(1);
}
