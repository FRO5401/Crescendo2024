// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
  PhotonCamera camera;
  /** Creates a new Camera. */
  public Camera() {
    camera = new PhotonCamera("Test");
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

  double getYaw(){

    var result = camera.getLatestResult();
    if(hasTarget()){
      return result.getBestTarget().getYaw();
    }
    return 0;
  }

  double getArea(){
    var result = camera.getLatestResult();
    if(hasTarget()){
      return result.getBestTarget().getArea();
    }
    return 0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Test", hasTarget());
    SmartDashboard.putNumber("Target area", getArea());
    SmartDashboard.putNumber(" ID", getAprilTagID());

  }
}
