// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivebase;
import frc.robot.Subsystems.Photonvision;

public class AutoAlign extends Command {
  /** Creates a new AutoAlign. */
  Photonvision camera;
  Drivebase drivebase;

  double distance; 
  double angle;

  public AutoAlign(Drivebase m_drivebase, Photonvision m_camera) {
    camera = m_camera;
    drivebase = m_drivebase;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(camera,drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      if(camera.hasTarget()){
        distance = camera.getDistance();
        angle = camera.getYaw();
        drivebase.arcadeDrive(drivebase.calculateFoward(.515, distance), drivebase.calculateTurn(18.05, angle));
      }
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
