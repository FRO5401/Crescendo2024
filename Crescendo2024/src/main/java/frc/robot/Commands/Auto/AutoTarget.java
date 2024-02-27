// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

//WPI Imports
import edu.wpi.first.wpilibj2.command.Command;

//Subsystem Imports
import frc.robot.Subsystems.Drivebase;
import frc.robot.Subsystems.Photonvision;

public class AutoTarget extends Command {
  //Declaring Variables
  private Photonvision camera;
  private Drivebase drivebase;
  private double targetDistance;
  private double targetAngle;

  /** Creates a new AutoTarget. */
  public AutoTarget(Photonvision m_camera, Drivebase m_drivebase, double m_targetDistance, double m_targetAngle) {
    camera = m_camera;
    drivebase = m_drivebase;
    targetDistance = m_targetDistance;
    targetAngle = m_targetAngle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase, camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(camera.hasTarget()){
      double range = camera.getDistance();
      double angle = camera.getYaw();
      drivebase.arcadeDrive(drivebase.calculateFoward(targetDistance, range), drivebase.calculateTurn(targetAngle, angle));
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
