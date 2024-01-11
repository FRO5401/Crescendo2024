// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants;

public class Drivebase extends SubsystemBase {
  /** Creates a new drivebase. */
    CANSparkMax leftDrive1;
    CANSparkMax leftDrive2;
    CANSparkMax rightDrive1;
    CANSparkMax rightDrive2;
  
    public Drivebase() {  
    leftDrive1 = new CANSparkMax(Constants.DriveConstants.LEFT_DRIVE_MOTOR_1, MotorType.kBrushless);
    leftDrive2 = new CANSparkMax(Constants.DriveConstants.LEFT_DRIVE_MOTOR_2, MotorType.kBrushless);
    rightDrive1 = new CANSparkMax(Constants.DriveConstants.RIGHT_DRIVE_MOTOR_1, MotorType.kBrushless);
    rightDrive2 = new CANSparkMax(Constants.DriveConstants.RIGHT_DRIVE_MOTOR_2, MotorType.kBrushless);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
