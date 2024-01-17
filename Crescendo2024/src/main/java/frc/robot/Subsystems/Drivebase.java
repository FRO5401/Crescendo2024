// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivebase extends SubsystemBase {
  /** Creates a new Drivebase. */
 //define left drive motors
  CANSparkMax leftDrive1;
  CANSparkMax leftDrive2;
 
 //define right drive motor
  CANSparkMax rightDrive1;
  CANSparkMax rightDrive2;

  DifferentialDrive allDrive;

  //TODO add gear shifter
  
  public Drivebase() {
    //initialize left drive motor
    leftDrive1 = new CANSparkMax(DriveConstants.LEFT_DRIVE1_ID, MotorType.kBrushed);
    leftDrive2 = new CANSparkMax(DriveConstants.LEFT_DRIVE2_ID, MotorType.kBrushed);
    
    //initialize right drive motor
    rightDrive1 = new CANSparkMax(DriveConstants.RIGHT_DRIVE1_ID, MotorType.kBrushed);
    rightDrive2 = new CANSparkMax(DriveConstants.RIGHT_DRIVE2_ID, MotorType.kBrushed);

    leftDrive1.restoreFactoryDefaults();
    leftDrive2.restoreFactoryDefaults();
    rightDrive1.restoreFactoryDefaults();
    rightDrive2.restoreFactoryDefaults();

    leftDrive1.setIdleMode(IdleMode.kBrake);
    leftDrive2.setIdleMode(IdleMode.kBrake);
    rightDrive1.setIdleMode(IdleMode.kBrake);
    rightDrive2.setIdleMode(IdleMode.kBrake);

    leftDrive2.follow(leftDrive1);
    rightDrive2.follow(rightDrive1);

    leftDrive1.setInverted(true);

    allDrive = new DifferentialDrive(leftDrive2, rightDrive1);
    allDrive.setMaxOutput(1);
    allDrive.setExpiration(0.1);
    
  }

  
  /**
   * 
   * @param left left move speed
   * @param right right move speed
   */
  public void drive(double left, double right){
    allDrive.tankDrive(left, right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
