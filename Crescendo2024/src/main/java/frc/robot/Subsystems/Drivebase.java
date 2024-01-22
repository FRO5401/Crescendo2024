// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivebase extends SubsystemBase {
  /* 
   Declaring Variables
  */

  // Left-Side Drive Motors
  private CANSparkMax leftDrive1;
  private CANSparkMax leftDrive2;

  // Right-Side Drive Motors
  private CANSparkMax rightDrive1;
  private CANSparkMax rightDrive2;

  private DifferentialDrive allDrive;

  //encoders used for messuring motor rotation. 
  private RelativeEncoder leftEncoder1;
  private RelativeEncoder leftEncoder2;

  private RelativeEncoder rightEncoder1;
  private RelativeEncoder rightEncoder2;

  //PID used for setpoints and velocity
  private SparkPIDController leftPIDController;
  private SparkPIDController rightPIDController;

 
 
  /** Creates a new Drivebase. */
  public Drivebase() {
    // Initalizing class variables
    leftDrive1 = new CANSparkMax(Constants.DriveMotors.LEFT_DRIVE1_ID, MotorType.kBrushless);
    leftDrive2 = new CANSparkMax(Constants.DriveMotors.LEFT_DRIVE2_ID, MotorType.kBrushless);
    rightDrive1 = new CANSparkMax(Constants.DriveMotors.RIGHT_DRIVE1_ID, MotorType.kBrushless);
    rightDrive2 = new CANSparkMax(Constants.DriveMotors.RIGHT_DRIVE2_ID, MotorType.kBrushless);

    //grab the encoders off the motors
    leftEncoder1 = leftDrive1.getEncoder();
    leftEncoder2 = leftDrive2.getEncoder();
    rightEncoder1 = rightDrive1.getEncoder();
    rightEncoder2 = rightDrive2.getEncoder();

    // Sets the drive motor params to factory default on every boot
    leftDrive1.restoreFactoryDefaults();
    leftDrive2.restoreFactoryDefaults();
    rightDrive1.restoreFactoryDefaults();
    rightDrive2.restoreFactoryDefaults();

    // When the driver lefts go of the trigger, apply full breaking, do not allow coasting
    leftDrive1.setIdleMode(IdleMode.kBrake);
    leftDrive2.setIdleMode(IdleMode.kBrake);
    rightDrive1.setIdleMode(IdleMode.kBrake);
    rightDrive2.setIdleMode(IdleMode.kBrake);

    // Having drive motor 2 [Left & Right] follow the actions of drive motor 1
    leftDrive2.follow(leftDrive1);
    rightDrive2.follow(rightDrive1);

    // Has the left drive motors spin in the opposite direction so the robot does not turn in a circle
    leftDrive1.setInverted(true);
    
    leftEncoder1.setInverted(true);
    leftEncoder2.setInverted(true);

    
    // Initalized allDrive
    allDrive = new DifferentialDrive(leftDrive1, rightDrive1);
    // Sets the max output to motors as 100%
    allDrive.setMaxOutput(1);
    // To handle errors from WPI 2024
    allDrive.setExpiration(0.1);
  }

  /**
   * @param left left motor speed
   * @param right right motor speed
   */
  public void drive(double left, double right){
    allDrive.tankDrive(left, right);
  }

  public double getLeftPosition(){
    return leftEncoder1.getPosition();
  }

  public double getRightPostion(){
    return rightEncoder1.getPosition();
  }

  public double getLeftVelocity(){
    return leftEncoder1.getVelocity();
  }

  public double getRightVelocity(){
    return rightEncoder1.getVelocity();
  }

  public void setVelocity(double velocity){
    leftPIDController.setReference(velocity, ControlType.kVelocity);
  }

  public void setPosition(double setPoint){
    leftPIDController.setReference(setPoint, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
