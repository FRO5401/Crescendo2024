// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

//REV imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

//WPI imports
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;

//File imports
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

  // Quad Encoders
  private Encoder rightEncoder;
  private Encoder leftEncoder;

  // Stopped Encoders
  boolean rightStopped;
  boolean leftStopped;

  /** Creates a new Drivebase. */
  public Drivebase() {
    // Initalizing class variables
    leftDrive1 = new CANSparkMax(Constants.DriveMotors.LEFT_DRIVE1_ID, MotorType.kBrushed);
    leftDrive2 = new CANSparkMax(Constants.DriveMotors.LEFT_DRIVE2_ID, MotorType.kBrushed);
    rightDrive1 = new CANSparkMax(Constants.DriveMotors.RIGHT_DRIVE1_ID, MotorType.kBrushed);
    rightDrive2 = new CANSparkMax(Constants.DriveMotors.RIGHT_DRIVE2_ID, MotorType.kBrushed);    

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

    //Inverts left Motors so robot doesn't spin in circles
    leftDrive1.setInverted(true);
    leftDrive2.setInverted(true);

    // Initalized allDrive
    allDrive = new DifferentialDrive(leftDrive1, rightDrive1);

    // Sets the max output to motors as 100%
    allDrive.setMaxOutput(1);
    
    // To handle errors from WPI 2024
    allDrive.setExpiration(0.1);

    // Intaizing encoders
    rightEncoder = new Encoder(0, 8, false, Encoder.EncodingType.k2X);
    leftEncoder = new Encoder(1, 9, true, Encoder.EncodingType.k2X);

    // Resets the encoder to read a distance of zero
    rightEncoder.reset();
    leftEncoder.reset();

    // Configures the encoder to consider itself stopped when its rate is below 5
    rightEncoder.setMinRate(5);
    leftEncoder.setMinRate(5);

    // Gets when the encoder is stopped
    rightEncoder.getStopped();
    leftEncoder.getStopped();
  }

  /**
   * @param left left motor speed
   * @param right right motor speed
   */
  public void drive(double left, double right){
    allDrive.tankDrive(left, right);
  }
  // Right Encoder Distance
  public double getRightDistance(){
    return rightEncoder.getDistance();
  }
  // Left Encoder Distance
  public double getLeftDistance(){
    return leftEncoder.getDistance();
  }
  public void stoppedRightDrive(){
    if (rightEncoder.getStopped() == true){
      rightStopped = true;
    } else {
      rightStopped = false;
    }
  }
  public void stoppedLeftDrive(){
    if (leftEncoder.getStopped() == true){
      leftStopped = true;
    } else {
      leftStopped = false;
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     SmartDashboard.putNumber("Right Drive", getRightDistance());
     SmartDashboard.putNumber("Left Drive", getLeftDistance());

     SmartDashboard.getBoolean("Right Stopped", rightStopped);
     SmartDashboard.getBoolean("Left Stopped", leftStopped);
  }
}