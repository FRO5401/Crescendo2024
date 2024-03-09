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

    // Gets the distance traveled
    rightEncoder.getDistance();
    leftEncoder.getDistance();

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
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}