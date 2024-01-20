// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  // defines left encoders
  RelativeEncoder leftEncoder1;
  RelativeEncoder leftEncoder2;
  // defines right encoders <---- (Encoders measure how many times the motor turns (i think))
  RelativeEncoder rightEncoder1;
  RelativeEncoder rightEncoder2;


  //define pid controllers
  SparkPIDController leftPIDController;
  SparkPIDController rightPIDController;


  //TODO add gear shifter
  
  public Drivebase() {
    //initialize left drive motor
    leftDrive1 = new CANSparkMax(DriveConstants.LEFT_DRIVE1_ID, MotorType.kBrushed);
    leftDrive2 = new CANSparkMax(DriveConstants.LEFT_DRIVE2_ID, MotorType.kBrushed);
    
    //initialize right drive motor
    rightDrive1 = new CANSparkMax(DriveConstants.RIGHT_DRIVE1_ID, MotorType.kBrushed);
    rightDrive2 = new CANSparkMax(DriveConstants.RIGHT_DRIVE2_ID, MotorType.kBrushed);

    //restores factory defauls of all motors
    leftDrive1.restoreFactoryDefaults();
    leftDrive2.restoreFactoryDefaults();
    rightDrive1.restoreFactoryDefaults();
    rightDrive2.restoreFactoryDefaults();

    // sets the idle mode for all motors
    leftDrive1.setIdleMode(IdleMode.kBrake);
    leftDrive2.setIdleMode(IdleMode.kBrake);
    rightDrive1.setIdleMode(IdleMode.kBrake);
    rightDrive2.setIdleMode(IdleMode.kBrake);

    // sets the second right and left motor to folow their respective firsts
    leftDrive2.follow(leftDrive1);
    rightDrive2.follow(rightDrive1);

    // sets the left drive to be inverted
    leftDrive1.setInverted(true);

    //  makes a new all drive
    allDrive = new DifferentialDrive(leftDrive2, rightDrive1);
    allDrive.setMaxOutput(1);
    allDrive.setExpiration(0.1);

    // grabs the enncoders off of the left motors
    leftEncoder1 = leftDrive1.getEncoder();
    leftEncoder2 = leftDrive2.getEncoder();
    // grabs the encoders off of the right motors
    rightEncoder1 = rightDrive1.getEncoder();
    rightEncoder2 = rightDrive2.getEncoder();

    // sets the left encoders to be inverted
    leftEncoder1.setInverted(true);
    leftEncoder2.setInverted(true);

    //init pid controllers
    leftPIDController = leftDrive1.getPIDController();
    rightPIDController = rightDrive1.getPIDController();

  }

  
  /**
   * 
   * @param left left move speed
   * @param right right move speed
   */
  public void drive(double left, double right){
    allDrive.tankDrive(left, right);
  }

  public void configurePID(){
    // sets the P, I, and D controls for the left motor
    leftPIDController.setP(Constants.DriveConstants.KP);
    leftPIDController.setI(Constants.DriveConstants.KI);
    leftPIDController.setD(Constants.DriveConstants.KD);
    // sets the P, I, and D controls for the right motor
    rightPIDController.setP(Constants.DriveConstants.KP);
    rightPIDController.setI(Constants.DriveConstants.KI);
    rightPIDController.setD(Constants.DriveConstants.KD);  
  }

  // sets the reference points of the motors
  public void setPoint(double setPoint){
    leftPIDController.setReference(setPoint, ControlType.kPosition);
    rightPIDController.setReference(setPoint, ControlType.kPosition);
  }

  // sets the velocity of the motors
  public void setVelocity(double velocity){
    leftPIDController.setReference(velocity, ControlType.kVelocity);
    rightPIDController.setReference(velocity, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
