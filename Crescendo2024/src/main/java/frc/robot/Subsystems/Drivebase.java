// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

//REV imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
//WPI imports
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  //encoders used for messuring motor rotation. 
  private RelativeEncoder leftEncoder1;
  private RelativeEncoder leftEncoder2;

  private RelativeEncoder rightEncoder1;
  private RelativeEncoder rightEncoder2;

  //PID used for setpoints and velocity
  private SparkPIDController leftPIDController;
  private SparkPIDController rightPIDController;

  //solenoid 

  private Solenoid solenoid;
  private Compressor compressor;

  private boolean isHighGear = false; 
 
 
  /** Creates a new Drivebase. */
  public Drivebase() {
    // Initalizing class variables
    leftDrive1 = new CANSparkMax(Constants.DriveMotors.LEFT_DRIVE1_ID, MotorType.kBrushless);
    leftDrive2 = new CANSparkMax(Constants.DriveMotors.LEFT_DRIVE2_ID, MotorType.kBrushless);
    rightDrive1 = new CANSparkMax(Constants.DriveMotors.RIGHT_DRIVE1_ID, MotorType.kBrushless);
    rightDrive2 = new CANSparkMax(Constants.DriveMotors.RIGHT_DRIVE2_ID, MotorType.kBrushless);

    //Initalizing PID controller
    leftPIDController = leftDrive1.getPIDController();
    rightPIDController = rightDrive1.getPIDController();

    //grab the encoders off the motors
    leftEncoder1 = leftDrive1.getEncoder();
    leftEncoder2 = leftDrive2.getEncoder();
    rightEncoder1 = rightDrive1.getEncoder();
    rightEncoder2 = rightDrive2.getEncoder();

    //Set Encoder Value
    leftEncoder1.equals(0);
    leftEncoder2.equals(0);
    rightEncoder1.equals(0);
    rightEncoder2.equals(0);

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
    
    //Right PID declaration 
    rightPIDController.setP(Constants.DriveMotors.KP);
    rightPIDController.setI(Constants.DriveMotors.KI);
    rightPIDController.setD(Constants.DriveMotors.KD);

    //Left PID declaration
    leftPIDController.setP(Constants.DriveMotors.KP);
    leftPIDController.setI(Constants.DriveMotors.KI);
    leftPIDController.setD(Constants.DriveMotors.KD);
    
    compressor = new Compressor(PneumaticsModuleType.REVPH);
    compressor.enableDigital();
    solenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);
    solenoid.set(isHighGear);
  }

  /**
   * @param left left motor speed
   * @param right right motor speed
   */
  public void drive(double left, double right){
    allDrive.tankDrive(left, right);
  }
  //Gets position of left motor 1 
  public double getLeftPosition(){
    return leftEncoder1.getPosition();
  }
  //Gets position of right motor 1 
  public double getRightPostion(){
    return rightEncoder1.getPosition();
  }
  //Gets velocity of left motor 1
  public double getLeftVelocity(){
    return leftEncoder1.getVelocity();
  }
  //Gets velocity of right motor 1
  public double getRightVelocity(){
    return rightEncoder1.getVelocity();
  }
  //Sets velocity of left motor 1
  public void setLeftVelocity(double velocity){
    leftPIDController.setReference(velocity, ControlType.kVelocity);
  }
  //Sets position of left motor 1
  public void setLeftPosition(double setPoint){
    leftPIDController.setReference(setPoint, ControlType.kPosition);
  }
  //Sets velocity of right motor 1
  public void setRightVelocity(double velocity){
    rightPIDController.setReference(velocity, ControlType.kVelocity);
  }
  //Sets position of right motor 1
  public void setRightPostion(double setPoint){
    rightPIDController.setReference(setPoint, ControlType.kPosition);
  }
  public void toggleGearShifter(){
    isHighGear = !isHighGear;
    solenoid.set(isHighGear);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Displays left drives encoder value to smart dashboard
    SmartDashboard.getNumber("Left Drive Encoder Value", getLeftPosition());
    //Displays right drives encoder value to smart dashboard
    SmartDashboard.getNumber("Right Drive Encoder Value", getRightPostion());
    //displays gear shift state
    SmartDashboard.putBoolean("isHighGear", isHighGear);
    SmartDashboard.putNumber("PSI", compressor.getPressure());
  }
}