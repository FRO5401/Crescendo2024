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
  }

  /**
   * @param left left motor speed
   * @param right right motor speed
   */
  public void drive(double left, double right){
    allDrive.tankDrive(left, right);
  }

  /**
   *
   * @return position of the left drive motor via the encoder
   */
  public double getLeftPosition(){
    return leftEncoder1.getPosition();
  }

  /**
   *
   * @return position of the right drive motor via the encoder
   */
  public double getRightPostion(){
    return rightEncoder1.getPosition();
  }

  /**
   *
   * @return velocity of the left drive motor via the encoder
   */
  public double getLeftVelocity(){
    return leftEncoder1.getVelocity();
  }

  /**
   *
   * @return velocity of the right drive motor via the encoder
   */
  public double getRightVelocity(){
    return rightEncoder1.getVelocity();
  }

  /**
   *
   * @param velocity sets the speed of the left drive motor
   */
  public void setLeftVelocity(double velocity){
    leftPIDController.setReference(velocity, ControlType.kVelocity);
  }

  /**
   *
   * @param velocity sets the position of the left drive motor
   */
  public void setLeftPosition(double setPoint){
    leftPIDController.setReference(setPoint, ControlType.kPosition);
  }

    /**
   *
   * @param velocity sets the speed of the right drive motor
   */
  public void setRightVelocity(double velocity){
    rightPIDController.setReference(velocity, ControlType.kVelocity);
  }
  /**
   *
   * @param velocity sets the position of the right drive motor
   */
  public void setRightPostion(double setPoint){
    rightPIDController.setReference(setPoint, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.getNumber("Left Drive Encoder Value", getLeftPosition());
    SmartDashboard.getNumber("Right Drive Encoder Value", getRightPostion());
  }
}