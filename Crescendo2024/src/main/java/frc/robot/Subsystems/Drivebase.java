// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

//REV imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
//WPI imports
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static  edu.wpi.first.units.MutableMeasure.mutable;
//File imports
import frc.robot.Constants;

public class Drivebase extends SubsystemBase {
  /* 
   Declaring Variables
  */

  //NAVX Gyro
    private  AHRS navxGyro;

  //Odomentry
  private  Odometry odometry;

  // Left-Side Drive Motors
  private  CANSparkMax leftDrive1;
  private CANSparkMax leftDrive2;

  // Right-Side Drive Motors
  private  CANSparkMax rightDrive1;
  private CANSparkMax rightDrive2;

  private  DifferentialDrive allDrive;

  //encoders used for messuring motor rotation. 
  private  RelativeEncoder leftEncoder1;
  private RelativeEncoder leftEncoder2;

  private  RelativeEncoder rightEncoder1;
  private RelativeEncoder rightEncoder2;

  //conversion factor

  //PID used for setpoints and velocity
  private  SparkPIDController leftPIDController;
  private  SparkPIDController rightPIDController;

  //WPILIB pid controllers for calculations
  private  PIDController turnWPIDController;
  private  PIDController forwardWPIDController;




  //solenoid 

  private  Solenoid solenoid;
  private Compressor compressor;

  private  boolean isHighGear = false; 

  //auto things

    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  private final SysIdRoutine m_sysIdRoutine =
  new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Measure<Voltage> volts) -> {
            leftDrive1.setVoltage(volts.in(Volts));
            rightDrive1.setVoltage(volts.in(Volts));
          },
          // Tell SysId how to record a frame of data for each motor on the mechanism being
          // characterized.
          log -> {
            // Record a frame for the left motors.  Since these share an encoder, we consider
            // the entire group to be one motor.
            log.motor("drive-left")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        leftDrive1.get() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(leftEncoder1.getPosition(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(leftEncoder1.getVelocity(), MetersPerSecond));
            // Record a frame for the right motors.  Since these share an encoder, we consider
            // the entire group to be one motor.
            log.motor("drive-right")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        rightDrive1.get() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(rightEncoder1.getPosition(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(rightEncoder1.getVelocity(), MetersPerSecond));
          },
          // Tell SysId to make generated commands require this subsystem, suffix test state in
          // WPILog with this subsystem's name ("drive")
          this));

 
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

    ///configure encoder 
    leftEncoder1.setPositionConversionFactor(Constants.AutoConstants.CONVERSION_FACTOR);
    leftEncoder2.setPositionConversionFactor(Constants.AutoConstants.CONVERSION_FACTOR);
    rightEncoder1.setPositionConversionFactor(Constants.AutoConstants.CONVERSION_FACTOR);
    rightEncoder2.setPositionConversionFactor(Constants.AutoConstants.CONVERSION_FACTOR);

    leftEncoder1.setVelocityConversionFactor(Constants.AutoConstants.CONVERSION_FACTOR / 60);
    leftEncoder2.setVelocityConversionFactor(Constants.AutoConstants.CONVERSION_FACTOR/ 60);
    rightEncoder1.setVelocityConversionFactor(Constants.AutoConstants.CONVERSION_FACTOR/ 60);
    rightEncoder2.setVelocityConversionFactor(Constants.AutoConstants.CONVERSION_FACTOR/60);



    //Set Encoder Value
    leftEncoder1.setPosition(0);
    leftEncoder2.setPosition(0);
    rightEncoder1.setPosition(0);
    rightEncoder2.setPosition(0);

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

    leftDrive1.setSmartCurrentLimit(45);
    leftDrive2.setSmartCurrentLimit(45);
    rightDrive1.setSmartCurrentLimit(45);
    rightDrive2.setSmartCurrentLimit(45);

    //Current limit 

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

    turnWPIDController = new PIDController(Constants.DriveMotors.ANGULAR_KP, Constants.DriveMotors.ANGULAR_KI, Constants.DriveMotors.ANGULAR_KD);
    forwardWPIDController = new PIDController(Constants.DriveMotors.KP, Constants.DriveMotors.KI, Constants.DriveMotors.KD);

    //Left PID declaration
    leftPIDController.setP(Constants.DriveMotors.KP);
    leftPIDController.setI(Constants.DriveMotors.KI);
    leftPIDController.setD(Constants.DriveMotors.KD);
    
    compressor = new Compressor(PneumaticsModuleType.REVPH);
    compressor.enableDigital();
    solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
    solenoid.set(isHighGear);

    navxGyro = new AHRS(SPI.Port.kMXP);
    navxGyro.reset();

     odometry = new DifferentialDriveOdometry(navxGyro.getRotation2d(), getLeftPosition(),
        getRightPostion());


  }
  /**
   * @param left left motor speed
   * @param right right motor speed
   */
  public   void drive(double left, double right){
    allDrive.tankDrive(left, right);
  }

  public  void arcadeDrive(double forwardSpeed, double rotateSpeed){
    allDrive.arcadeDrive(forwardSpeed, rotateSpeed);
  }
  //Gets position of left motor 1 
  public   double getLeftPosition(){
    return leftEncoder1.getPosition() * Constants.AutoConstants.CONVERSION_FACTOR ;
  }
  //Gets position of right motor 1 
  public  double getRightPostion(){
    return rightEncoder1.getPosition() * Constants.AutoConstants.CONVERSION_FACTOR;
  }
  //Gets velocity of left motor 1
  public  double getLeftVelocity(){
    return leftEncoder1.getVelocity() * (Constants.AutoConstants.CONVERSION_FACTOR / 60);
  }
  //Gets velocity of right motor 1
  public  double getRightVelocity(){
    return rightEncoder1.getVelocity() * (Constants.AutoConstants.CONVERSION_FACTOR / 60);
  }
  //Sets velocity of left motor 1
  public  void setLeftVelocity(double velocity){
    leftPIDController.setReference(velocity, ControlType.kVelocity);
  }
  //Sets position of left motor 1
  public void setLeftPosition(double setPoint){
    leftPIDController.setReference(setPoint, ControlType.kPosition);
  }
  //Sets velocity of right motor 1
  public  void setRightVelocity(double velocity){
    rightPIDController.setReference(velocity, ControlType.kVelocity);
  }
  //Sets position of right motor 1
  public  void setRightPostion(double setPoint){
    rightPIDController.setReference(setPoint, ControlType.kPosition);
  }

  public  double calculateTurn(double desired, double current){
    return turnWPIDController.calculate(current, desired);
  }
    public  double calculateFoward(double desired, double current){
    return forwardWPIDController.calculate(current, desired);
  }
  public  void toggleGearShifter(){
    isHighGear = !isHighGear;
    solenoid.set(isHighGear);
  }

    /**

   * Returns the currently-estimated pose of the robot.

   *

   * @return The pose.

   */

  public Pose2d getPose() {

    return odometry.getPoseMeters();

  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {

    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());

  }

  public  DifferentialDriveWheelPositions getWheelPositions() {

    return new DifferentialDriveWheelPositions(getLeftPosition(), getRightPostion());

  }


  public void resetOdometry(Pose2d pose) {

    odometry.resetPosition(

        navxGyro.getRotation2d(), getWheelPositions(), pose);

  }

  public  void tankDriveVolts(double leftVolts, double rightVolts) {

    leftDrive1.setVoltage(leftVolts);

    rightDrive1.setVoltage(rightVolts);

    allDrive.feed();

  }

  public  void zeroHeading() {

    navxGyro.reset();

  }

    /**

   * Returns the heading of the robot.

   *

   * @return the robot's heading in degrees, from -180 to 180

   */

   public  double getHeading() {

    return navxGyro.getRotation2d().getDegrees();

  }


  /**

   * Returns the turn rate of the robot.
   * @return The turn rate of the robot, in degrees per second
   */

  public  double getTurnRate() {

    return -navxGyro.getRate();

  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  return m_sysIdRoutine.quasistatic(direction);
}

public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  return m_sysIdRoutine.dynamic(direction);
}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    odometry.update(navxGyro.getRotation2d(), getWheelPositions());
    SmartDashboard.putNumber("Conversion factor", leftEncoder1.getPositionConversionFactor());
        SmartDashboard.putNumber("Conversion factor2", Constants.AutoConstants.CONVERSION_FACTOR);


    //Displays left drives encoder value to smart dashboard
    SmartDashboard.putNumber("Left Drive Encoder Value", getLeftPosition());
    //Displays right drives encoder value to smart dashboard
    SmartDashboard.putNumber("Right Drive Encoder Value", getRightPostion());
    //displays gear shift state
    SmartDashboard.putBoolean("isHighGear", isHighGear);
    SmartDashboard.putNumber("PSI", compressor.getPressure());
    SmartDashboard.putNumber("Position X", getPose().getX());
    SmartDashboard.putNumber("Position Y", getPose().getY());
    SmartDashboard.putNumber("Angle", getPose().getRotation().getDegrees());

  }
}