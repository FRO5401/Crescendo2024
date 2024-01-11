// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

//Imports
import static frc.robot.Utilities.Tabs.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Solenoid;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import java.util.Collection;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

//TODO: Remember to bring gear shifter back
public class DriveBase extends SubsystemBase {
  private AHRS navxGyro;
  PhotonCamera camera;
  PhotonCamera camera2;

  // Configuring Motors
  private CANSparkMax leftDrive1;
  private CANSparkMax leftDrive2;

  private CANSparkMax rightDrive1;
  private CANSparkMax rightDrive2;

  private SparkMaxPIDController leftDrive1PidController;
  private SparkMaxPIDController leftDrive2PidController;

  private SparkMaxPIDController rightDrive1PidController;
  private SparkMaxPIDController rightDrive2PidController;

  DigitalInput minSwitch, maxSwitch;

  // Configuring Drives
  private MotorControllerGroup leftDrives;
  private MotorControllerGroup rightDrives;
  private DifferentialDrive allDrive;
  private DifferentialDriveOdometry odometry;
  private int motorCurrent = 24;

  private boolean compressorState = false;

  // PID stuff
  // private int loopIndex, slotIndex;
  private double kP = 1.92;
  private double kI = 0;
  private double kD = 1;

  // varibles for speed adjustment
  private double kEncoder = 0.01;
  double percentDifference;

  // Solenoid
  private Solenoid gearShifter;

  // Sensors
  private RelativeEncoder leftEncoders[];
  private RelativeEncoder rightEncoders[];
  public Compressor compressor;
  private PowerDistribution pdp;
  Collection<CANSparkMax> drivebaseMotors;
  SparkMaxPIDController pidRotateMotorLeft;
  PhotonTrackedTarget target;
  /**
   *
   */
  SparkMaxPIDController pidRotateMotorRight;
  RelativeEncoder rotate_encoder_left, rotate_encoder_right;

  public DriveBase() {

    // Instantating the physical parts on the drivebase
    compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

    compressor.enableDigital();
    camera = new PhotonCamera("camera");
    camera2 = new PhotonCamera("camera2");

    pdp = new PowerDistribution();
    navxGyro = new AHRS(SPI.Port.kMXP);

    leftDrive1 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_LEFT_1, MotorType.kBrushless);
    leftDrive2 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_LEFT_2, MotorType.kBrushless);
    rightDrive1 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_1, MotorType.kBrushless);
    rightDrive2 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_2, MotorType.kBrushless);

    leftEncoders = new RelativeEncoder[3];
    rightEncoders = new RelativeEncoder[3];

    leftEncoders[0] = leftDrive1.getEncoder();
    leftEncoders[1] = leftDrive2.getEncoder();

    rightEncoders[0] = rightDrive1.getEncoder();
    rightEncoders[1] = rightDrive2.getEncoder();

    rightDrive1.setSmartCurrentLimit(motorCurrent, motorCurrent); // 15, 10
    rightDrive2.setSmartCurrentLimit(motorCurrent, motorCurrent);
    leftDrive1.setSmartCurrentLimit(motorCurrent, motorCurrent);
    leftDrive2.setSmartCurrentLimit(motorCurrent, motorCurrent);

    gearShifter = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    drivebaseShuffleboard();

    leftDrive1PidController = leftDrive1.getPIDController();
    leftDrive2PidController = leftDrive2.getPIDController();
    rightDrive1PidController = rightDrive1.getPIDController();
    rightDrive2PidController = rightDrive2.getPIDController();

    leftDrives = new MotorControllerGroup(leftDrive1, leftDrive2, leftDrive3);
    rightDrives = new MotorControllerGroup(rightDrive1, rightDrive2, rightDrive3);

    odometry = new DifferentialDriveOdometry(navxGyro.getRotation2d(), leftEncoders[0].getPosition(),
        -rightEncoders[0].getPosition());

    allDrive = new DifferentialDrive(leftDrives, rightDrives);

    allDrive.setExpiration(0.1);
    allDrive.setMaxOutput(1.0);

    leftDrive1PidController.setP(kP);
    leftDrive1PidController.setI(kI);
    leftDrive1PidController.setD(kD);
    leftDrive2PidController.setP(kP);
    leftDrive2PidController.setI(kI);
    leftDrive2PidController.setD(kD);

    rightDrive1PidController.setP(kP);
    rightDrive1PidController.setI(kI);
    rightDrive1PidController.setD(kD);
    rightDrive2PidController.setP(kP);
    rightDrive2PidController.setI(kI);
    rightDrive2PidController.setD(kD);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftDrives.setVoltage(leftVolts);
    rightDrives.setVoltage(-rightVolts);
    allDrive.feed();
  }

  public void stopMotors() {
    leftDrives.set(0);
    rightDrives.set(0);
  }

  public void pidDrive(double velocityLeft, double velocityRight) {
    setPIDVelocity(leftDrive1PidController, leftEncoders[0], ControlType.kSmartVelocity, velocityLeft);
    setPIDVelocity(leftDrive2PidController, leftEncoders[1], ControlType.kSmartVelocity, velocityLeft);

    setPIDVelocity(rightDrive1PidController, rightEncoders[0], ControlType.kSmartVelocity, velocityRight);
    setPIDVelocity(rightDrive2PidController, rightEncoders[1], ControlType.kSmartVelocity, velocityRight);
  }

  public void switchVisionMode(int i) {
    camera.setPipelineIndex(i);
  }

  public void toggleDriverMode() { // Sets camera on or off
    if (camera.getDriverMode()) {
      camera.setDriverMode(false);
    } else {
      camera.setDriverMode(true);
    }
  }

  public PhotonCamera getCamera() {
    return camera;
  }

  public PhotonCamera getCamera2() {
    return camera2;
  }

  public float getPitch() {
    return navxGyro.getPitch();
  }

  public float getRoll() {

    return navxGyro.getRoll();

  }

  public void setPIDVelocity(CANPIDController pidTransMotor2, RelativeEncoder m_encoder, ControlType kposition,
      double setPoint) {

    pidTransMotor2.setReference(setPoint, ControlType.kVelocity);
  }

  public void drive(double left, double right) {
    allDrive.tankDrive(left, right);
  }

  public double getGyro() {
    return navxGyro.getAngle();
  }

  public PowerDistribution getPDP() {
    return pdp;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(navxGyro.getRotation2d(), 0, 0, pose);
  }

  public void gearShift(String mode) {
    if (mode.equalsIgnoreCase("HIGH")) {
      gearShifter.set(true);
    } else if (mode.equalsIgnoreCase("LOW")) {
      gearShifter.set(false);
    }

  }

  public boolean getGear() {
    return gearShifter.get();
  }

  public double getPressure() {
    return compressor.getPressure();
  }

  public boolean getPressureStatus() {
    if (compressor.getPressure() <= 110.0) {
      return true;
    } else {
      return false;
    }
  }

  public void compressorToggle() {
    compressorState = !compressorState;
    setCompressor(compressorState);
  }

  // Set the Compressor
  public void setCompressor(boolean state) {
    if (state == false)
      compressor.disable();
    else
      compressor.enableDigital();
  }

  public double getAverageMotorVelocity() {
    return (Math.abs(leftEncoders[0].getVelocity()) + Math.abs(rightEncoders[0].getVelocity())) / 2;
  }

  public double getLeftVelocity() {
    return leftEncoders[0].getVelocity();
  }

  public double getRightVelocity() {
    return rightEncoders[0].getVelocity();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {

    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), -getRightVelocity());

  }

  public void drivebaseShuffleboard() {
    // Graph conf

    // Testing Tab

    speedEntry = testingTab.add("Robot Speed", getAverageMotorVelocity()).getEntry();
    leftSpeedEntry = testingTab.add("Left Motor Speed", getLeftVelocity()).getEntry();
    rightSpeedEntry = testingTab.add("Right Motor Speed", getRightVelocity()).getEntry();
    leftPositionEntry = testingTab.add("Left Motor Position", leftEncoders[0].getPosition()).getEntry();
    rightPositionEntry = testingTab.add("Right Motor Position", rightEncoders[0].getPosition()).getEntry();
    rotationsEntry = testingTab.add("Gyro Rotations", getGyro() / 360).getEntry();
    angleEntry = testingTab.add("Gyro Angle", getGyro()).getEntry();
    shifterEntry = testingTab.add("Solenoid Gear", getGear()).getEntry();
    pressureEntry = testingTab.add("Pressure ", getPressureStatus()).getEntry();

  }

  public double getPosition() {
    return leftEncoders[2].getPosition();
  }

  public void updateOdometry() {
    odometry.update(navxGyro.getRotation2d(), leftEncoders[1].getPosition(), rightEncoders[1].getPosition());
  }

  public void resetEncoders() {
    leftEncoders[0].setPosition(0);
    leftEncoders[1].setPosition(0);
    rightEncoders[0].setPosition(0);
    rightEncoders[1].setPosition(0);

  }

  public void autoTurn(double speed, double angle) {
    double gyroAngle = getGyro();
    if (gyroAngle > (angle))
      drive(-speed, speed);
    else if (gyroAngle < (angle))
      drive(speed, -speed);
    else
      drive(0, 0);
  }

  // method used to fix the drift if the speed is wrong
  public double getAdjustedSpeed(double num, double dom) {
    percentDifference = 1 - (kEncoder * (1 - (dom / num)));
    return percentDifference;
  }

  public void autoDrive(double left, double right, double angle) {

    if (left > 0 && right > 0) { // driving forwards
      drive(
          angle > 0.1 ? left : left * 1.01,
          angle < -0.1 ? right : right * 1.01);
    } else if (left < 0 && right < 0) { // driving backwards
      drive(
          angle < -0.1 ? left : left * 1.01,
          angle > 0.1 ? right : right * 1.01);
    }

  }

  public void resetGyroAngle() {
    navxGyro.reset();
  }

  public void smoothStop() {
    for (int i = 0; i < 1000; i++) {
      leftDrives.set(-0.3);
      rightDrives.set(-0.3);

    }
    leftDrives.set(0);
    rightDrives.set(0);
  }

  public double getRightEncoder() {
    return rightEncoders[0].getPosition();
  }

  public double getLeftEncoder() {
    return leftEncoders[0].getPosition();
  }
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
