// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

//WPI Imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Rev Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

//File Imports
import frc.robot.Constants;

public class Infeed extends SubsystemBase {
  //Creates CANSparkMax's
  private CANSparkMax pivotMotor;
  private CANSparkMax intakeMotor1;

  //Creates Encoders
  private RelativeEncoder pivotEncoder;

  //Creates PIDController
  private SparkPIDController pivotPID;

  /** Creates a new Infeed. */
  public Infeed() {
    //Initalizes CANSparkMax Motors
    pivotMotor = new CANSparkMax(Constants.InfeedConstants.PIVOT_ID, MotorType.kBrushless);
    intakeMotor1 = new CANSparkMax(Constants.InfeedConstants.INTAKE1_ID, MotorType.kBrushless);

    //Defines PID controller
    pivotPID = pivotMotor.getPIDController();

    //Gets Encoders for Pivot Motor
    pivotEncoder = pivotMotor.getEncoder();

    //Start position of Pivot Motor
    pivotEncoder.setPosition(0);

    //Resets Motors to factory defaults
    pivotMotor.restoreFactoryDefaults();
    intakeMotor1.restoreFactoryDefaults();

    pivotMotor.setSmartCurrentLimit(50);

    //Sets idle mode for when motors aren't being directly used
    pivotMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor1.setIdleMode(IdleMode.kBrake);

    //Creates PID for Pivot
    pivotPID.setP(Constants.InfeedConstants.pivotP);
    pivotPID.setI(Constants.InfeedConstants.pivotI);
    pivotPID.setD(Constants.InfeedConstants.pivotD);
    pivotPID.setIZone(Constants.InfeedConstants.pivotILimit);
    
  }
  //Gets position of pivotMotor
  public double getPosition(){
    return pivotEncoder.getPosition();
  }
  //Gets Velocity of pivotMotor
  public double getVelocity(){
    return pivotEncoder.getVelocity();
  }  
  //Sets point of pivot motor
  public void setPoint(double position){
    pivotPID.setReference(position, ControlType.kPosition);
  }
  //Makes Infeed take in a Note
  public void intake(){
    intakeMotor1.set(Constants.InfeedConstants.INTAKE_SPEED);
  }
  //Makes Infeed expel Note
  public void expel(){
    intakeMotor1.set(Constants.InfeedConstants.EXPEL_SPEED);
  }
  //Stop Infeed Motor
  public void stopIntake(){
    intakeMotor1.set(0);
  }
  //Rotates the pivot Motor to ground
  public void rotatetoGround(){
    //If pivot motor position is greater than out_position pivot motor stops
    if(pivotEncoder.getPosition() > Constants.InfeedConstants.OUT_POSITION){
        pivotMotor.set(0);
    } else {
      //Other wise pivot motor moves at -35% speed
      pivotMotor.set(Constants.InfeedConstants.PIVOT_TO_GROUND_SPEED);
    }

  }
  //Rotates the pivot motor to Shooter
  public void rotatetoShooter(){
    //If pivot motor position is greater than in_position pivot motor moves back at -20% speed
    if (pivotEncoder.getPosition() > Constants.InfeedConstants.IN_POSITION){
      pivotMotor.set(Constants.InfeedConstants.OVERSHOOT_FIX_SPEED);
    }
    else{
      //Other wise pivot motor moves at 35% speed
      pivotMotor.set(Constants.InfeedConstants.PIVOT_TO_SHOOTER_SPEED);
    }
  }
  //Stops pivot motor
  public void stopPivot(){
    pivotMotor.set(0);
  }
  //Resets enocoder position to 0
  public void resetEncoder(){
    pivotEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Current Pivot", pivotMotor.getOutputCurrent());
    //Creates SmartDashBoard pivotMotor Encoder Value
    SmartDashboard.putNumber("pivotMotor Encoder Value", getPosition());

  }
}
