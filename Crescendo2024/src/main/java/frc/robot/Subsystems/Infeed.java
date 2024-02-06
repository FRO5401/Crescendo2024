// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

  /*Finished
  1. Create a new subsystem called "Infeed"
  2. Create 3 CANSparkMax objects, 2 for the intake wheels and 1 for the pivoting
  5. Create a method called intake() that causes the intake wheels to spin in a positive direction
  6. Create a method called expel() that causes the intake wheels to spin in a negative direction
  7. Create a method called rotate that causes the pivot motor to spin in either a positive 
  or negative direction depending on the controller input 
 */

  /* TODO
  3. Create 2 DigitalInput objects that track the state of the frame rail & shooter limit switches. 
  4. When either switch is true, do not let the motors continue in either a higher 
  position (Shooter switch true) or a lower position (Framerail switch true).
  */

package frc.robot.Subsystems;

//WPI Imports
//import edu.wpi.first.math.controller.PIDController;
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
    //TODO findout which type of motors are being used, brushed or brushless
    //Initalizes CANSparkMax Motors
    pivotMotor = new CANSparkMax(Constants.InfeedConstants.PIVOT_ID, MotorType.kBrushless);
    intakeMotor1 = new CANSparkMax(Constants.InfeedConstants.INTAKE1_ID, MotorType.kBrushless);

    //Defines PID controller
    pivotPID = pivotMotor.getPIDController();

    //Gets Encoders for Pivot Motor
    pivotEncoder = pivotMotor.getEncoder();
    pivotEncoder.setPosition(0);

    //Resets Motors to factory defaults
    pivotMotor.restoreFactoryDefaults();
    intakeMotor1.restoreFactoryDefaults();

    //Sets idle mode for when motors aren't being directly used
    pivotMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor1.setIdleMode(IdleMode.kBrake);

    //Creates PID for Pivot
    pivotPID.setP(Constants.InfeedConstants.pivotP);
    pivotPID.setI(Constants.InfeedConstants.pivotI);
    pivotPID.setD(Constants.InfeedConstants.pivotD);
    
  }
  //Gets position of pivotMotor
  public double getPosition(){
    return pivotEncoder.getPosition();
  }
  //Gets Velocity of pivotMotor
  public double getVelocity(){
    return pivotEncoder.getVelocity();
  }  
  //
  public void setPoint(double position){
    pivotPID.setReference(position, ControlType.kPosition);
  }
  //Makes Infeed take in a Note
  public void intake(){
    intakeMotor1.set(0.5);
  }
  //Makes Infeed expel Note
  public void expel(){
    intakeMotor1.set(-0.5);
  }
  //Stop Infeed Motor
  public void stopIntake(){
    intakeMotor1.set(0);
  }
  //Rotates the pivot Motor to ground
  public void rotatetoGround(){
    if(pivotEncoder.getPosition() > Constants.InfeedConstants.OUT_POSITION){
        pivotMotor.set(0);
    } else {
      pivotMotor.set(-0.35);
    }

  }
  //Rotates the pivot motor to Shooter
  public void rotatetoShooter(){
    if (pivotEncoder.getPosition() > Constants.InfeedConstants.IN_POSITION){
      pivotMotor.set(-0.2);
    }
    else{
      pivotMotor.set(0.35);
    }
  }
  //Stops pivot motor
  public void stopPivot(){
    pivotMotor.set(0);
  }

  public void resetEncoder(){
    pivotEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Creates SmartDashBoard pivotMotor Encoder Value
    SmartDashboard.putNumber("pivotMotor Encoder Value", getPosition());

  }
}
