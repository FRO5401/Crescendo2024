// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*Physical requirements
CANSparkmax called infeedMotor with an id of 6
Idlemode is set to break
Software requirements
Method that spins motor forward at 100% speed
Method that spins motor Backward at 100% speed 
*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;


public class Intake extends SubsystemBase {
  /** Creates a new Intake Motor. */
  CANSparkMax intakeMotor;

  public Intake() {
    //Sets intake motor
    intakeMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_ID, MotorType.kBrushless);
    //resets intake motor
    intakeMotor.restoreFactoryDefaults();
    //set ideal mode
    intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  public void Pickup(){
    //makes motor pickup cube
    intakeMotor.set(.60);
  }
  public void Putdown(){
    //Makes motor remove cube
    intakeMotor.set(-.60);
  }
  public void StopIntake(){
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
