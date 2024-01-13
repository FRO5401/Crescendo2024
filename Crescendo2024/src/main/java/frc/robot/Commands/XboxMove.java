package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivebase;
import frc.robot.Controls;


public class XboxMove extends Command {
  /*** Variables ***/
    //Input Axes
  double turn;
  double throttle;
  double reverse;

    //Input Buttons
  boolean rotate;
  boolean brake;
  boolean precision;
  boolean gearShiftHigh;
  boolean gearShiftLow;

     //Testing Buttons
  boolean resetSensors;

    //Instance Vars
  double left;
  double right;
  double sensitivity;

  Drivebase drivebase;

  public XboxMove(Drivebase m_drivebase) {
    drivebase = m_drivebase;
    sensitivity = Constants.Controls.DEFAULT_SENSITIVTY;
    addRequirements(drivebase);
  }


  // Called just before this Command runs the first time
  @Override
  public void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {

    precision = Controls.xbox_driver.getRightBumper();
    brake = Controls.xbox_driver.getLeftBumper();
    rotate = Controls.xbox_driver.getLeftStickButton();
    throttle = Controls.xbox_driver.getRightTriggerAxis();
    reverse = Controls.xbox_driver.getLeftTriggerAxis();
    turn = Controls.xbox_driver.getLeftX() * 0.8;

      //Braking
       /*** Precision ***/
      //Hold for Precision Speed
      if(precision){
        sensitivity = 0.4;
      }
      else{
        sensitivity = 1.0;
      }
    /*** Driving ***/
      //Braking
    if(brake){
      left = 0;
      right = 0;
    }
        //Pirouetting (Turn in place).
      if(rotate){
          //If the joystick is pushed passed the threshold.
        if(Math.abs(turn) > Constants.Controls.AXIS_THRESHOLD){
            //Sets it to spin the desired direction.
          left = Constants.Controls.SPIN_SENSITIVITY * turn;
          right = Constants.Controls.SPIN_SENSITIVITY * (turn * -1);
        }
          //If its not past the threshold stop spinning
        else if(Math.abs(turn) < Constants.Controls.AXIS_THRESHOLD){
          left = 0;
          right = 0;
        }
      }
        //Not pirouetting (Not turning in place).
      else{
          //Turning right
        if(turn > Constants.Controls.AXIS_THRESHOLD){
            //Makes left slow down by a factor of how far the axis is pushed.
          left = (throttle - reverse) * sensitivity;
          right = (throttle - reverse) * sensitivity * (1 - turn);
        }
          //Turning left
        else if(turn < (-1 * Constants.Controls.AXIS_THRESHOLD)){
            //Makes right speed up by a factor of how far the axis is pushed.
          left = (throttle - reverse) * sensitivity  * (1 + turn);
          right = (throttle - reverse) * sensitivity;
        }
          //Driving straight
        else{
            //No joystick manipulation.
          left = (throttle - reverse) * sensitivity;
          right = (throttle - reverse) * sensitivity;
        }
      }

      //After speed manipulation, send to drivebase.
    drivebase.drive(left, right);
    //drivebase.pidDrive(left, right);
  }

  @Override
  public void end(boolean interrupted) {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
    public boolean runsWhenDisabled() {
      return false;
  }
}