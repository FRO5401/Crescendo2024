package frc.robot.Commands;

//WPI Imports
import edu.wpi.first.wpilibj2.command.Command;

//File Imports
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.Subsystems.Drivebase;
import frc.robot.Subsystems.Photonvision;

public class XboxMove extends Command {
  /*** Variables ***/
  // Input Axes
  private double turn;
  private double throttle;
  private double reverse;

  // Input Buttons
  private boolean rotate;
  private boolean brake;
  private boolean precision;
  private boolean autoTargetSpeaker;
  private boolean autoTargetNote;



  // Testing Buttons
  boolean resetSensors;

  // Instance Vars
  double left;
  double right;
  double sensitivity;

  private double distance;
  private double angle;

  Drivebase drivebase;

  private Photonvision backCamera;
  private Photonvision frontCamera;

  


  public XboxMove(Drivebase m_drivebase,Photonvision m_backCamera, Photonvision m_frontCamera) {
    drivebase = m_drivebase;
    frontCamera = m_frontCamera;
    backCamera = m_backCamera;
    sensitivity = Constants.Controls.DEFAULT_SENSITIVTY;
    addRequirements(drivebase, frontCamera, backCamera);
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
    autoTargetSpeaker = Controls.xbox_driver.getAButton();
    autoTargetNote = Controls.xbox_driver.getBButton();
    turn = Controls.xbox_driver.getLeftX();

    // Braking
    /*** Precision ***/
    
    // Hold for Precision Speed
    if (precision) {
      sensitivity = Constants.DriveMotors.PERCISION_SENSITIVITY;
    } else {
      sensitivity = Constants.DriveMotors.DEFAULT_SENSITIVTY;
    }
    /*** Driving ***/
    // Braking
    if (brake) {
      left = Constants.DriveMotors.POWER_STOP;
      right = Constants.DriveMotors.POWER_STOP;
    }

    if (autoTargetSpeaker){
      if(backCamera.hasTarget()){
        distance = backCamera.getDistance();
        angle = backCamera.getYaw();
        drivebase.arcadeDrive(drivebase.calculateFoward(1, distance), drivebase.calculateTurn(-6, angle));
      }
    }

    // Pirouetting (Turn in place).
    if (rotate) {
      // If the joystick is pushed passed the threshold.
      if (Math.abs(turn) > Constants.Controls.AXIS_THRESHOLD) {
        // Sets it to spin the desired direction.
        left = Constants.Controls.SPIN_SENSITIVITY * turn;
        right = Constants.Controls.SPIN_SENSITIVITY * (turn * Constants.DriveMotors.INVERSE_DIRECTION);
      }
      // If its not past the threshold stop spinning
      else if (Math.abs(turn) < Constants.Controls.AXIS_THRESHOLD) {
        left = Constants.DriveMotors.POWER_STOP;
        right = Constants.DriveMotors.POWER_STOP;
      }
    } 


    // Not pirouetting (Not turning in place).
    else {
      // Turning right
      if (turn > Constants.Controls.AXIS_THRESHOLD) {
        // Makes left slow down by a factor of how far the axis is pushed.
        left = (throttle - reverse) * sensitivity;
        right = (throttle - reverse) * sensitivity * (Constants.DriveMotors.STRAIGHT_DIRECTION - turn);
      }
      // Turning left
      else if (turn < (Constants.DriveMotors.INVERSE_DIRECTION * Constants.Controls.AXIS_THRESHOLD)) {
        // Makes right speed up by a factor of how far the axis is pushed.
        left = (throttle - reverse) * sensitivity * (Constants.DriveMotors.STRAIGHT_DIRECTION + turn);
        right = (throttle - reverse) * sensitivity;
      }
      // Driving straight
      else {
        // No joystick manipulation.
        left = (throttle - reverse) * sensitivity;
        right = (throttle - reverse) * sensitivity;
      }
    }

    // After speed manipulation, send to drivebase.
    drivebase.drive(left, right);
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
