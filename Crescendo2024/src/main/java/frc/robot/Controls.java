package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class Controls {
  public static final String DriveConstants = null;
// The driver's controller
  public static XboxController xbox_driver = new XboxController(Constants.Controls.XBOX_CONTROLLER_DRIVER);
// The operator's controller 
  public static XboxController xbox_operator = new XboxController(Constants.Controls.XBOX_CONTROLLER_OPERATOR);
// Creates a CommandXboxController on port 0.
  public static CommandXboxController driver = new CommandXboxController(0); 
// Creates a CommandXboxController on port 1.
  public static CommandXboxController operator = new CommandXboxController(1); 
}