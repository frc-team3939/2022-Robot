package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/**
 * Default command for the Drivetrain Subsystem. This command reads input from
 * the joystick and calculates angle and velocity for each of the swerve drive
 * modules along with the speed multiplier from the throttle. The joystick input
 * code includes a deadband of 20% for x and y, and the deadband for rotation is
 * handled by OI.
 */
public class DriveCommandSetValue extends CommandBase {

  double f;
  double s;
  double r;
  double speedMult;

  int schedule_count = 0;

  public DriveCommandSetValue(double forward, double strafe, double rotation, double speedMultiplier) {
    addRequirements(Robot.drive);
    f = forward;
    s = strafe;
    r = rotation;
    speedMult = speedMultiplier;
  }

  /**
   * Turn all of the wheels to be facing forward and stop the drive motors.
   */
  @Override
  public void initialize() {
    // All on scale -1.0 to 1.0
    Robot.drive.drive(f, s, r, speedMult);
  }

  /**
   * Read the joystick input and set the angle and velocity for each of the drive
   * modules.
   */
  @Override
  public void execute() {
  }
  
  public boolean isFinished() {
    return true;
  }

  protected void end() {
    
  }
  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
  protected void interrupted() {
    end();
  }
}
