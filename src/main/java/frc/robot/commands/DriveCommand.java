package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.PID_DrivetrainSubsystem;

import static frc.robot.Robot.*;

/**
 * Default command for the Drivetrain Subsystem. This command reads input from
 * the joystick and calculates angle and velocity for each of the swerve drive
 * modules along with the speed multiplier from the throttle. The joystick input
 * code includes a deadband of 20% for x and y, and the deadband for rotation is
 * handled by OI.
 */
public class DriveCommand extends CommandBase {

  private final PID_DrivetrainSubsystem drive;

  public DriveCommand(PID_DrivetrainSubsystem subsystem) {
    drive = subsystem;
    addRequirements(drive);

  }

  /**
   * Turn all of the wheels to be facing forward and stop the drive motors.
   */
  @Override
  public void initialize() {
    /*drive.frontRight.setAngle(1);
    drive.frontLeft.setAngle(1);
    drive.backLeft.setAngle(1);
    drive.backRight.setAngle(1);

    drive.frontRight.setSpeed(0);
    drive.frontLeft.setSpeed(0);
    drive.backLeft.setSpeed(0);
    drive.backRight.setSpeed(0);
    */
  }

  /**
   * Read the joystick input and set the angle and velocity for each of the drive
   * modules.
   */
  @Override
  public void execute() {
    // All on scale -1.0 to 1.0
    double f = m_oi.getY();
    double s = -m_oi.getX();
    double r = m_oi.getTwist();
    double speedMult = Robot.m_oi.getThrottle() / 2.0 + 0.5;

    Robot.drive.drive(f, s, r, speedMult);
  }

  /**
   * Always false, Always run
   */
  
  public boolean isFinished() {
    return false;
  }

  protected void end() {
    
  }
  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
  protected void interrupted() {
    end();
  }
}
