// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.PID_DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToVision extends PIDCommand {

  /**
   * This turn the robot to the desired angle (This value is absolute!!!)
   * 
   * @param targetAngleDegrees Angle that you want to turn to 
   * @param drive Pass in the Swerve drive subsystem
   */
  public TurnToVision(PID_DrivetrainSubsystem drive) {
    super(
        // The controller that the command will use
        new PIDController(RobotMap.turnkP, RobotMap.turnkI, RobotMap.turnkD),
        //new PIDController(SmartDashboard.getNumber("turnkP", 0), SmartDashboard.getNumber("turnkI", 0), SmartDashboard.getNumber("turnkD", 0)),
        // This should return the measurement
        drive::gettx,
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> drive.drive(0, 0, output, .5), 
        
        drive);

        addRequirements(Robot.drive);
        getController().enableContinuousInput(-180, 180); //Input is allowed between -180 adn 180 degrees
        getController().setTolerance(0.5); //One Degree Tollernce
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
