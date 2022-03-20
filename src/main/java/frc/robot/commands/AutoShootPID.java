// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.PID_DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootPID extends PIDCommand {
  /** Creates a new AutoShootPID. */
  public AutoShootPID(double distance, PID_DrivetrainSubsystem drive) {
    super(
        // The controller that the command will use
        new PIDController(RobotMap.turnkP, RobotMap.turnkI, RobotMap.turnkD),
        // This should return the measurement
        drive::getDistance,
        // This should return the setpoint (can also be a constant)
        distance,
        // This uses the output
        output -> drive.drive(output, 0, 0, .5),
        
        drive);
        
      addRequirements(Robot.drive);
      getController().enableContinuousInput(-1, 1);
      getController().setTolerance(0.8);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
