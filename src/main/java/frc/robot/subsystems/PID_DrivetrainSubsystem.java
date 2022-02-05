/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.RobotMap;
import frc.robot.util.DriveModule;
import static frc.robot.RobotMap.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;


/**
 * Control all 4 swerve drive modules TESTING
 */
public class PID_DrivetrainSubsystem extends PIDSubsystem {
  /**
   * Control all 4 swerve drive modules TESTING
   */
    public AHRS ahrs;
    public DriveModule frontLeft;
    public DriveModule frontRight;
    public DriveModule backLeft;
    public DriveModule backRight;
    public boolean inverse;
    public double pidOut;
    public double pidSet;

  public PID_DrivetrainSubsystem() {
    // Intert a subsystem name and PID values here
    super(new PIDController(RobotMap.turnkP, RobotMap.turnkI, RobotMap.turnkD));
    //Setup for the PID controller
    setOutputRange(-1.0, 1.0);
    //Other Setup
    inverse = false;
    ahrs = new AHRS(SPI.Port.kMXP);
    frontLeft = new DriveModule(LeftFrontDriveMotorId, LeftFrontAngleMotorId, LeftFrontEncoderId, LeftFrontEncoderOffset);
    frontRight = new DriveModule(RightFrontDriveMotorId, RightFrontAngleMotorId, RightFrontEncoderId, RightFrontEncoderOffset);
    backLeft = new DriveModule(LeftBackDriveMotorId, LeftBackAngleMotorId, LeftBackEncoderId, LeftBackEncoderOffset);
    backRight = new DriveModule(RightBackDriveMotorId, RightBackAngleMotorId, RightBackEncoderId, RightBackEncoderOffset);
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
  }

  private void setOutputRange(double d, double e) {
  }

  public void drive(double f, double s, double r, double speedMult){
    drive(f, s, r, speedMult, true);
  }
  public void drive(double f, double s, double r, double speedMult, Boolean dead){
    //Inverse
    if (inverse == true) {
      f = -f;
      s = -s;
      
      //r = -r;
    }

    // Deadband of 20%
     if (dead && Math.sqrt(f * f + s * s) < 0.2) {
      f = 0;
      s = 0;
    }

    // Stop moving but keep angle if joystick is centered
    if (f == 0 && s == 0 && r == 0) {
      frontRight.setSpeed(0);
      frontLeft.setSpeed(0);
      backLeft.setSpeed(0);
      backRight.setSpeed(0);
      //r = 1;
      return;
    }

    double a = s - r * (FrameLength / FrameDiagonal);
    double b = s + r * (FrameLength / FrameDiagonal);
    double c = f - r * (FrameWidth / FrameDiagonal);
    double d = f + r * (FrameWidth / FrameDiagonal);

    // Speed
    double ws1 = Math.sqrt(b * b + c * c);
    double ws2 = Math.sqrt(b * b + d * d);
    double ws3 = Math.sqrt(a * a + d * d);
    double ws4 = Math.sqrt(a * a + c * c);

    // Angle
    double wa1 = -Math.atan2(b, c) / Math.PI;
    double wa2 = -Math.atan2(b, d) / Math.PI;
    double wa3 = -Math.atan2(a, d) / Math.PI;
    double wa4 = -Math.atan2(a, c) / Math.PI;

    // Speed Limit
    double maxSpeed = Math.max(Math.max(ws1, ws2), Math.max(ws3, ws4));
    if (maxSpeed > 1.0) {
      ws1 /= maxSpeed;
      ws2 /= maxSpeed;
      ws3 /= maxSpeed;
      ws4 /= maxSpeed;
    }

    // Write angle and speed to each module
    backLeft.move(ws1, wa1);
    backRight.move(ws2, wa2);
    frontRight.move(ws3, wa3);
    frontLeft.move(ws4, wa4);

    // Speed multiplier from throttle
    frontRight.setSpeedMultiplier(speedMult);
    frontLeft.setSpeedMultiplier(speedMult);
    backLeft.setSpeedMultiplier(speedMult);
    backRight.setSpeedMultiplier(speedMult);
  }

  public double getAngle(){
    return ahrs.getAngle();
  }

  public void invertDirection(){
    if(inverse == true) {
      inverse = false;
    } else {
      inverse = true;
    }
    return;
  }

  public void gotoAngle(double f, double s, double angle, double speedMult){
    setSetpoint(angle);
    enable();
    drive(f, s, -pidOut, speedMult, true);
  }

  public void PIDdisable(){
    disable();
  }

  public void angleReset(){
    ahrs.reset();
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    pidOut = output;
    pidSet = setpoint;
    
  }

  @Override
  protected double getMeasurement() {
    return (ahrs.getAngle());
  }
 
  public void turnWheels90Right() {
    // TODO testing, edit this
    frontLeft.setAngle(0.5);
    frontRight.setAngle(0.5); 
    frontLeft.setSpeed(0.7);
    frontRight.setSpeed(0.7);
  }

public void resetEncoder() {
}

public void drivePosition(double p, int i) {
}

  /**
   * True will return the distance for the front sensor and false will return the back
   */
  /**
  public double getDistance(Boolean front){
    double volt;
    double mV_to_inch = 24;
    if (front == true) {
      volt = ultrafront.getVoltage() * 1000;
      return(volt / mV_to_inch);
    } else {
      volt = ultraback.getVoltage() * 1000;
      return (volt / mV_to_inch);
    }
  }
  */
  //public
}
