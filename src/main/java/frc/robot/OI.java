package frc.robot;

import java.lang.Math;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.commandgroups.AutoShootCommandGroup;
import frc.commandgroups.FireTwoBalls;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveCommandSetValue;
import frc.robot.commands.Reset_Gyro_Command;
import frc.robot.commands.Sync_Encoder;
import frc.robot.commands.TurnToVision;
import frc.robot.commands.Turn_to_Angle_Command;
import frc.robot.commands.Turn_to_Angle_New;
import frc.robot.commands.Intake.ExtendIntake;
import frc.robot.commands.Intake.IntakeRunVariableSpeed;
import frc.robot.commands.Intake.RetractIntake;
import frc.robot.commands.Intake.RunMiddle;
import frc.robot.commands.Intake.RunMiddleAndIntake;
import frc.robot.commands.Shoot.AutoShootCommand;
import frc.robot.commands.Shoot.AutoShootGroup;
import frc.robot.commands.Shoot.ExpungeWrongColorShooter;
import frc.robot.commands.Shoot.HomeAngleLimitSwitchCommand;
import frc.robot.commands.Shoot.MoveToAngleCommand;
import frc.robot.commands.Shoot.ResetAngleCommand;
import frc.robot.commands.Shoot.ShootCommand;
import frc.robot.commands.Shoot.adjustAngleCommand;
import frc.robot.commands.climber.ExtendRetractClimber;
import frc.robot.commands.climber.HomeClimber;
import frc.robot.commands.climber.ResetEncoder;
import frc.robot.commands.climber.StopWinch;
import frc.robot.commands.climber.WinchPullPosition;

import static frc.robot.RobotMap.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  Joystick stick;
  Joystick stick2;
  Joystick stick3;

  Button button1;
  Button button2;
  Button button3;
  Button button4;
  Button button5;
  Button button6;
  Button button7;
  Button button8;
  Button button9;
  Button button10;
  Button button11;
  Button button12;

  Button button21;
  Button button22;
  Button button23;
  Button button24;
  Button button25;
  Button button26;
  Button button27;
  Button button28;
  Button button29;
  Button button210;

  Button button31;
  Button button32;
  Button button33;
  Button button34;
  Button button35;
  Button button36;
  Button button37;
  Button button38;
  Button button39;
  Button button310;
  POVButton up;
  POVButton right;
  POVButton down;
  POVButton left;
  /**
   * Initialize the joystick and it's buttons. After initialization, attach any
   * commands to there buttons.
   */
  public OI() {
    stick = new Joystick(joystickId);
    stick2 = new Joystick(joystick2Id);
    stick3 = new Joystick(joystick3Id);

    // Joystick 1 Buttons
    up = new POVButton(stick, 0);
    right = new POVButton(stick, 90);
    down = new POVButton(stick, 180);
    left = new POVButton(stick, 270);
    button1 = new JoystickButton(stick, 1);
    button2 = new JoystickButton(stick, 2);
    button3 = new JoystickButton(stick, 3);
    button4 = new JoystickButton(stick, 4);
    button5 = new JoystickButton(stick, 5);
    button6 = new JoystickButton(stick, 6);
    button7 = new JoystickButton(stick, 7);
    button8 = new JoystickButton(stick, 8);
    button9 = new JoystickButton(stick, 9);
    button10 = new JoystickButton(stick, 10);
    button11 = new JoystickButton(stick, 11);
    button12 = new JoystickButton(stick, 12);
    // Joystick 2 Buttons
    button21 = new JoystickButton(stick2, 1);
    button22 = new JoystickButton(stick2, 2);
    button23 = new JoystickButton(stick2, 3);
    button24 = new JoystickButton(stick2, 4);
    button25 = new JoystickButton(stick2, 5);
    button26 = new JoystickButton(stick2, 6);
    button27 = new JoystickButton(stick2, 7);
    button28 = new JoystickButton(stick2, 8);
    button29 = new JoystickButton(stick2, 9);
    button210 = new JoystickButton(stick2, 10);
    
    // Joystick 3 Buttons
    button31 = new JoystickButton(stick3, 1);
    button32 = new JoystickButton(stick3, 2);
    button33 = new JoystickButton(stick3, 3);
    button34 = new JoystickButton(stick3, 4);
    button35 = new JoystickButton(stick3, 5);
    button36 = new JoystickButton(stick3, 6);
    button37 = new JoystickButton(stick3, 7);
    button38 = new JoystickButton(stick3, 8);
    button39 = new JoystickButton(stick3, 9);
    button310 = new JoystickButton(stick3, 10);

    // Joystick 1 Actions
    up.whenHeld(new DriveCommandSetValue(0.3, 0, 0, 0.5));
    right.whenHeld(new DriveCommandSetValue(0, 0.3, 0, 0.5));
    down.whenHeld(new DriveCommandSetValue(-0.3, 0, 0, 0.5));
    left.whenHeld(new DriveCommandSetValue(0, -0.3, 0, 0.5));
    button1.whileHeld(new RunMiddleAndIntake());
    button2.whileHeld(new AutoShootCommand());
    button3.whileHeld(new ShootCommand(SmartDashboard.getNumber("Shooter Speed Testing", 0)));
    button4.whenPressed(new TurnToVision(Robot.drive)); // autoshoot but no shoot values yet
    button5.whenPressed(new ExtendIntake()); // extend intake
    button6.whenPressed(new RetractIntake()); // retract intake
    button7.whenPressed(new Reset_Gyro_Command());
    button9.whenPressed(new HomeClimber());
    button10.whenPressed(new SequentialCommandGroup(new adjustAngleCommand(400), new ShootCommand(0.55))); // next to target
    //button11.whileHeld(new AutoShootGroup(SmartDashboard.getNumber("Target Distance", 0)));
    button11.whileHeld(new AutoShootGroup());
    button12.whenPressed(new Sync_Encoder());




    button21.whenPressed(new WinchPullPosition(0, true)); // starting slow for now, 25%
    button22.whenPressed(new ExtendRetractClimber(true)); //ANGLES CLIJMBer ARMS
    button23.whenPressed(new ExtendRetractClimber(false)); //UPRIGHTS CLIMBER ARMS
    button24.whenPressed(new ResetEncoder());
    button25.whenPressed(new StopWinch()); // faster pull speed
    button26.whenPressed(new MoveToAngleCommand(0, true));
    button27.whenPressed(new ResetAngleCommand());
    button28.whenPressed(new HomeAngleLimitSwitchCommand());
    button29.whenPressed(new ShootCommand(SmartDashboard.getNumber("Shooter Speed Testing", 0)));
    button210.whenPressed(new ShootCommand(0));
    
    button35.whenPressed(new RunMiddle(0.5, false, 0, true));
    button36.whenPressed(new RunMiddle(0, false, 0 , false));
    button37.whenPressed(new IntakeRunVariableSpeed(0, true));
    button38.whenPressed(new IntakeRunVariableSpeed(0, false));
    button39.whenPressed(new IntakeRunVariableSpeed(-0.6, false)); // reverse
    /*button31.whenPressed(new ShooterSpeedCommand(0.2, 1, false)); //FULL POWER
    button32.whenPressed(new RunMiddle(false, 0)); //runs middle motor
    button33.whenPressed(new FireTwoBalls()); // fires two balls
    button34.whenPressed(new ExpungeWrongColorShooter());
    button35.whenPressed(new HomeAngleLimitSwitchCommand());*/
    
//button1.whenPressed(new MatchLocANDAbsEncoderCommand(Robot.drive));
    //button3.whenPressed(new SetAngle(100, 0.7));
    // button3.whenReleased(new camera_Command());
    // button4.whenPressed(new AimShooterCommand(-4.2, true));//12.9 is for level 1
    // at blue line, 7
    // button5.whenPressed(new SpinShooterCommand(1000)); //pull legs up
    // button6.whenPressed(new camera_Command());
    // button7.whenPressed(new AimShooterCommand(0, true));
    // button8.whenPressed(new AimShooterCommand(-.25, false));
    // button9.whenPressed(new AimShooterCommand(.25, false));
    // button10.whenPressed(new MoveShooterSolenoidCommand(false));
    // button11.whenPressed(new IntakeGroup(true));
    // button12.whenPressed(new IntakeGroup(false));

    // Joystick 2 Actions

    // button21.whenPressed(new MoveLegsCommand(true, true, .25, false));
    // button22.whenPressed(new MoveLegsCommand(true, true, -.25, false));
    // button23.whenPressed(new MoveLegsCommand(true, true, 5, false));
    // button24.whenPressed(new MoveLegsCo`mmand(true, true, -5, false));
    // button25.whenPressed(new PistonMoveCommand());
    // button26.whenPressed(new MoveLegsCommand(true, true, 0, true));

    // button21.whenPressed(new SpinShooterCommand(25, true));
    // button21.whenReleased(new MoveShooterSolenoidCommand(false));
    // button21.whileHeld(new SpinIntakeCommand(.3));
    // button21.whenReleased(new SpinIntakeCommand(0));
    // button22.whenPressed(new SpinShooterCommand(-25, true));
    // button22.whenReleased(new MoveShooterSolenoidCommand(false));
    // button22.whileHeld(new SpinIntakeCommand(.60));
    // button22.whenReleased(new SpinIntakeCommand(0));
    // button23.whenPressed(new SpinShooterCommand(550, false));
    // button23.whenReleased(new MoveShooterSolenoidCommand(false));
    // button24.whenPressed(new SpinShooterCommand(650, false));
    // button24.whenReleased(new MoveShooterSolenoidCommand(false));
    // button25.whenPressed(new SpinShooterCommand(0, false));
    // button26.whenPressed(new KickCommand(1));
    // button26.whileHeld(new SpinIntakeCommand(.5));
    // button26.whenReleased(new AimShooterCommand(-3.375, true));
    // button27.whenPressed(new AimShooterCommand(-10.2, true));
    // button28.whenPressed(new AimShooterCommand(-4.2, true));
    // button29.whenPressed(new AimShooterCommand(-23.25, true));
    // button210.whenPressed(new AimShooterCommand(-1, true));
    // button3.whenPressed(new SpinShooterCommand(100));
    // button4.whenPressed(new SpinShooterCommand(0));
    // button5.whenPressed(new SpinIntakeCommand(.25));
    // button6.whenPressed(new SpinIntakeCommand(0));

    // Joystick 3 Actions
    // button31.whenPressed(new Move_Legs_Command(true));
    // button32.whenPressed(new Move_Legs_Command(false));
    // button33.whenPressed(new HomeCommandGroup());
    // button35.whenPressed(new HomeShooterAngleCommand());
    // button36.whenPressed(new LiftHatchCommand(0, true));
    // button37.whenPressed(new Pickup_Hatch());
    // button38.whenPressed(new Dropoff());
    // button39.whenPressed(new LiftHatchCommand(1720, true));
    // button310.whenPressed(new LiftHatchCommand(13600, true));
    // button311.whenPressed(new LiftHatchCommand(25300, true));

  }

  /**
   * Get the X axis from the Joystick. This is on the scale -1.0 to 1.0
   *
   * @return the X axis value
   */
  public double getX() {
    return stick.getX();
  }

  /**
   * Get the Y axis from the Joystick. This is on the scale -1.0 to 1.0
   *
   * @return the Y axis value
   */
  public double getY() {
    return stick.getY();
  }

  /**
   * Get the X axis from the Joystick. This is on the scale -1.0 to 1.0
   *
   * @return the X axis value
   */
  public double get2X() {
    return stick2.getX();
  }

  /**
   * Get the Y axis from the Joystick. This is on the scale -1.0 to 1.0
   *
   * @return the Y axis value
   */
  public double get2Y() {
    return stick2.getY();
  }

  /**
   * Get the Twist axis from the Joystick. This is on the scale -1.0 to 1.0 and
   * has a deadband of 20%.
   *
   * @return the Twist axis value
   */
  public double getTwist() {
    return deadband(stick.getTwist(), 0.2);
  }

  /**
   * Get the Throttle axis from the Joystick. This is on the scale -1.0 to 1.0
   *
   * @return the Throttle axis value
   */
  public double getThrottle() {
    return -stick.getThrottle();
  }

  /**
   * Helper method for applying a deadband to an axis.
   *
   * @param input the axis value on the scale -1.0 to 1.0
   * @param pad   the deadband percent as a decimal 0.0 to 1.0
   * @return the input unless it is within the deadband, then 0 is returned.
   */
  private double deadband(double input, double pad) {
    if (Math.abs(input) > pad)
      return input;
    else
      return 0;
  }

  /*
   * Start the command when the button is pressed and let it run the command until
   * it is finished as determined by it's isFinished method.
   * button.whenPressed(new GoNumStepsCommand());
   *
   * Run the command while the button is being held down and interrupt it once the
   * button is released. button.whileHeld(new ExampleCommand());
   *
   * Start the command when the button is released and let it run the command
   * until it is finished as determined by it's isFinished method.
   * button.whenReleased(new ExampleCommand());
   */
}
