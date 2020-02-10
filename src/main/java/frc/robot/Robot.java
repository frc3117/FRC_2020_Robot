package frc.robot;

import frc.robot.Component.AutonomousSequence;
import frc.robot.Component.BallIntake;
import frc.robot.Component.BallThrower;
import frc.robot.Component.ColorSensor;
import frc.robot.Component.PneumaticSystem;
import frc.robot.Component.Swerve;
import frc.robot.Component.Data.AutonomousSequenceAction;
import frc.robot.Component.Data.Input;
import frc.robot.Component.Data.RobotOdometry;
import frc.robot.Component.Data.WheelData;
import frc.robot.Math.PID;
import frc.robot.Math.Timer;

import java.sql.Time;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.Vector2d;

public class Robot extends TimedRobot {

  private final ColorSensor ColorSensor = new ColorSensor(0, 7);

  public static Swerve SwerveDrive;
  public static BallThrower Thrower;
  public static BallIntake Intake;

  public static RobotOdometry Odometry;

  public static PID DirectionHoldPID = new PID(3, 0, 0);
  public static PID PositionHoldPID = new PID(0.3, 0, 0);

  public Servo servo = new Servo(0);

  @Override
  public void robotInit() {
    //Initializing the SwerveDrive drivetrain
    WheelData[] data = {
      new WheelData(20, 7, new Vector2d(0, 1), 0, 3, new Vector2d(-10, -11), 0.6964067- (3.1415/2.0)),
      new WheelData(21, 8, new Vector2d(4, 5), 1, 0, new Vector2d(10, -11), 4.163101- (3.1415/2.0)),
      new WheelData(22, 9, new Vector2d(2, 3), 2, 1, new Vector2d(10, 11), 0.8344609- (3.1415/2.0)),
      new WheelData(23, 4, new Vector2d(6, 7), 3, 2, new Vector2d(-10, 11), 3.1476357- (3.1415/2.0))
    };

    SwerveDrive = new Swerve(data, new Joystick(0));
    SwerveDrive.SetCurrentMode(Swerve.DrivingMode.World);

    SwerveDrive.SetPIDGain(0, 1, 0, 0);
    SwerveDrive.SetPIDGain(1, 1, 0, 0);
    SwerveDrive.SetPIDGain(2, 1, 0, 0);
    SwerveDrive.SetPIDGain(3, 1, 0, 0);

    SwerveDrive.SetRateLimiter(100000);
    SwerveDrive.SetRotationRateLimiter(100000);

    SwerveDrive.SetDeadzone(0.2);
    SwerveDrive.InitIMU();

    Thrower = new BallThrower(SwerveDrive, 4, 2, 500, 2700);
    Intake = new BallIntake(6, 6, 7, 4, 4, 1000);

    Odometry = new RobotOdometry(0.05, 1.2192, 6.7056);

    Input.CreateButton("ServoUp", 0, 6);
    Input.CreateButton("ServoDown", 0, 7);
  }

  
  @Override
  public void teleopInit() 
  {
    //Initialize all the robot component before a match
    ColorSensor.Init();

    SwerveDrive.RecalibrateIMU();

    Thrower.Init();
    Intake.Init();

    PneumaticSystem.Init();
    Timer.Init();

    Odometry.SetPosition(new Vector2d(0, 0));

    AutonomousSequence autonomousSequence = new AutonomousSequence(new Vector2d(0, 0), 
    AutonomousSequenceAction.CreateMoveTo(5, new Vector2d(0, 1)),
    AutonomousSequenceAction.CreateMoveTo(5, new Vector2d(0, 0)),
    AutonomousSequenceAction.CreateMoveTo(5, new Vector2d(0, 1)),
    AutonomousSequenceAction.CreateMoveTo(5, new Vector2d(0, 0)),
    AutonomousSequenceAction.CreateMoveTo(5, new Vector2d(0, 1)),
    AutonomousSequenceAction.CreateMoveTo(5, new Vector2d(0, 0)),
    AutonomousSequenceAction.CreateMoveTo(5, new Vector2d(0, 1)),
    AutonomousSequenceAction.CreateMoveTo(5, new Vector2d(0, 0))
    );

    //autonomousSequence.StartSequence();
  }

  @Override
  public void teleopPeriodic() { 
    //Calculate the first time after use GetDeltaTime()
    Timer.Calculate();

    //Execute Needed Component
    //PneumaticSystem.CheckPressure();
    
    Thrower.DoThrower();
    Intake.DoIntake();

    SwerveDrive.DoSwerve();
    //Odometry.DoOdometry();

    double val = servo.get();
    if(Input.GetButton("ServoUp"))
    {
      servo.set(val + 0.2 * Timer.GetDeltaTime());
    }
    else if(Input.GetButton("ServoDown"))
    {
      servo.set(val + -0.2 * Timer.GetDeltaTime());
    }

    //System.out.println(Odometry.GetPosition().x + " : " + Odometry.GetPosition().y);
    //ColorSensor.DoColorSensor();;
  } 
}