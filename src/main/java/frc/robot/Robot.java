package frc.robot;

import frc.robot.Component.BallIntake;
import frc.robot.Component.BallThrower;
import frc.robot.Component.PneumaticSystem;
import frc.robot.Component.Swerve;
import frc.robot.Component.Data.LimeLightPosition;
import frc.robot.Component.Data.RobotOdometry;
import frc.robot.Component.Data.WheelData;
import frc.robot.Math.PID;
import frc.robot.Math.Timer;

import java.sql.Time;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.Vector2d;

public class Robot extends TimedRobot {
  public static Swerve SwerveDrive;
  public static RobotOdometry Odometry;

  public static BallThrower Thrower;
  public static BallIntake Intake;

  public static PID DirectionHoldPID = new PID(3, 0, 0);
  public static PID PositionHoldPID = new PID(0.3, 0, 0);

  public static LimeLightPosition Position = new LimeLightPosition(new Vector2d(0, 0));

  private boolean _isInit = false;

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

    Thrower = new BallThrower(1, 2, 500, 3250);
    Intake = new BallIntake(6, 6, 7, 2, 3, -1200);
  }
  
  @Override
  public void autonomousInit() 
  {
    Init();

    currentStep = 0;
    _autonomousEnterTime = Timer.GetCurrentTime();
  }

  private double _autonomousEnterTime;
  private double _shootBallEnterTime;

  int currentStep = 0;
  String[] Step = {"CrossLine", "ShootBall"};
  @Override
  public void autonomousPeriodic() 
  {
    if(currentStep == Step.length)
    {
      Thrower.DoThrower();
      Intake.DoIntake();
  
      SwerveDrive.DoSwerve();
      return;
    }

    //Calculate the first time after use GetDeltaTime()
    Timer.Calculate();

    String current = Step[currentStep];

    Vector2d pos;

    switch(current)
    {
      case "CrossLine":
      if((Timer.GetCurrentTime() - _autonomousEnterTime <= 2))
      {
        SwerveDrive.OverrideVerticalAxis(0.8);
        //Thrower.StartOverrideAlign();

        SwerveDrive.OverrideRotationAxis(0.8);
      }
      else
      {
        currentStep++;
        _shootBallEnterTime = Timer.GetCurrentTime();
      }
      break;

      case "ShootBall":
      if(Timer.GetCurrentTime() - _shootBallEnterTime <= 2.5)
      {
        Thrower.StartOverrideAlign();
      }
      if(Timer.GetCurrentTime() - _shootBallEnterTime <= 10)
      {
        Thrower.SetAutoShoot(true);
      }
      else
      {
        Thrower.SetAutoShoot(false);
        Thrower.StopOverrideAlign();
        currentStep++;
      }
      break;
    }

    Thrower.DoThrower();
    Intake.DoIntake();

    SwerveDrive.DoSwerve();
  }

  @Override
  public void teleopInit() 
  {
    if(!_isInit)
    {
      Init();
    }

    Thrower.SetAutoShoot(false);
    Thrower.StopOverrideAlign();
    _isInit = false; //Put it to false so it init next time you enable the robot
  }

  public void Init()
  {
    //Initialize all the robot component before a match
    SwerveDrive.RecalibrateIMU();

    Thrower.Init();
    Intake.Init();
    
    PneumaticSystem.Init();
    Timer.Init();

    _isInit = true;

    //System.out.println(Position.GetCurrent().x + " " + Position.GetCurrent().x);
  }

  @Override
  public void teleopPeriodic() { 
    //Calculate the first time after use GetDeltaTime()
    Timer.Calculate();

    //Execute Needed Component
    PneumaticSystem.CheckPressure();
    
    Thrower.DoThrower();
    Intake.DoIntake();

    SwerveDrive.DoSwerve();
  } 
}