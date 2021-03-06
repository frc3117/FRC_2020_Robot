package frc.robot.Component;

import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.Robot;
import frc.robot.Component.Data.LimeLightPosition;
import frc.robot.Interface.Component;
import frc.robot.Math.Mathf;
import frc.robot.Math.PID;
import frc.robot.Math.Timer;

public class Autonomous implements Component
{
    public enum AutonomousMode
    {
        ReverseShoot,
        CenterCell,
        Trench,
        TrenchLateral,
        EnnemyTrench
    }

    public Autonomous(AutonomousMode Mode)
    {
        _mode = Mode;
    }

    private AutonomousMode _mode;
    private LimeLightPosition _robotPosition = new LimeLightPosition(new Vector2d(0, 0));
    
    private PID _horizontalPositionPID;
    private PID _verticalPositionPID;

    private Vector2d _currentPos;

    private double _enterTime;
    private double _currentTime;

    public void Init()
    {
        _enterTime = Timer.GetCurrentTime();
    }
    public void DoSystem()
    {
        _currentPos = _robotPosition.Evaluate();
        _currentTime = Timer.GetCurrentTime() - _enterTime;

        switch(_mode)
        {
            case ReverseShoot:
            DoReverseShoot();
            break;

            case CenterCell:
            DoCenterCell();
            break;

            case Trench:
            case TrenchLateral:
            DoTrench();
            break;

            case EnnemyTrench:
            DoEnnemyTrench();
            break;
        }
        
        Robot.Thrower.DoSystem();
        Robot.Intake.DoSystem();
    
        Robot.SwerveDrive.DoSystem();

        Robot.Leds.DoSystem();
    }

    private void PositionPID(Vector2d TargetPos)
    {
        double xError = TargetPos.x - _currentPos.x;
        double yError = TargetPos.y - _currentPos.y;

        if(Math.abs(xError) <= 0.5)
        {
            xError = 0;
        }
        if(Math.abs(yError) <= 0.5)
        {
            yError = 0;
        }

        Robot.SwerveDrive.OverrideHorizontalAxis(Mathf.Clamp(_horizontalPositionPID.Evaluate(xError), -1, 1));
        Robot.SwerveDrive.OverrideRotationAxis(Mathf.Clamp(_verticalPositionPID.Evaluate(yError), -1, 1));
    }

    private void DoReverseShoot()
    {
        if(_currentTime <= 2)
        {
          double rotationAxis = Robot.DirectionHoldPID.Evaluate(Mathf.DeltaAngle(0 ,Robot.SwerveDrive.GetHeading()));
        
          Robot.SwerveDrive.OverrideVerticalAxis(0.8);
          Robot.SwerveDrive.OverrideRotationAxis(rotationAxis);
        }
        else if(_currentTime <= 11)
        {
            Robot.Thrower.StartOverrideAlign();
            if(_currentTime >= 5.5)
            {
                Robot.Thrower.SetAutoShoot(true);
            }
        }
        else
        {
            Robot.Thrower.SetAutoShoot(false);
            Robot.Thrower.StopOverrideAlign();
        }
    }
    private void DoCenterCell() 
    {
        if(_currentTime <= 3.5)
        {
            Robot.SwerveDrive.OverrideRotationAxis(Mathf.Clamp(Mathf.DeltaAngle(Robot.SwerveDrive.GetHeading(), 90 * Mathf.DEG_2_RAD), -1, 1));

            Robot.Intake.OpenIntake();
            Robot.SwerveDrive.OverrideVerticalAxis(0.9);
            Robot.SwerveDrive.OverrideHorizontalAxis(0.05);

            Robot.Intake.OverrideIntake(-1);
        }
        else if (_currentTime <= 4.5)
        {
            Robot.Intake.OverrideIntake(-1);
        }
        else if(_currentTime <= 14.5)
        {
            Robot.Thrower.StartOverrideAlign();
            if(_currentTime >= 9.5)
            {
                Robot.Thrower.SetAutoShoot(true);
            }
        }
        else
        {
            Robot.Thrower.SetAutoShoot(false);
            Robot.Thrower.StopOverrideAlign();
        }
    }
    private void DoTrench()
    {
        if(_currentTime <= 4.85)
        {
            Robot.SwerveDrive.OverrideRotationAxis(Mathf.Clamp(Mathf.DeltaAngle(Robot.SwerveDrive.GetHeading(), 90 * Mathf.DEG_2_RAD), -1, 1));

            Robot.Intake.OpenIntake();
            Robot.SwerveDrive.OverrideVerticalAxis(0.9);
            Robot.SwerveDrive.OverrideHorizontalAxis(0.05);

            Robot.Intake.OverrideIntake(-1);
        }
        else if(_currentTime <= 6)
        {
            if(_mode == AutonomousMode.TrenchLateral)
            {
                Robot.SwerveDrive.OverrideHorizontalAxis(1);
            }
            Robot.Intake.OverrideIntake(-1);
        }
        else if(_currentTime <= 14.5)
        {
            Robot.Thrower.StartOverrideAlign();
            if(_currentTime >= 10)
            {
                Robot.Thrower.SetAutoShoot(true);
            }
        }
        else
        {
            Robot.Thrower.SetAutoShoot(false);
            Robot.Thrower.StopOverrideAlign();
        }
    }
    private void DoEnnemyTrench()
    {
        
    }
}
