package frc.robot.Component;

import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.Robot;
import frc.robot.Component.Data.LimeLightPosition;
import frc.robot.Interface.System;
import frc.robot.Math.Mathf;
import frc.robot.Math.PID;
import frc.robot.Math.Timer;

public class Autonomous implements System
{
    public enum AutonomousMode
    {
        Default,
        Trench,
    }

    public Autonomous(AutonomousMode Mode)
    {
        _mode = Mode;
    }

    private AutonomousMode _mode;
    private LimeLightPosition _robotPosition;
    
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
            case Default:
            DoDefault();
            break;

            case Trench:
            DoTrench();
            break;
        }
        
        Robot.Thrower.DoSystem();
        Robot.Intake.DoSystem();
    
        Robot.SwerveDrive.DoSystem();
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

    private void DoTrench()
    {
        
    }
    private void DoDefault()
    {

    }
}
