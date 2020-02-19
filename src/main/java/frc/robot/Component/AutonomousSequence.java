package frc.robot.Component;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.Robot;
import frc.robot.Component.Data.AutonomousSequenceAction;
import frc.robot.Component.Data.Input;
import frc.robot.Math.Mathf;
import frc.robot.Math.Timer;

public class AutonomousSequence 
{
    public AutonomousSequence(Vector2d InitialPosition, AutonomousSequenceAction... Actions)
    {
        Robot.SwerveDrive.SetPosition(InitialPosition);

        _actions = Actions.clone();
    }

    private AutonomousSequenceAction[] _actions;
    private boolean _isRunning = false;

    private String _killButton = null;

    public void SetKillButton(String ButtonName)
    {
        if(Input.ContainButton(_killButton))
        {
            _killButton = ButtonName;
            return;
        }

        System.out.println("Button Not Registered");
    }

    public void StartSequence()
    {
        _isRunning = true;

        AutonomousThread();
    }
    public void StopSequencec()
    {
        _isRunning = false;
    }

    private void AutonomousThread()
    {
        int i = 0;
        double waitTime = 0;

        double targetAngle = 0;
        Vector2d targetPosition = Robot.SwerveDrive.GetPostion();
        int targetShiftGear = 0;

        Vector2d lookAtTarget = new Vector2d(0, 0);
        boolean isLookAt = false;

        while(_isRunning && DriverStation.getInstance().isEnabled())
        {
            //Button to kill the sequence
            if(_killButton != null && Input.GetButton(_killButton))
            {
                break;
            }

            try 
            {
                //Wait so time betwen "frame"
                edu.wpi.first.wpilibj.Timer.delay(0.02);

                waitTime -= 0.02;
            } catch (Exception e) {}

            Timer.Calculate();

            //Call the next action if the wait time is over
            if(waitTime <= 0)
            {
                if(i < _actions.length)
                {
                    AutonomousSequenceAction currentAction = _actions[i];

                    waitTime = currentAction.GetTime();

                    switch(currentAction.GetType())
                    {
                        case Wait:
                        //Nothing Else
                        break;

                        case RotateToward:
                        targetAngle = currentAction.GetNumber();

                        isLookAt = false;
                        break;

                        case Rotate:
                        targetAngle += currentAction.GetNumber();      
                        
                        isLookAt = false;
                        break;

                        case LookAt:
                        isLookAt = true;

                        lookAtTarget = currentAction.GetVector();
                        break;

                        case Translate:
                        targetPosition = new Vector2d(targetPosition.x + currentAction.GetVector().x, targetPosition.y + currentAction.GetVector().y);
                        break;

                        case MoveTo:
                        targetPosition = currentAction.GetVector();
                        break;

                        case ShiftTo:
                        targetShiftGear = (int)currentAction.GetNumber();
                        break;
                    }

                    i++;
                }
            }

            Vector2d pos = Robot.Odometry.GetPosition();          

            if(isLookAt)
            {
                Vector2d relative = new Vector2d(lookAtTarget.x - pos.x, lookAtTarget.y - pos.y);

                targetAngle = Math.atan2(relative.y, relative.x);
            }

            //Override the robot commands to match the currents target
            Robot.SwerveDrive.OverrideRotationAxis(Robot.DirectionHoldPID.Evaluate(Mathf.DeltaAngle(Robot.SwerveDrive.GetHeading(), targetAngle), Timer.GetDeltaTime()));

            Robot.SwerveDrive.OverrideHorizontalAxis(Robot.PositionHoldPID.Evaluate(targetPosition.x - pos.x, Timer.GetDeltaTime()));
            Robot.SwerveDrive.OverrideVerticalAxis(Robot.PositionHoldPID.Evaluate(targetPosition.y - pos.y, Timer.GetDeltaTime()));

            Robot.SwerveDrive.OverrideShift(targetShiftGear);

            Robot.SwerveDrive.DoSystem();
            Robot.Odometry.DoOdometry();
        }
    }
}
