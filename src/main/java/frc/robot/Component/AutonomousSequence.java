/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Component;

import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.Robot;
import frc.robot.Component.Data.AutonomousSequenceAction;
import frc.robot.Math.Mathf;
import frc.robot.Math.Timer;

public class AutonomousSequence 
{
    public AutonomousSequence(Vector2d InitialPosition, AutonomousSequenceAction... Actions)
    {
        Robot.SwerveDrive.SetPosition(InitialPosition);

        _actions = Actions.clone();
    }

    private Thread _thread;

    private AutonomousSequenceAction[] _actions;
    private boolean _isRunning = false;

    public void StartSequence()
    {
        _thread = new Thread(() -> AutonomousThread());

        _isRunning = true;
        _thread.start();
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

        while(_isRunning)
        {
            try 
            {
                Thread.sleep(20);

                waitTime -= 0.02;
            } catch (Exception e) {}

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

            Vector2d pos = Robot.SwerveDrive.GetPostion();
       
            if(isLookAt)
            {
                Vector2d relative = new Vector2d(lookAtTarget.x - pos.x, lookAtTarget.y - pos.y);

                targetAngle = Math.atan2(relative.y, relative.x);
            }

            Robot.SwerveDrive.OverrideRotationAxis(Robot.DirectionHoldPID.Evaluate(Mathf.DeltaAngle(Robot.SwerveDrive.GetHeading(), targetAngle), Timer.GetDeltaTime()));

            Robot.SwerveDrive.OverrideHorizontalAxis(Robot.PositionHoldPID.Evaluate(targetPosition.x - pos.x, Timer.GetDeltaTime()));
            Robot.SwerveDrive.OverrideVerticalAxis(Robot.PositionHoldPID.Evaluate(targetPosition.y - pos.y, Timer.GetDeltaTime()));

            Robot.SwerveDrive.OverrideShift(targetShiftGear);
        }
    }
}
