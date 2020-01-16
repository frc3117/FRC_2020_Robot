/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Component;

import frc.robot.Component.Data.AutonomousSequenceAction;

public class AutonomousSequence 
{
    public AutonomousSequence(AutonomousSequenceAction... Actions)
    {
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

                        break;

                        case RotateToward:

                        break;

                        case Rotate:

                        break;

                        case Translate:

                        break;

                        case MoveTo:

                        break;
                    }

                    i++;
                }
            }
        }
    }
}
