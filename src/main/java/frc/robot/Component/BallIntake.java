package frc.robot.Component;

public class BallIntake 
{
    public BallIntake()
    {

    }

    private boolean _isStarted = false;

    public void InitIntake()
    {
        _isStarted = false;
    }

    public void StartIntake()
    {
        _isStarted = true;
    }
    public void StopIntake()
    {
        _isStarted = false;
    }

    public void DoIntake()
    {
        if(_isStarted)
        {

        }
    }
}
