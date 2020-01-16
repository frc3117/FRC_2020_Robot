package frc.robot.Component;

import frc.robot.Component.Data.*;
import frc.robot.Math.PID;
import frc.robot.Math.Timer;
import frc.robot.Math.Mathf;

public class BallThrower 
{
    public BallThrower(Swerve swerve, int AllignButton, int ShootButton)
    {
        if(!Input.ContainButton("Allign"))
        {
            Input.CreateButton("Align", 0, AllignButton);
        }
        if(!Input.ContainButton("Shoot"))
        {
            Input.CreateButton("Shoot", 0, ShootButton);
        }

        _swerve = swerve;
        _elevationController = new MotorController(MotorController.MotorControllerType.TalonSRX, 6, false);

        //_directionPID.SetTolerancy(1);

        _limeLight.SetDriveMode();
    }

    private Swerve  _swerve;
    private MotorController _elevationController;
    private LimeLight _limeLight = new LimeLight();
    private PID _directionPID = new PID(0.06, 0.02, 0.002);
    private PID _elevationPID = new PID(0.035, 0, 0);

    private boolean _isAutoShoot = false;

    private boolean _isAllign = false;
    private boolean _isReady = false;

    private boolean _alignButtonLastState = false;

    private double _timeBetwenShot = 0.5;
    private double _errorTolerency = 5;

    private double _lastShotTime = 0;

    public void Init()
    {
        _isAutoShoot = false;
        _isAllign = false;
        _isReady = false;
    }

    public void DoThrower()
    {
        boolean allignButton = Input.GetButton("Align");
        if(allignButton && !_alignButtonLastState)
        {
            _isAllign = !_isAllign;

            if(_isAllign)
            {
                _limeLight.SetRecognitionMode();
            }
            else
            {
                _limeLight.SetDriveMode();
            }
        }
        _alignButtonLastState = allignButton;

        if(_isAllign)
        {
            LimeLightData currentData = _limeLight.GetCurrent();
            
            //Only try to align if there is a target in the line of sight
            if(currentData.IsTarget())
            {
                double rotationAxis = _directionPID.Evaluate(currentData.GetAngleX(), Timer.GetDeltaTime());
                double elevationAxis = _elevationPID.Evaluate(currentData.GetAngleY(), Timer.GetDeltaTime()) * -1;

              _swerve.OverrideRotationAxis(Mathf.Clamp(rotationAxis, -1, 1));
              _elevationController.Set(Mathf.Clamp(elevationAxis, -1, 1));
            }

            if((!_isAutoShoot || currentData.GetAngleX() + currentData.GetAngleY() <= _errorTolerency) && Timer.GetCurrentTime() - _lastShotTime >= _timeBetwenShot)
            {
                _isReady = true;
            }

            if(_isReady && (_isAutoShoot || Input.GetButton("Shoot")))
            {
                //StartShooting

                _lastShotTime = Timer.GetCurrentTime();

                Thread thread = new Thread(() -> ShooterThread());
                thread.start();
            }
        }
        else
        {
            _elevationController.Set(0);
            _directionPID.Reset();
            _elevationPID.Reset();
        }
    }
    private void ShooterThread()
    {
        
    }

    public void SetTimeBetwenShot(double time)
    {
        _timeBetwenShot = time; 
    }
    public void SetErrorTolerency(double tolerency)
    {
        _errorTolerency = tolerency;
    }
    public void SetAutoShoot(boolean state)
    {
        _isAutoShoot = state;
    }
}
