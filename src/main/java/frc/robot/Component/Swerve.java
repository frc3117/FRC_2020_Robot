package frc.robot.Component;

import frc.robot.Component.Data.Input;
import frc.robot.Component.Data.InputManager;
import frc.robot.Component.Data.MotorController;
import frc.robot.Component.Data.RobotPosition;
import frc.robot.Component.Data.WheelData;
import frc.robot.Component.Data.MotorController.MotorControllerType;
import frc.robot.Interface.System;
import frc.robot.Math.Mathf;
import frc.robot.Math.PID;
import frc.robot.Math.Polar;
import frc.robot.Math.RateLimiter;
import frc.robot.Math.Timer;

import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve implements System {
    public Swerve(WheelData[] WheelsData)
    {
        _wheelCount = WheelsData.length;

        _driveMotor = new MotorController[_wheelCount];
        _directionMotor = new MotorController[_wheelCount];
        _directionEncoder = new AnalogInput[_wheelCount];
        _shifterValve = new Solenoid[_wheelCount];
        _rotationVector = new Vector2d[_wheelCount];
        _wheelPosition = new Vector2d[_wheelCount];
        _directionPID = new PID[_wheelCount];
        _angleOffset = new double[_wheelCount];

        _flipAngleOffset = new double[_wheelCount];
        _flipDriveMultiplicator = new double[_wheelCount];

        _horizontalRateLimiter = new RateLimiter(10000, 0);
        _verticaRateLimiter = new RateLimiter(10000, 0);
        _rotationRateLimiter = new RateLimiter(10000, 0);

        //Initializing all component of the swerve swerve system
        for(int i  = 0; i < _wheelCount; i++)
        {
            _driveMotor[i] = new MotorController(MotorControllerType.TalonFX, WheelsData[i].DriveChannel, true);
            _directionMotor[i] = new MotorController(MotorControllerType.TalonSRX , WheelsData[i].DirectionChannel, false);
            _directionEncoder[i] = new AnalogInput(WheelsData[i].DirectionEncoderChannel);
            _shifterValve[i] = new Solenoid(WheelsData[i].ShifterChannel);
            _shifterValve[i].set(false);

            _rotationVector[i] = WheelsData[i].GetWheelRotationVector();
            _wheelPosition[i] = WheelsData[i].WheelPosition;
            _angleOffset[i] = WheelsData[i].AngleOffset;

            _flipAngleOffset[i] = 0;
            _flipDriveMultiplicator[i] = 1;

            _directionPID[i] = new PID();
        }
    }

    public enum DrivingMode
    {
        Point,
        Local,
        World,
        Tank
    }
    public enum ShifterMode
    {
        Automatic,
        Manual
    }

    private int _wheelCount;

    private MotorController[] _driveMotor;
    private MotorController[] _directionMotor;
    private AnalogInput[] _directionEncoder;
    private Solenoid[] _shifterValve;
    private Vector2d[] _rotationVector;

    private Vector2d[] _wheelPosition;

    private ADIS16448_IMU _IMU;

    private double[] _angleOffset;

    private double[] _flipAngleOffset;
    private double[] _flipDriveMultiplicator;

    private PID[] _directionPID;

    private DrivingMode _mode = DrivingMode.Local;
    private ShifterMode _shiftMode = ShifterMode.Manual;
    private double _speedRatio = 0.5;
    private double _roationSpeedRatio = 0.5;

    private double _pointExponent = 0;
    private double _pointDistance = 0;

    private double _headingOffset;

    private RobotPosition _position = new RobotPosition(new Vector2d(0, 0));

    private RateLimiter _horizontalRateLimiter;
    private RateLimiter _verticaRateLimiter;
    private RateLimiter _rotationRateLimiter;

    private boolean _shiftState = false;

    private double _minShiftTime;
    private double _lastAutomaticShiftTime;

    private double _downshiftThreshold;
    private double _upshiftThreshold;

    private boolean _isShiftOverriden = false;
    private boolean _overridenShiftState = false;

    private double _rotationAxisOverride = 0;
    private boolean _isRotationAxisOverriden = false;

    private double _horizontalAxisOverride = 0;
    private boolean _isHorizontalAxisOverride = false;

    private double _verticalAxisOverride = 0;
    private boolean _isVerticalAxisOverride = false;

    private double _instantHorizontal = 0;
    private double _instantVertical = 0;
    private double _instantRotation = 0;

    public void Init()
    {
        
    }

    public void SetCurrentMode(DrivingMode Mode)
    {
        _mode = Mode;
    }
    public void SetCurrentMode(int Mode)
    {
        switch(Mode)
        {
            case 0:
            _mode = DrivingMode.Point;
            break;

            case 1:
            _mode = DrivingMode.Local;
            break;

            case 2:
            _mode = DrivingMode.World;
            break;

            case 3:
            _mode = DrivingMode.Tank;
            break;
        }
    }   
    public void SetPointDriveExponent(double Exponent)
    {
        _pointExponent = Exponent;
    }
    public void SetPointDriveDistance(double Distance)
    {
        _pointDistance = Distance;
    }

    public void SetShifterMode(ShifterMode Mode)
    {
        _shiftMode = Mode;
    }
    public void SetShifterMode(int Mode)
    {
        switch(Mode)
        {
            case 0:
            _shiftMode = ShifterMode.Automatic;
            break;

            case 1:
            _shiftMode = ShifterMode.Manual;
            break;
        }
    }
    public void SetShiftMinTime(double Time)
    {
        _minShiftTime = Time;
    }
    public void SetShiftThreshold(double Downshift, double Upshift)
    {
        _downshiftThreshold = Downshift;
        _upshiftThreshold = Upshift;
    }
    
    public void InitIMU()
    {
        _IMU = new ADIS16448_IMU();

        RecalibrateIMU();
    }
    public void RecalibrateIMU()
    {
        _IMU.reset();
        _IMU.calibrate();

        _headingOffset = (_IMU.getGyroAngleZ() / 180) * 3.1415 + 3.1415;
    }

    public void SetSpeed(double Speed)
    {
        _speedRatio = Speed;
    }
    public void SetPIDGain(int ID, double Kp, double Ki, double Kd)
    {
        _directionPID[ID].SetGain(Kp, Ki, Kd);
        _directionPID[ID].SetDebugMode("Swerve");
    }

    public boolean GetGear()
    {
        return _shiftState;
    }

    public void SetRateLimiter(double MaxSpeed)
    {
        _horizontalRateLimiter.SetVelocity(MaxSpeed);
        _verticaRateLimiter.SetVelocity(MaxSpeed);
    }
    public void SetRotationRateLimiter(double MaxSpeed)
    {
        _rotationRateLimiter.SetVelocity(MaxSpeed);
    }

    public void OverrideShift(int Speed)
    {
        if(Speed == 0)
        {
            _overridenShiftState = true;
            _isShiftOverriden = true;
        }
        else if (Speed == 1)
        {
            _overridenShiftState = false;
            _isShiftOverriden = true;
        }
    }
    public void OverrideRotationAxis(double AxisValue)
    {
        _rotationAxisOverride = Mathf.Clamp(AxisValue, -1, 1);
        _isRotationAxisOverriden = true;
    }
    public void OverrideHorizontalAxis(double AxisValue)
    {
        _horizontalAxisOverride = Mathf.Clamp(AxisValue, -1, 1);
        _isHorizontalAxisOverride = true;
    }
    public void OverrideVerticalAxis(double AxisValue)
    {
        _verticalAxisOverride = Mathf.Clamp(AxisValue, -1, 1);
        _isVerticalAxisOverride = true;
    }

    public double GetHeading()
    {
        return (_IMU.getGyroAngleZ() / 180) * 3.1415 - _headingOffset;
    }
    public Vector2d GetPostion()
    {
        return _position.GetPosition();
    }

    public void SetPosition(Vector2d Position)
    {
        _position.SetPosition(Position);
    }

    public Vector2d GetWheelVector(int ID)
    {
        double Angle = ((_directionEncoder[ID].getValue() / 4096f) * 2 * 3.1415f) - _angleOffset[ID] - 3.1415;

        if(Angle > 3.1415)
        {
            Angle -= 2 * 3.1415;
        }
        if(Angle < -3.1415)
        {
            Angle += 2 * 3.1415;
        }

        Vector2d vec = new Polar((_driveMotor[ID].GetEncoderVelocity() / 256.) * 3.1415 * 2, Angle).vector();

        return vec;
    }
    public int GetWheelCount()
    {
        return _wheelCount;
    }

    public double GetInstantHorizontalAxis()
    {
        return _instantHorizontal;
    }
    public double GetInstanVerticalAxis()
    {
        return _instantVertical;
    }
    public double GetInstantRotationAxis()
    {
        return _instantRotation;
    }

    int f = 0;
    public void DoSystem()
    {/*
        System.out.println("(0): " + ((_directionEncoder[0].getValue() / 4096f) * 2 * 3.1415f));
        System.out.println("(1): " + ((_directionEncoder[1].getValue() / 4096f) * 2 * 3.1415f));

        System.out.println("(2): " + ((_directionEncoder[2].getValue() / 4096f) * 2 * 3.1415f));
        System.out.println("(3): " + ((_directionEncoder[3].getValue() / 4096f) * 2 * 3.1415f));
*/
        double dt = Timer.GetDeltaTime();

        //Override the shift state of the robot for a peculiar task
        if(_isShiftOverriden)
        {
            if(_shiftState != _overridenShiftState)
            {
                for(int i = 0; i < _wheelCount; i++)
                {
                    _shifterValve[i].set(_overridenShiftState);   
                }

                _shiftState = _overridenShiftState;
            }
        }
        else
        {
            switch(_shiftMode)
            {
                case Automatic:
                //Autoshift only if the delta time is reach and the robot velocity reach the threshold
                if(Timer.GetCurrentTime() - _lastAutomaticShiftTime >= _minShiftTime)
                {
                    Vector2d velocityVector = new Vector2d(0, 0);

                    for(int i = 0; i < _wheelCount; i++)
                    {
                        velocityVector = Mathf.Vector2Sum(velocityVector, GetWheelVector(i));
                    }
                    double Mag = velocityVector.magnitude();

                    if(_shiftState && Mag<= _downshiftThreshold)
                    {
                        _lastAutomaticShiftTime = Timer.GetCurrentTime();
                        _shiftState = false;

                        for(int i = 0; i < _shifterValve.length; i++)
                        {
                            _shifterValve[i].set(_shiftState);
                        }
                    }
                    else if (!_shiftState && Mag>= _upshiftThreshold)
                    {
                        _lastAutomaticShiftTime = Timer.GetCurrentTime();
                        _shiftState = true;

                        for(int i = 0; i < _shifterValve.length; i++)
                        {
                            _shifterValve[i].set(_shiftState);
                        }

                        _horizontalRateLimiter.SetCurrent(_horizontalRateLimiter.GetCurrent() * 0.4);
                        _verticaRateLimiter.SetCurrent(_verticaRateLimiter.GetCurrent() * 0.4);
                        _rotationRateLimiter.SetCurrent(_rotationRateLimiter.GetCurrent() * 0.4);
                    }
                }

                SmartDashboard.putBoolean("Gear", _shiftState);
                break;

                case Manual:
                
                if(InputManager.GetButtonDown("GearShift"))
                {
                    _shiftState = !_shiftState;

                    for(int i = 0; i < _shifterValve.length; i++)
                    {
                        _shifterValve[i].set(_shiftState);
                    }
                }

                SmartDashboard.putBoolean("Gear", _shiftState);

                break;
            }
        }

        _isShiftOverriden = false;

        switch(_mode)
        {
            case Local:
            case World:

            //Adding a rate limiter to the translation joystick to make the driving smoother
            double horizontal = Input.GetAxis("Horizontal");
            double vertical = Input.GetAxis("Vertical");
            double rotation = Input.GetAxis("Rotation");

            double x = _horizontalRateLimiter.Evaluate(horizontal);
            double y = _verticaRateLimiter.Evaluate(vertical);
            double z = _rotationRateLimiter.Evaluate(rotation);

            double mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));

            if(mag > 1)
            {
                horizontal = _horizontalRateLimiter.GetCurrent() / mag;
                vertical = _verticaRateLimiter.GetCurrent() / mag;
                rotation =  _rotationRateLimiter.GetCurrent() / mag;
            }

            //Translation vector is equal to the translation joystick axis
            Vector2d translation = new Vector2d(_isHorizontalAxisOverride ? _horizontalAxisOverride : _horizontalRateLimiter.GetCurrent() * _speedRatio * -1, (_isVerticalAxisOverride ? _verticalAxisOverride : _verticaRateLimiter.GetCurrent()) * _speedRatio);
            Polar translationPolar = Polar.fromVector(translation);

            //Remove the angle of the gyroscope to the azymuth to make the driving relative to the world
            translationPolar.azymuth -= _mode == DrivingMode.World ? (_IMU.getGyroAngleZ() % 360) * 0.01745 + 3.1415: 0;

            double rotationAxis = _isRotationAxisOverriden ? _rotationAxisOverride : _rotationRateLimiter.GetCurrent() * _roationSpeedRatio;

            for(int i = 0; i < _wheelCount; i++)
            {
                //Each wheel have a predetermined rotation vector based on wheel position
                Vector2d scaledRotationVector = new Vector2d(_rotationVector[i].x * rotationAxis * _roationSpeedRatio, _rotationVector[i].y * rotationAxis * _roationSpeedRatio);               

                Vector2d SumVec = Mathf.Vector2Sum(scaledRotationVector, translationPolar.vector());
                Polar Sum = Polar.fromVector(SumVec);

                //Radius = Wheel Speed
                //Azymuth = Wheel Heading

                double wheelSpeed = Sum.radius;

                _driveMotor[i].Set(Mathf.Clamp(wheelSpeed, -1, 1) * _flipDriveMultiplicator[i]);

                double angle = Mathf.Clamp(_directionPID[i].Evaluate(GetDeltaAngle(i, Sum.vector()), dt), -1, 1);
                if(Math.abs(angle) <= 1 * Mathf.DEG_2_RAD)
                {
                    angle = 0;
                }

                _directionMotor[i].Set(angle);
            }

            f++;
            break;

            case Point:
            Vector2d point = GetPoint(Input.GetAxis("Horizontal"), Input.GetAxis("Vertical"));

            Polar[] wheelPol = new Polar[_wheelCount];

            double average = 0;
            for(int i = 0; i < _wheelCount; i++)
            {
                wheelPol[i] = Polar.fromVector(Mathf.Vector2Sub(point, _wheelPosition[i]));
                average += wheelPol[i].radius;
            }
            average /= (double)_wheelCount;

            for(int i = 0; i < _wheelCount; i++)
            {
                wheelPol[i].radius /= average;
                wheelPol[i].radius *=  Input.GetAxis("Rotation");

                _driveMotor[i].Set(Mathf.Clamp(wheelPol[i].radius, -1, 1) * _flipDriveMultiplicator[i]);
                _directionMotor[i].Set(Mathf.Clamp(_directionPID[i].Evaluate(GetDeltaAngle(i, wheelPol[i].vector()), dt), -1, 1));
            }
            break;

            case Tank:
            for(int i = 0; i < _wheelCount; i++)
            {
                //Always Allign Wheel Forward
                _directionMotor[i].Set(Mathf.Clamp(_directionPID[i].Evaluate(GetDeltaAngle(i, new Vector2d(0, 1)), dt), -1, 1));

                if(i + 1 <= _wheelCount / 2)
                {
                    //Right
                }
                else
                {
                    //Left
                }
            }
            break;
        }

        //Evaluate the robot position from the accelerometers
        _position.Evaluate(Mathf.RotatePoint(new Vector2d(_IMU.getAccelInstantX(), _IMU.getAccelInstantY()), GetHeading()), Timer.GetDeltaTime());

        _instantHorizontal = _isHorizontalAxisOverride ? _horizontalAxisOverride : _horizontalRateLimiter.GetCurrent();
        _instantVertical = _isVerticalAxisOverride ? _verticalAxisOverride : _verticaRateLimiter.GetCurrent();
        _instantRotation = _isRotationAxisOverriden ? _rotationAxisOverride : _rotationRateLimiter.GetCurrent();

        //Reset the overriden state to false a the end of the "frame"
        _isRotationAxisOverriden = false;
        _isVerticalAxisOverride = false;
        _isHorizontalAxisOverride = false;
    }

    private double GetDeltaAngle(int ID, Vector2d Target)
    {
        double Source = ((_directionEncoder[ID].getValue() / 4096f) * 2 * 3.1415f) - _angleOffset[ID] - 3.1415;

        if(Source > 3.1415)
        {
            Source -= 2 * 3.1415;
        }
        if(Source < -3.1415)
        {
            Source += 2 * 3.1415;
        }

        double xPrim = Target.x * Math.cos(Source) - Target.y * Math.sin(Source); //Change of coordinate system
        double yPrim = Target.x * Math.sin(Source) + Target.y * Math.cos(Source);

        double angle = Math.atan2(yPrim * _flipDriveMultiplicator[ID], xPrim * _flipDriveMultiplicator[ID]); //Angle betwen Source and target

        if(Math.abs(angle) > (3.1415 / 2)) //Check if it's faster to just flip the drive motor instead of doing 180° turn
        {
            _flipDriveMultiplicator[ID] *= -1;
        }

        return angle;
    }
    private Vector2d GetPoint(double xAxis, double yAxis)
    {
        return new Vector2d(Math.pow(xAxis, _pointExponent) * _pointDistance, Math.pow(yAxis, _pointExponent) * _pointDistance);
    }
}