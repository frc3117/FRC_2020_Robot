/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Component;

import frc.robot.Component.Data.RobotPosition;
import frc.robot.Component.Data.WheelData;
import frc.robot.Math.Mathf;
import frc.robot.Math.PID;
import frc.robot.Math.Polar;
import frc.robot.Math.Timer;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.Vector2d;

public class Swerve {
    public Swerve(WheelData[] WheelsData, Joystick Input)
    {
        _wheelCount = WheelsData.length;

        _driveMotor = new CANSparkMax[_wheelCount];
        _directionMotor = new TalonSRX[_wheelCount];
        _driveEncoder = new Encoder[_wheelCount];
        _directionEncoder = new AnalogInput[_wheelCount];
        _shifterValve = new Solenoid[_wheelCount];

        _rotationVector = new Vector2d[_wheelCount];
        _directionPID = new PID[_wheelCount];

        _angleOffset = new double[_wheelCount];

        _flipAngleOffset = new double[_wheelCount];
        _flipDriveMultiplicator = new double[_wheelCount];

        //Initializing all component of the swerve swerve system
        for(int i  = 0; i < _wheelCount; i++)
        {
            _driveMotor[i] = new CANSparkMax(WheelsData[i].DriveChannel, MotorType.kBrushless);
            _directionMotor[i] = new TalonSRX(WheelsData[i].DirectionChannel);
            _driveEncoder[i] = new Encoder(WheelsData[i].DriveEncoderA, WheelsData[i].DriveEncoderB);
            _directionEncoder[i] = new AnalogInput(WheelsData[i].DirectionEncoderChannel);
            _shifterValve[i] = new Solenoid(WheelsData[i].ShifterChannel);
            _shifterValve[i].set(false);

            _rotationVector[i] = WheelsData[i].GetWheelRotationVector();
            _angleOffset[i] = WheelsData[i].AngleOffset;

            _flipAngleOffset[i] = 0;
            _flipDriveMultiplicator[i] = 1;

            _directionPID[i] = new PID();
        }

        _input = Input;
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

    private CANSparkMax[] _driveMotor;
    private TalonSRX[] _directionMotor;
    private Encoder[] _driveEncoder;
    private AnalogInput[] _directionEncoder;
    private Solenoid[] _shifterValve;
    private Vector2d[] _rotationVector;
    private Joystick _input;

    private ADIS16448_IMU _IMU;

    private double[] _angleOffset;

    private double[] _flipAngleOffset;
    private double[] _flipDriveMultiplicator;

    private PID[] _directionPID;

    private double _deadzone = 0;
    private DrivingMode _mode = DrivingMode.Local;
    private ShifterMode _shiftMode = ShifterMode.Manual;
    private double _speedRatio = 0.5;
    private double _roationSpeedRatio = 0.5;

    private double _headingOffset;

    private RobotPosition _position = new RobotPosition(new Vector2d(0, 0));

    private boolean _shiftButtonLastState = false;
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

    public void SetDeadzone(double Deadzone)
    {
        _deadzone = Deadzone;
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

        _headingOffset = (_IMU.getGyroAngleZ() / 180) * 3.1415;
    }

    public void SetSpeed(double Speed)
    {
        _speedRatio = Speed;
    }
    public void SetPIDGain(int ID, double Kp, double Ki, double Kd)
    {
        _directionPID[ID].SetGain(Kp, Ki, Kd);
    }

    public boolean GetGear()
    {
        return _shiftState;
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

        Vector2d vec = new Polar((_driveEncoder[ID].getRate() / 256.) * 3.1415 * 2, Angle).vector();

        return vec;
    }
    public int GetWheelCount()
    {
        return _wheelCount;
    }

    int f = 0;
    public void DoSwerve()
    {
        double dt = Timer.GetDeltaTime();

        if(_isShiftOverriden)
        {
            if(_shiftState != _overridenShiftState)
            {
                for(int i = 0; i < _wheelCount; i++)
                {
                    _shifterValve[i].set(_shiftState);   
                }

                _shiftState = _overridenShiftState;
            }
        }
        else
        {
            switch(_shiftMode)
            {
                case Automatic:
                if(Timer.GetCurrentTime() - _lastAutomaticShiftTime >= _minShiftTime)
                {
                    if(_shiftState && 0 <= _downshiftThreshold)
                    {
                        _lastAutomaticShiftTime = Timer.GetCurrentTime();
                        _shiftState = false;

                        for(int i = 0; i < _shifterValve.length; i++)
                        {
                            _shifterValve[i].set(_shiftState);
                        }
                    }
                    else if (!_shiftState && 0 >= _upshiftThreshold)
                    {
                        _lastAutomaticShiftTime = Timer.GetCurrentTime();
                        _shiftState = true;

                        for(int i = 0; i < _shifterValve.length; i++)
                        {
                            _shifterValve[i].set(_shiftState);
                        }
                    }
                }
                //If (CurrentTime - LastTime) >= WaitTime && Velocity + AngularVelocity >= UpshiftThreshold
                //If (CurrentTime - LastTime) >= WaitTime && Velocity + AngularVelocity <= DownShiftThreshold
                break;

                case Manual:
                if(_shiftButtonLastState == false && _input.getRawButton(3))
                {
                    _shiftState = !_shiftState;

                    for(int i = 0; i < _shifterValve.length; i++)
                    {
                        _shifterValve[i].set(_shiftState);
                    }
                }

                _shiftButtonLastState = _input.getRawButton(3);
                break;
            }
        }

        _isShiftOverriden = false;

        switch(_mode)
        {
            case Local:
            case World:
            Vector2d translation = new Vector2d(_isHorizontalAxisOverride ? _horizontalAxisOverride : GetAxisDeadzone(0) * _speedRatio * -1, (_isVerticalAxisOverride ? _verticalAxisOverride : GetAxisDeadzone(1)) * _speedRatio);
            Polar translationPolar = Polar.fromVector(translation);
            translationPolar.azymuth -= _mode == DrivingMode.World ? (_IMU.getGyroAngleZ() % 360) * 0.01745 : 0;

            double rotationAxis = _isRotationAxisOverriden ? _rotationAxisOverride : GetAxisDeadzone(3) - GetAxisDeadzone(2);

            for(int i = 0; i < _wheelCount; i++)
            {
                //Each wheel have a predetermined rotation vector based on wheel position
                Vector2d scaledRotationVector = new Vector2d(_rotationVector[i].x * rotationAxis * _roationSpeedRatio, _rotationVector[i].y * rotationAxis * _roationSpeedRatio);               
                Polar rotationPolar = Polar.fromVector(scaledRotationVector);

                Polar Sum = translationPolar.add(rotationPolar); //Putting the translation and the rotation together

                double wheelSpeed = Sum.radius;

                _driveMotor[i].set(Mathf.Clamp(wheelSpeed, -1, 1) * _flipDriveMultiplicator[i]);
                _directionMotor[i].set(ControlMode.PercentOutput, Mathf.Clamp(_directionPID[i].Evaluate(GetDeltaAngle(i, Sum.vector()), dt), -1, 1));
            }

            f++;
            break;

            case Point:

            break;

            case Tank:
            for(int i = 0; i < _wheelCount; i++)
            {
                //Allwais Allign Wheel Forward
                _directionMotor[i].set(ControlMode.PercentOutput, Mathf.Clamp(_directionPID[i].Evaluate(GetDeltaAngle(i, new Vector2d(0, 1)), dt), -1, 1));

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

        _position.Evaluate(Mathf.RotatePoint(new Vector2d(_IMU.getAccelInstantX(), _IMU.getAccelInstantY()), GetHeading()), Timer.GetDeltaTime());

        _isRotationAxisOverriden = false;
        _isVerticalAxisOverride = false;
        _isHorizontalAxisOverride = false;
    }

    private double GetAxisDeadzone(int channel)
    {
        double axis = _input.getRawAxis(channel);

        if(Math.abs(axis) <= _deadzone)
        {
            return 0;
        }

        return axis;
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
}