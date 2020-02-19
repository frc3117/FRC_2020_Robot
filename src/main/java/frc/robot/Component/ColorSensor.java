package frc.robot.Component;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;

public class ColorSensor 
{
    public ColorSensor(int TalonChannel ,int HalfTurnCount)
    {
        _motor = new TalonSRX(TalonChannel);

        _halftTurnCount = HalfTurnCount;

        redColor = ColorMatch.makeColor(0.561, 0.232, 0.114);
        greenColor = ColorMatch.makeColor(0.197, 0.561, 0.240);
        blueColor = ColorMatch.makeColor(0.143, 0.427, 0.429);
        yellowColor = ColorMatch.makeColor(0.361, 0.524, 0.113);

        //Initializing the color sensor match 
        colorMatch.addColorMatch(redColor);
        colorMatch.addColorMatch(greenColor);
        colorMatch.addColorMatch(blueColor);
        colorMatch.addColorMatch(yellowColor);  
    }

    public ColorSensor(int TalonChannel, int HalfTurnCount, Color Red, Color Green, Color Blue, Color Yellow)
    {
        _motor = new TalonSRX(TalonChannel);

        _halftTurnCount = HalfTurnCount;

        redColor = Red;
        greenColor = Green;
        blueColor = Blue;
        yellowColor = Yellow;

        //Initializing the color sensor match 
        colorMatch.addColorMatch(redColor);
        colorMatch.addColorMatch(greenColor);
        colorMatch.addColorMatch(blueColor);
        colorMatch.addColorMatch(yellowColor); 
    }

    private TalonSRX _motor;
    private double _motorSpeed = 0.3;

    private final ColorSensorV3 colorSensorV3 = new ColorSensorV3(Port.kOnboard);
    private ColorMatch colorMatch = new ColorMatch();
  
    private Color redColor;
    private Color greenColor;
    private Color blueColor;
    private Color yellowColor;

    private int _halftTurnCount;

    private boolean _isWatchingTurn = false;
    private String _watchColor = "none";
    private int _watchColorCount = 0;
    private String _lastColor = "";

    private boolean _isAllignColor = false;
    private String _requiredColor = "";

    public void Init()
    {
        _isWatchingTurn = false;
        _watchColor = "none";
        _watchColorCount = 0;
        _lastColor = "";

        _isAllignColor = false;
        _requiredColor = "";
    }

    public void SetMotorSpeed(double Speed)
    {
        _motorSpeed = Speed;
    }
    public void SetConfidence(double Confidence)
    {
        colorMatch.setConfidenceThreshold(Confidence);
    }

    public void StartRegisterTurn()
    {
        _isWatchingTurn = true;

        _watchColor = GetColor();
        _watchColorCount = 0;
    }
    public void StopRegisterTurn()
    {
        _isWatchingTurn = false;
    }
    
    public void StartAllignOnColor()
    {
        if(_requiredColor == "")
            CheckRequiredColor();

        _isAllignColor = true;
    }
    public void StopAllignOnColor()
    {
        _isAllignColor = false;
    }

    public boolean IsTurnDone()
    {
        if(_watchColorCount >= _halftTurnCount)
        {
            StopRegisterTurn();
            return true;
        }
        return false;
    }

    private boolean CheckRequiredColor()
    {
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        
        if(_requiredColor != "")
        {
            return true;
        }

        if(gameData.length() > 0)
        {
            switch (gameData.charAt(0))
            {
                case 'B' :
                _requiredColor = "blue";
                break;

                case 'G' :
                _requiredColor = "green";
                break;

                case 'R' :
                _requiredColor = "red";
                break;

                case 'Y' :
                _requiredColor = "yellow";
                break;

                default :
                return false;
            }
        }
        else
        {
            return false;
        }

        return true;
    }

    private String GetColor()
    {
        ColorMatchResult result = colorMatch.matchClosestColor(colorSensorV3.getColor());

        String color = "";

        if(result.color == redColor)
        {
            color = "red";
        }
        else if (result.color == greenColor)
        {
            color = "green";
        }
        else if (result.color == blueColor)
        {
            color = "blue";
        }
        else if (result.color == yellowColor)
        {
            color = "yellow";
        }
        else
        {
            color = "none";
        }

        return color;
    }

    private void watchTurn(String Color)
    {
        if(Color == "none")
            return;
        else if (Color != _lastColor && Color == _watchColor)
        {
            _watchColorCount++;
        }

        _lastColor = Color;
    }

    public void DoColorSensor()
    {
        String Color = GetColor();

        if(_isWatchingTurn)
        {
            watchTurn(Color);

            if(IsTurnDone())
            {
                _motor.set(ControlMode.PercentOutput, 0);
                StopRegisterTurn();
            }
            else
            {
                _motor.set(ControlMode.PercentOutput, _motorSpeed);
            }
        }
        else if (_isAllignColor)
        {
            if(Color == _requiredColor)
            {
                _motor.set(ControlMode.PercentOutput, 0);
                StopAllignOnColor();
            }
            else
            {
                _motor.set(ControlMode.PercentOutput, _motorSpeed);
            }
        }
    }
}
