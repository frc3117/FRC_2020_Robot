/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Component;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C.Port;

public class ColorSensor 
{
    public ColorSensor(int HalfTurnCount)
    {
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

    public ColorSensor(int HalfTurnCount, Color Red, Color Green, Color Blue, Color Yellow)
    {
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

    public boolean IsTurnDone()
    {
        if(_watchColorCount >= _halftTurnCount)
        {
            StopRegisterTurn();
            return true;
        }
        return false;
    }

    public String GetColor()
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

        if(_isWatchingTurn)
            watchTurn(color);

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
}
