/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Math;

import java.util.function.Function;

import edu.wpi.first.wpilibj.drive.Vector2d;

public class Curve 
{
    public Curve(Vector2d... Points)
    {
        _points = Points.clone();
    }
    public Curve(int res, double min, double max, Function<Double, Double> Equation)
    {
        _points = new Vector2d[res];

        for(int i = 0; i < res; i++)
        {
            double x = Mathf.Lerp(min, max, i / (double)res);
            double y = Equation.apply(x);

            _points[i] = new Vector2d(x, y);
        }
    }

    private Vector2d[] _points;

    public double Evaluate(int x)
    {
        if(_points.length == 0)
        {
            return 0;
        }

        if(x <= _points[0].x)
        {
            return _points[0].y;
        }

        for(int i = 0; i < _points.length - 1; i++)
        {
            if(_points[i].x <= x)
            {
                return Mathf.Lerp(_points[i], _points[i + 1], x).y;
            }
        }

        return _points[_points.length - 1].y;
    }
}
