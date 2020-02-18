/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Math;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.wpilibj.drive.Vector2d;

public class Curve 
{
    public Curve(Vector2d... Points)
    {
        _points = new ArrayList<Vector2d>();
        Collections.addAll(_points, Points);
        Collections.sort(_points, new Vector2dSort());
    }

    private List<Vector2d> _points;

    public double Evaluate(double x)
    {
        if(_points.size() == 0)
        {
            return 0;
        }

        if(x <= _points.get(0).x)
        {
            return _points.get(0).y;
        }

        for(int i = 0; i < _points.size() - 1; i++)
        {
            if(_points.get(i).x >= x)
            {
                return Mathf.Lerp(_points.get(i), _points.get(i + 1), x).y;
            }
        }

        return _points.get(_points.size() - 1).y;
    }

    class Vector2dSort implements Comparator<Vector2d>
    {
        public int compare(Vector2d v1, Vector2d v2)
        {
            return (int)Math.signum(v1.x - v2.x);
        }
    }
}
