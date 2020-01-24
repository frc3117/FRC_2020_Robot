package frc.robot.Math;

import edu.wpi.first.wpilibj.drive.Vector2d;

import java.lang.Math;

public class Mathf 
{
    public final static double DEG_2_RAD = 0.0174533;
    public final static double RAD_2_DEG = 57.2958;

    public final static double METER_2_FEET = 3.28084;
    public final static double FEET_2_METER = 0.3048;

    public static double Clamp(double value, double min, double max)
    {
        return Math.max(min, Math.min(max, value));
    }
    
    public static double Lerp(double v1, double v2, double t)
    {
        return (t * (v2 - v1)) + v1;
    }
    public static Vector2d Lerp(Vector2d p1, Vector2d p2, double x)
    {
        return new Vector2d(x, ((p2.y - p2.y) * ((x - p1.x) / (p2.x - p1.x))) + p1.y);
    }

    public static double DeltaAngle(double Source, double Target)
    {
        return DeltaAngle(new Polar(1, Source).vector(), new Polar(1, Target).vector());
    }
    public static double DeltaAngle(Vector2d Source, Vector2d Target)
    {
        double SourceAngle = Polar.fromVector(Source).azymuth;

        double xPrim = Target.x * Math.cos(SourceAngle) - Target.y * Math.sin(SourceAngle); //Change of coordinate system
        double yPrim = Target.x * Math.sin(SourceAngle) + Target.y * Math.cos(SourceAngle);

        double angle = Math.atan2(yPrim, xPrim); //Angle betwen Source and target

        return angle;
    }
    
    public static double GetAngle(Vector2d p1, Vector2d p2)
    {
        return Math.atan2(p2.y - p1.y, p2.x - p1.x);
    }

    public static Vector2d RotatePoint(Vector2d Vector, double angle)
    {
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);
        
        double tx = Vector.x;
        double ty = Vector.y;

        return new Vector2d((cos * tx) - (sin * ty), (sin * tx) + (cos * ty));
    }

    public static Vector2d Vector2Scale(Vector2d vec, double value)
    {
        vec.x *= value;
        vec.y *= value;

        return vec;
    }
    public static Vector2d Vector2Scale(double value, Vector2d vec)
    {
        return Vector2Scale(vec, value);
    }

    public static Vector2d Vector2Sum(Vector2d v1, Vector2d v2)
    {
        v1.x += v2.x;
        v1.y += v2.y;

        return v1;
    }
    public static Vector2d Vector2Sum(Vector2d v1, double val)
    {
        v1.x += val;
        v1.y += val;

        return v1;
    }
    public static Vector2d Vector2Sum(double val, Vector2d v1)
    {
        return Vector2Sum(v1, val);
    }

    public static Vector2d Vector2Sub(Vector2d v1, Vector2d v2)
    {
        v1.x -= v2.x;
        v1.y -= v2.y;

        return v1;
    }
    public static Vector2d Vector2Sub(Vector2d v1, double val)
    {
        v1.x -= val;
        v1.y -= val;

        return v1;
    }
    public static Vector2d Vector2Sub(double val, Vector2d v1)
    {
        v1.x = val - v1.x;
        v1.y = val - v1.y;

        return v1;
    }
}
