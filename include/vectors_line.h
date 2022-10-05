#include <Arduino.h>
#pragma once
struct Vector2 
{
    public:
    Vector2(){};
    Vector2(double _x,double _y):x(_x),y(_y)
    {
        mag();        
    };
    Vector2(double _angle,double _mag,bool polar):angle_rad(_angle),magnitude(_mag)
    {
        resolvePolar(angle_rad,magnitude);        
    };
    double x;
    double y;
    double magnitude;
    //current internal angle where pi/4 is straight
    double angle_rad;
    //internally set the x and y
    void resolvePolar()
    {
        x = cos(angle_rad)*magnitude;
        y = sin(angle_rad)*magnitude;
    }
    //sets this vector as a new vector form polar coords
    void resolvePolar(double _angle,double _magnitude)
    {
        x = cos(_angle)*_magnitude;
        y = sin(_angle)*_magnitude;
        angle_rad = _angle;
        magnitude = _magnitude;
    }
    // returns the |angle| in radians between two vectors

    double angle(Vector2 const &lhs,Vector2 const &rhs)
    {
        //l.r = |l||r|cos(a)
        return acos(Vector2::dot(lhs,rhs)/(lhs.magnitude*rhs.magnitude));
    }
    Vector2 normalized()
    {
        if(magnitude<=0)
        {
            return Vector2(0,0);
        }
        return Vector2(x/magnitude,y/magnitude);
    }
    
    Vector2 operator + (Vector2 const &obj)
    {
        Vector2 _vec2;
        _vec2.x = x+obj.x;
        _vec2.y = y+obj.y;
        _vec2.mag();
        return _vec2;
    }
    Vector2 operator - (Vector2 const &obj)
    {
        Vector2 _vec2;
        _vec2.x = x-obj.x;
        _vec2.y = y-obj.y;
        _vec2.mag();
        return _vec2;
    }

    //returns the dot product between two vectors in rads
    static double dot(Vector2 const &lhs, Vector2 const &rhs)
    {
        return ((lhs.x*rhs.x) + (lhs.y*rhs.y));
    }
    
// cx = aybz − azby
// cy = azbx − axbz
// cz = axby − aybx
// returns the signed angle in radians between two vectors (RH rule)
    static double angle_signed(Vector2 const &lhs, Vector2 const &rhs)
    {
        // Vector2 lhs_n = lhs.normalized();
        // Vector2 rhs_n = rhs.normalized();
        return asin((lhs.x*rhs.y)-(lhs.y*rhs.x)/(lhs.magnitude*rhs.magnitude));
    }

    private:
    //Sets the internal angle anticlockwise from (1,0,0)
    void angle()
    {
        angle_rad = atan(y/x);
    }
    void mag()
    {
        magnitude = sqrt(pow(x,2)+pow(y,2));
    }
};

template <typename T>
Vector2 vector2(T a,T b)
{
    return Vector2((double) a,(double) b);
}