#ifndef DISTACE_H
#define DISTACE_H


#include "stdio.h"
#include "math.h"

class Vec2
{
public:
    float _x;
    float _y;

    Vec2()
    {
        _x = 0;
        _y = 0;
    }

    Vec2( const float x, const float y )
    {
        _x = x;
        _y = y;
    }

    Vec2 operator+( const Vec2 &v ) const
    {
        return Vec2( this->_x + v._x, this->_y + v._y );
    }

    Vec2 operator-( const Vec2 &v ) const
    {
        return Vec2( this->_x - v._x, this->_y - v._y );
    }

    Vec2 operator*( const float f ) const
    {
        return Vec2( this->_x * f, this->_y * f );
    }

    float DistanceToSquared( const Vec2 p ) const
    {
        const float dX = p._x - this->_x;
        const float dY = p._y - this->_y;

        return dX * dX + dY * dY;
    }

    float DistanceTo( const Vec2 p ) const
    {
        return sqrt( this->DistanceToSquared( p ) );
    }

    float DotProduct( const Vec2 p ) const
    {
        return this->_x * p._x + this->_y * p._y;
    }
};


//void TestDistanceFromLineSegmentToPoint( float segmentX1, float segmentY1, float segmentX2, float segmentY2, float pX, float pY )
//{
//    float qX;
//    float qY;
//    float d = DistanceFromLineSegmentToPoint( segmentX1, segmentY1, segmentX2, segmentY2, pX, pY, &qX, &qY );
//    printf( "line segment = ( ( %f, %f ), ( %f, %f ) ), p = ( %f, %f ), distance = %f, q = ( %f, %f )\n",
//            segmentX1, segmentY1, segmentX2, segmentY2, pX, pY, d, qX, qY );
//}
//
//void TestDistanceFromLineSegmentToPoint()
//{
//    TestDistanceFromLineSegmentToPoint( 0, 0, 1, 1, 1, 0 );
//    TestDistanceFromLineSegmentToPoint( 0, 0, 20, 10, 5, 4 );
//    TestDistanceFromLineSegmentToPoint( 0, 0, 20, 10, 30, 15 );
//    TestDistanceFromLineSegmentToPoint( 0, 0, 20, 10, -30, 15 );
//    TestDistanceFromLineSegmentToPoint( 0, 0, 10, 0, 5, 1 );
//    TestDistanceFromLineSegmentToPoint( 0, 0, 0, 10, 1, 5 );
//}


#endif // DISTACE_H
