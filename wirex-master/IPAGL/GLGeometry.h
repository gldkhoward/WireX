/*
* WireX  -  IPAGL
*
* Copyright (c) 2006-2019 Andreas Pott
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

/*! \file GLGeometry.h
 *
 *	\author   Andreas Pott
 *
 *  \par Summary
 *  This file contains a number of helper classes for algebra in 3-D space.
 *  Since we only deal with spatial vectors all allocations are fixed size
 *  allowing for a performant and lean memory management.
 *  The implementation is fully done in his header. Therefore, one can use
 *  this file without generating a static or dynamic libraray. Furthermore,
 *  the classes only depend on the standard header math.h.
 */

#ifndef CVECTOR3_H
#define CVECTOR3_H

#include <math.h>

//! some important constants for geometrical computations
const double GL_PI=3.14159265358979323846;
const double GL_DEG_TO_RAD=GL_PI/180.0;

/*! \class CVector3
 *  A three-dimensional vector with basic overloaded operations
 */
class CVector3
{
public:
    double  x , y , z ;              // the three components x,y,z of the vector

    //! Default constructor. Components of the vector are set to zero.
    CVector3() { x=y=z=0; }
    //! Copy constructor. Creates a vector and sets the value of its components equal to those of vector v.
    CVector3( const CVector3& v ) { x=v.x;y=v.y;z=v.z; }
    //! Create a vector with components a, b, c.
    CVector3( const double& a , const double& b , const double& c ) { x=a;y=b;z=c; }
	//! create a vector from the first three doubles in the array
	CVector3( const double* ptr) { x=ptr[0]; y=ptr[1]; z=ptr[2]; }

    //! Normalizes vector to unit length.
    void normalize(); 

	//! no index checking which index 1=x,2=y,3=z
    double& operator()( const int i )
    { return  *(&x+i-1) ; }

    //! Assigns a copy of the components of vector v to those of this vector.
    CVector3& operator= ( const CVector3& v )
        { x=v.x;y=v.y;z=v.z; return *this; }

	//! Adds to this vector the vector v.
	CVector3& operator+= ( const CVector3& v )
	    { x+=v.x;y+=v.y;z+=v.z;return *this; }

	//! Subtracts from this vector the vector v.
	CVector3& operator-= ( const CVector3& v )
		{ x-=v.x;y-=v.y;z-=v.z; return *this; }

	//! Multiplies this vector with scalar c.
    CVector3& operator*= ( const double& c )
        { x*=c;y*=c;z*=c; return *this; }

    //! Divides vector by scaler c.
    CVector3& operator/= ( const double& c )
	    { x/=c; y/=c; z/=c; return *this; };

    //!Overwrites this vector with the result of the vector product of this vector with vector v.
    CVector3& operator%= ( const CVector3& v )
    {
        double xc=x,yc=y;
        x=y *v.z-z *v.y;
        y=z *v.x-xc*v.z;
        z=xc*v.y-yc*v.x;
        return *this;
    }
};

//! Compute and returns the sum of the two vectors a and b.
inline CVector3 operator+( const CVector3& a , const CVector3& b )
{
  return CVector3 ( a.x+b.x , a.y+b.y , a.z+b.z ) ;
}

//! Compute and returns the difference of the two vectors a and b.
inline CVector3 operator-( const CVector3& a , const CVector3& b )
{
  return CVector3( a.x-b.x , a.y-b.y , a.z-b.z ) ;
}

//! Compute and returns the vector product of vectors a with vector b, where a % b represents a × b.
inline CVector3 operator%( const CVector3& a , const CVector3& b )
{
  return CVector3(a.y*b.z-a.z*b.y , a.z*b.x-a.x*b.z , a.x*b.y-a.y*b.x);
}

//! Compute and returns the vector resulting from the product of scalar x with vector a.
inline CVector3 operator*( const double& x , const CVector3& a )
{
  return CVector3( x*a.x , x*a.y , x*a.z ) ;
}

//! Compute and returns the vector resulting from the product of scalar x with vector a.
inline CVector3 operator*( const CVector3& a, const double& x )
{
  return CVector3( x*a.x , x*a.y , x*a.z ) ;
}

//! Devide each componant of a by x
inline CVector3 operator/( const CVector3& a, const double& x )
{
  return CVector3( a.x/x , a.y/x , a.z/x ) ;
}

//! Compute and returns a copy of vector a with negative coefficients.
inline CVector3 operator-( const CVector3& a )
{
  return CVector3( -a.x , -a.y , -a.z ) ;
}

//! Compute and returns the scalar product of the two vectors a and b.
inline double operator*( const CVector3& a , const CVector3& b )
{
  return ( a.x*b.x + a.y*b.y + a.z*b.z ) ;
}

//! Returns the squared length of vector v.
inline double squaredLength ( const CVector3& v )
{
  return ( v.x*v.x + v.y*v.y + v.z*v.z ) ;
}

//! Returns the length of vector v.
inline double length ( const CVector3& v )
{
  return sqrt( squaredLength(v) ) ;
}

inline void CVector3::normalize()
{ 
	double k=length(*this); x/=k; y/=k; z/=k; 
}

/*! inline version for basic math operations.
 *  the extended versions avoids to copy the returned value, because the result
 *  is passed as reference, too. the inline versions can be used in low-level 
 *  routines to optimize performance. add(a,b,c) works about 25% faster than a=b+c
 */

inline void mul(CVector3& des, const double& q)
{ des.x*=q; des.y*=q; des.z*=q; }

inline void mul(CVector3& des, const CVector3& src, const double& q)
{ des.x=src.x*q; des.y=src.y*q; des.z=src.z*q; }

inline void div(CVector3& des, const double& q)
{ des.x/=q; des.y/=q; des.z/=q; }

inline void div(CVector3& des, const CVector3& src, const double& q)
{ des.x=src.x/q; des.y=src.y/q; des.z=src.z/q; }

inline void add(CVector3& des, const CVector3& src)
{ des.x+=src.x; des.y+=src.y; des.z+=src.z; }

inline void add(CVector3& des, const CVector3& src1, const CVector3& src2)
{ des.x=src1.x+src2.x; des.y=src1.y+src2.y; des.z=src1.z+src2.z; }

inline void sub(CVector3& des, const CVector3& src)
{ des.x-=src.x; des.y-=src.y; des.z-=src.z; }

inline void sub(CVector3& des, const CVector3& src1, const CVector3& src2)
{ des.x=src1.x-src2.x; des.y=src1.y-src2.y; des.z=src1.z-src2.z; }

inline void cross(CVector3& des, const CVector3& a , const CVector3& b )
{ des.x=a.y*b.z-a.z*b.y; des.y= a.z*b.x-a.x*b.z; des.z=a.x*b.y-a.y*b.x; }


/*! \class CMatrix3
 *  A 3x3 matrix class with basic aritmetric operations
 */
class CMatrix3
{
public:
  CVector3  e1, e2, e3 ;        //!< Elements are stored columnwise

  //! Default constructor. Relaying on CVector3's behaviour this creates a null matrix
  CMatrix3() { }

  //! Copy constructor. Creates a matrix as a copy of the argument m.
  CMatrix3( const CMatrix3& m ) : e1(m.e1), e2(m.e2), e3(m.e3) {}

  //! Creates a matrix by prescribing its three columns vectors.
  CMatrix3( const CVector3& a, const CVector3& b, const CVector3& c )
   : e1(a), e2(b), e3(c) {}

  //! Creates a matrix by prescribing its nine coefficients in column order.
  CMatrix3( const double& a11, const double& a21, const double& a31,
            const double& a12, const double& a22, const double& a32,
            const double& a13, const double& a23, const double& a33 )
  {
    e1.x = a11 ; e1.y = a21 ; e1.z = a31 ;
    e2.x = a12 ; e2.y = a22 ; e2.z = a32 ;
    e3.x = a13 ; e3.y = a23 ; e3.z = a33 ;
  }

  //! Assigns a copy of matrix A to this matrix.
  CMatrix3& operator=( const CMatrix3& A )
  {
    e1=A.e1 ;
    e2=A.e2 ;
    e3=A.e3 ;
    return *this ;
  }

  //! return a column of the matrix; with index from 1 to 3 without index checking
  CVector3& operator()(int i) 
  { return *( &e1+i-1 ); }
  
  // extract component by index [1..3,1..3] without index checking
  double& operator()(int i, int j)     
  { return *( &e1.x+3*(j-1)+i-1 ) ; }

  CMatrix3 operator*( const CMatrix3& B) const ;
  CMatrix3& operator*=(const CMatrix3& B);

  CMatrix3 operator+( const CMatrix3& B_) const;
  CMatrix3& operator+=(const CMatrix3& B_);

  CMatrix3 operator-( const CMatrix3& B_) const;
  CMatrix3& operator-=(const CMatrix3& B_);
};

//! Creates and returns a matrix, product of the scalar c and the matrix A.
inline CMatrix3 operator*( const double& c , const CMatrix3& A )
{
    return CMatrix3(c*A.e1.x, c*A.e1.y, c*A.e1.z,
                    c*A.e2.x, c*A.e2.y, c*A.e2.z,
                    c*A.e3.x, c*A.e3.y, c*A.e3.z);

}

//!Creates and returns a copy of matrix A with negative coefficients.
inline CMatrix3 operator-( const CMatrix3& A )
{
    return CMatrix3( -A.e1.x, -A.e1.y, -A.e1.z,
                     -A.e2.x, -A.e2.y, -A.e2.z,
                     -A.e3.x, -A.e3.y, -A.e3.z);
}

inline CVector3 operator*( const CMatrix3& A , const CVector3& v )
{
    return CVector3(v.x * A.e1.x + v.y * A.e2.x + v.z * A.e3.x,
                    v.x * A.e1.y + v.y * A.e2.y + v.z * A.e3.y,
                    v.x * A.e1.z + v.y * A.e2.z + v.z * A.e3.z);
}

//! Creates and returns the vector product of matrix A with vector v.
inline CVector3 operator*( const CVector3& v , const CMatrix3& A )
{
    return CVector3(v.x * A.e1.x + v.y * A.e1.y + v.z * A.e1.z,
                    v.x * A.e2.x + v.y * A.e2.y + v.z * A.e2.z,
                    v.x * A.e3.x + v.y * A.e3.y + v.z * A.e3.z);
}

//! Create and return the transpose of matrix A.
inline CMatrix3 operator~( const CMatrix3& A )
{
  return CMatrix3(A.e1.x , A.e2.x , A.e3.x ,
                  A.e1.y , A.e2.y , A.e3.y ,
                  A.e1.z , A.e2.z , A.e3.z);
}

//! Creates and returns the anti-symmetric matrix corresponding to the vector-product operation by vector v.
inline CMatrix3 operator~( const CVector3& v )
{
  return CMatrix3(double(0) ,      v.z  ,    - v.y  ,
                     - v.z  , double(0) ,      v.x  ,
                       v.y  ,    - v.x  , double(0));
}

//! Creates and returns the matrix corresponding to the dyadic product of vectors a, b.
inline CMatrix3 operator^( const CVector3& a , const CVector3& b )
{ return CMatrix3(b.x * a , b.y * a , b.z * a) ; }

/*! inline version for basic math operations.
 *  this avoids to copy the returned value, because the result
 *  is passed by reference, too. the inline versions can be
 *  used in low-level routines to optimize performance. 
 *  when the result is a matrix the benefit of passing the result by
 *  reference is even greater than the routines for CVector3
 ****************************************************************/

inline void add(CMatrix3& des, const CMatrix3& src)
{ add(des.e1,src.e1); add(des.e2,src.e2); add(des.e3,src.e3); }

inline void sub(CMatrix3& des, const CMatrix3& src)
{ sub(des.e1,src.e1); sub(des.e2,src.e2); sub(des.e3,src.e3); }

inline void mul(CMatrix3& des, const double& q)
{ mul(des.e1,q); mul(des.e2,q);	mul(des.e3,q); }

inline void div(CMatrix3& des, const double& q)
{ div(des.e1,q); div(des.e2,q); div(des.e3,q); }

inline void add(CMatrix3& des, const CMatrix3& src1, const CMatrix3& src2)
{ add(des.e1,src1.e1,src2.e1); add(des.e2,src1.e2,src2.e2); add(des.e3,src1.e3,src2.e3); }

inline void sub(CMatrix3& des, const CMatrix3& src1, const CMatrix3& src2)
{ sub(des.e1,src1.e1,src2.e1); sub(des.e2,src1.e2,src2.e2); sub(des.e3,src1.e3,src2.e3); }

inline void mul(CMatrix3& des, const CMatrix3& src, const double& q)
{ mul(des.e1,src.e1,q); mul(des.e2,src.e2,q); mul(des.e3,src.e3,q); }

inline void div(CMatrix3& des, const CMatrix3& src, const double& q)
{ div(des.e1,src.e1,q); div(des.e2,src.e2,q); div(des.e3,src.e3,q); }

inline void mul(CMatrix3& des, const CVector3 &a, const CVector3 &b)
{ 
	des.e1.x=a.x*b.x; des.e2.x=a.x*b.y; des.e3.x=a.x*b.z; 
	des.e1.y=a.y*b.x; des.e2.y=a.y*b.y; des.e3.y=a.y*b.z; 
	des.e1.z=a.z*b.x; des.e2.z=a.z*b.y; des.e3.z=a.z*b.z;
}

inline void mul(CVector3& des, const CMatrix3 &A, const CVector3 &q)
{
	des.x=A.e1.x*q.x + A.e2.x*q.y + A.e3.x*q.z;
	des.y=A.e1.y*q.x + A.e2.y*q.y + A.e3.y*q.z;
	des.z=A.e1.z*q.x + A.e2.z*q.y + A.e3.z*q.z;
}

inline void mul(CVector3& des, const CVector3 &q, const CMatrix3 &A)
{
	des.x=A.e1.x*q.x + A.e1.y*q.y + A.e1.z*q.z;
	des.y=A.e2.x*q.x + A.e2.y*q.y + A.e2.z*q.z;
	des.z=A.e3.x*q.x + A.e3.y*q.y + A.e3.z*q.z;
}

inline void mul(CMatrix3& des, const CMatrix3 &A, const CMatrix3 &B)
{
	des.e1.x=A.e1.x*B.e1.x + A.e2.x*B.e1.y + A.e3.x*B.e1.z;
	des.e1.y=A.e1.y*B.e1.x + A.e2.y*B.e1.y + A.e3.y*B.e1.z;
	des.e1.z=A.e1.z*B.e1.x + A.e2.z*B.e1.y + A.e3.z*B.e1.z;
	des.e2.x=A.e1.x*B.e2.x + A.e2.x*B.e2.y + A.e3.x*B.e2.z;
	des.e2.y=A.e1.y*B.e2.x + A.e2.y*B.e2.y + A.e3.y*B.e2.z;
	des.e2.z=A.e1.z*B.e2.x + A.e2.z*B.e2.y + A.e3.z*B.e2.z;
	des.e3.x=A.e1.x*B.e3.x + A.e2.x*B.e3.y + A.e3.x*B.e3.z;
	des.e3.y=A.e1.y*B.e3.x + A.e2.y*B.e3.y + A.e3.y*B.e3.z;
	des.e3.z=A.e1.z*B.e3.x + A.e2.z*B.e3.y + A.e3.z*B.e3.z;
}

/*
//! Creates the elementary rotation matrizes around the x-axis
CMatrix3& getXRotationMatrix( const double& phi)
{
	double si = sin(phi);
	double co = cos(phi);
	return CMatrix3(1.0, 0.0 , 0.0,
					0.0,  co ,  si, 
					0.0, -si ,  co );
}

//! Creates the elementary rotation matrizes around the y-axis
CMatrix3& getYRotationMatrix( const double& phi)
{
	double si = sin(phi);
	double co = cos(phi);
	return CMatrix3( co, 0.0 , -si, 
					0.0, 1.0 , 0.0, 
					 si, 0.0 ,  co );
}

//! Creates the elementary rotation matrizes around the z-axis
CMatrix3& getZRotationMatrix( const double& phi)
{
	double si = sin(phi);
	double co = cos(phi);
	return CMatrix3(  co,  si, 0.0,
					 -si,  co, 0.0, 
					 0.0, 0.0, 1.0 );
}
*/

/*! \todo Implement Rodriguez formula (rotation around abitrary axis)
 *  \todo add conversion between quaternion and rotation matrix
 *  \todo add conventions for euler, bryant, roll-pitch-yaw angles
 */

/*! \class CFrame
 *  A spatial coordinate frame or a robot pose consisting of an orientation 
 *  matrix R and a vector r. One can also think of the coordinate frame as a 
 *  homogeous transformation matrix where the last row with [0 0 0 1] is 
 *  omitted.
 */
class CFrame
{
public:
	CMatrix3 R ;	//!< the orientation matrix of the frame
	CVector3 r ;	//!< the position of the origin of the frame decomposed in its own coordinates
	//! create an "unity" frame at origin
	CFrame() {
	  R.e1.x=R.e2.y=R.e3.z=1;
	}
	//! initialize with frame
	CFrame( const CFrame& K) : R(K.R), r(K.r) {}  
	//! build from vector/matrix
	CFrame( const CVector3& r_, const CMatrix3& R_) : r(r_), R(R_) {} 
};

#endif
