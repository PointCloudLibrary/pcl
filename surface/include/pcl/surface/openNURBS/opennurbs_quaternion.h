/* $NoKeywords: $ */
/*
//
// Copyright (c) 1993-2011 Robert McNeel & Associates. All rights reserved.
// OpenNURBS, Rhinoceros, and Rhino3D are registered trademarks of Robert
// McNeel & Associates.
//
// THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT EXPRESS OR IMPLIED WARRANTY.
// ALL IMPLIED WARRANTIES OF FITNESS FOR ANY PARTICULAR PURPOSE AND OF
// MERCHANTABILITY ARE HEREBY DISCLAIMED.
//				
// For complete openNURBS copyright information see <http://www.opennurbs.org>.
//
////////////////////////////////////////////////////////////////
*/

#if !defined(ON_QUATERNION_INC_)
#define ON_QUATERNION_INC_

class ON_CLASS ON_Quaternion
{
public:
  // quaternion = a + bi + cj + dk
  double a,b,c,d;

  static const ON_Quaternion Zero;     // 0   = (0,0,0,0
  static const ON_Quaternion Identity; // 1   = (1,0,0,0)
  static const ON_Quaternion I;        // "i" = (0,1,0,0)
  static const ON_Quaternion J;        // "j" = (0,0,1,0)
  static const ON_Quaternion K;        // "k" = (0,0,0,1)

  ON_Quaternion() {}

  ON_Quaternion(double qa, double qb, double qc, double qd);

  // (a,b,c,d) = (0,v.x,v.y,v.z)
  ON_Quaternion(const ON_3dVector& v);

  // (a,b,c,d) = (0,v.x,v.y,v.z)
  ON_Quaternion& operator=(const ON_3dVector& v);

  void Set(double qa, double qb, double qc, double qd);

  // arithmetic operators
  ON_Quaternion  operator*(int) const;
  ON_Quaternion  operator/(int) const;
  ON_Quaternion  operator*(float) const;
  ON_Quaternion  operator/(float) const;
  ON_Quaternion  operator*(double) const;
  ON_Quaternion  operator/(double) const;

  ON_Quaternion  operator+(const ON_Quaternion&) const;
  ON_Quaternion  operator-(const ON_Quaternion&) const;

  // quaternion multiplication is not commutative
  ON_Quaternion  operator*(const ON_Quaternion&) const;

  /*
  Returns:
    True if a, b, c, and d are valid finite IEEE doubles.
  */
  bool IsValid() const;

  /*
  Description:
    Returns the conjugate of the quaternion = (a,-b,-c,-d).
  */
  ON_Quaternion Conjugate() const;

  /*
  Description:
    Sets the quaternion to a/L2, -b/L2, -c/L2, -d/L2,
    where L2 = length squared = (a*a + b*b + c*c + d*d).
    This is the multiplicative inverse, i.e.,
    (a,b,c,d)*(a/L2, -b/L2, -c/L2, -d/L2) = (1,0,0,0).
  Returns:
    True if successful.  False if the quaternion is zero
    and cannot be inverted.
  */
  bool Invert();

  /*
  Returns:
    Sets the quaternion to a/L2, -b/L2, -c/L2, -d/L2,
    where L2 = length squared = (a*a + b*b + c*c + d*d).
    This is the multiplicative inverse, i.e.,
    (a,b,c,d)*(a/L2, -b/L2, -c/L2, -d/L2) = (1,0,0,0).
    If "this" is the zero quaternion, then the zero quaternion
    is returned.
  */
  ON_Quaternion Inverse() const;

  /*
  Returns:
    Returns the length or norm of the quaternion
    sqrt(a*a + b*b + c*c + d*d).
  */
  double Length() const;

  /*
  Returns:
    Returns a*a + b*b + c*c + d*d.
  */
  double LengthSquared() const;

  /*
  Returns:
    The distance or norm of the difference between the two quaternions.
    = ("this" - q).Length().
  */
  double DistanceTo(const ON_Quaternion& q) const;

  /*
  Returns:
    The distance or norm of the difference between the two quaternions.
    = (p - q).Length().
  */
  static double Distance(const ON_Quaternion& p, const ON_Quaternion& q);

  /*
  Returns:
    4x4 real valued matrix form of the quaternion

            a  b  c  d
           -b  a -d  c
           -c  d  a -b
           -d -c  b  a

    which has the same arithmetic properties in as the
    quaternion. 
  Remarks:
    Do not confuse this with the rotation defined
    by the quaternion. This function will only be interesting
    to math nerds and is not useful in rendering or animation
    applications.
  */
  ON_Xform MatrixForm() const;

  /*
  Description:
    Scales the quaternion's coordinates so that
    a*a + b*b + c*c + d*d = 1.
  Returns:
    True if successful.  False if the quaternion is zero
    and cannot be unitized.
  */
  bool Unitize();

  /*
  Description:
    Sets the quaternion to 

       cos(angle/2), sin(angle/2)*x, sin(angle/2)*y, sin(angle/2)*z

    where (x,y,z) is the unit vector parallel to axis.  This is
    the unit quaternion that represents the rotation of angle
    about axis.
  Parameters:
    angle - [in] in radians
    axis - [in] axis of rotation
  Returns:
  */
  void SetRotation(double angle, const ON_3dVector& axis);

  /*
  Parameters:
    angle - [in] in radians
    axis - [in] axis of rotation
  Returns:
    The unit quaternion 

       cos(angle/2), sin(angle/2)*x, sin(angle/2)*y, sin(angle/2)*z

    where (x,y,z) is the unit vector parallel to axis.  This is the
    unit quaternion that represents the rotation of angle about axis.
  */
  static ON_Quaternion Rotation(double angle, const ON_3dVector& axis);

  /*
  Descriptin:
    Sets the quaternion to the unit quaternion which rotates
    plane0.xaxis to plane1.xaxis,
    plane0.yaxis to plane1.yaxis, and 
    plane0.zaxis to plane1.zaxis.
  Parameters:
    plane0 - [in]
    plane1 - [in]
  Remarks:
    The plane origins are ignored.
  */
  void SetRotation(const ON_Plane& plane0, const ON_Plane& plane1);

  /*
  Parameters:
    plane0 - [in]
    plane1 - [in]
  Returns:
    The unit quaternion that represents the the rotation that maps
    plane0.xaxis to plane1.xaxis,
    plane0.yaxis to plane1.yaxis, and 
    plane0.zaxis to plane1.zaxis.
  Remarks:
    The plane origins are ignored.
  */
  static ON_Quaternion Rotation(const ON_Plane& plane0, const ON_Plane& plane1);

  /*
  Parameters:
    angle - [out]
      in radians
    axis - [out]
      unit axis of rotation of 0 if (b,c,d) is the zero vector.
  Returns:
    The rotation defined by the quaternion.
  Remarks:
    If the quaternion is not unitized, the rotation of its
    unitized form is returned.
  */
  bool GetRotation(double& angle, ON_3dVector& axis) const;

  /*
  Description:
    The transformation returned by this function has the property
    that xform*V = q.Rotate(V).
  Parameters:
    xform - [out]
  Returns:
    A transformation matrix that performs the rotation defined 
    by the quaternion.
  Remarks:
    If the quaternion is not unitized, the rotation of its
    unitized form is returned.  Do not confuse the result of this
    function the matrix returned by ON_Quaternion::MatrixForm().
    The transformation returned by this function has the property
    that xform*V = q.Rotate(V).
  */
  bool GetRotation(ON_Xform& xform) const;

  /*
  Parameters:
    plane - [out]
  Returns:
    The frame created by applying the quaternion's rotation
    to the canonical world frame (1,0,0),(0,1,0),(0,0,1).
  */
  bool GetRotation(ON_Plane& plane) const;

  /*
  Description
    Rotate a 3d vector.  This operation is also called
    conjugation, because the result is the same as
    
    (q.Conjugate()*(0,x,y,x)*q/q.LengthSquared()).Vector()

  Parameters:
    v - [in]
  Returns:
    R*v, where R is the rotation defined by the unit quaternion.
    This is mathematically the same as the values
      (Inverse(q)*(0,x,y,z)*q).Vector()
    and
      (q.Conjugate()*(0,x,y,x)*q/q.LengthSquared()).Vector()
  Remarks:
    If you need to rotate more than a dozen or so vectors, it will
    be more efficient to call GetRotation(ON_Xform& xform)
    and multiply the vectors by xform.
  */
  ON_3dVector Rotate(ON_3dVector v) const;

  /*
  Returns:
    The "vector" or "imaginary" part of the quaternion = (b,c,d)
  */
  ON_3dVector Vector() const;

  /*
  Returns:
    The "real" or "scalar" part of the quaternion = a.
  */
  double Scalar() const;

  /*
  Returns:
    True if a, b, c, and d are all zero.
  */
  bool IsZero() const;

  /*
  Returns:
    True if a, b, c, and d are all valid, finite and at least one is non-zero.
  */
  bool IsNotZero() const;

  /*
  Returns:
    True if b, c, and d are all zero.
  */
  bool IsScalar() const;

  /*
  Returns:
    True if a = 0 and at least one of b, c, or d is not zero.
  */
  bool IsVector() const; 


  /*
    Returns:
      exp(q) = e^a*( cos(|V|) + V/|V|*sin(|V|) ), where V = b*i + c*j + d*k.
  */
  static ON_Quaternion Exp(ON_Quaternion q);

  /*
    Returns:
      log(q) = log(|q|) + V/|V|*acos(a/|q|), where V = b*i + c*j + d*k.
  */
  static ON_Quaternion Log(ON_Quaternion q);

  /*
    Returns:
      q^t = Exp(t*Log(q))
  */
  static ON_Quaternion Pow(ON_Quaternion q, double t);


  static ON_Quaternion Slerp(ON_Quaternion q0, ON_Quaternion q1, double t);

};

/*
Returns:
  The quaternion product of p and q.  This is the same value as p*q.
*/
ON_DECL
ON_Quaternion ON_QuaternionProduct( const ON_Quaternion& p, const ON_Quaternion& q);

/*
Returns:
  The vector cross product of p and q = (0,x,y,z) where
  (x,y,z) = ON_CrossProduct(p.Vector(),q.Vector())

  This is NOT the same as the quaternion product p*q.
*/
ON_DECL
ON_Quaternion ON_CrossProduct( const ON_Quaternion& p, const ON_Quaternion& q);

ON_DECL
ON_Quaternion operator*(int, const ON_Quaternion&);

ON_DECL
ON_Quaternion operator*(float, const ON_Quaternion&);

ON_DECL
ON_Quaternion operator*(double, const ON_Quaternion&);

#endif
