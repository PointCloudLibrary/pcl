/* $NoKeywords: $ */
/*
//
// Copyright (c) 1993-2012 Robert McNeel & Associates. All rights reserved.
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

////////////////////////////////////////////////////////////////
//
//   defines float precision point, vector, and array classes
//
////////////////////////////////////////////////////////////////
#if !defined(ON_FPOINT_INC_)
#define ON_FPOINT_INC_

class ON_Xform;

class ON_2fPoint;
class ON_3fPoint;
class ON_4fPoint;

class ON_2fVector;
class ON_3fVector;

////////////////////////////////////////////////////////////////
//
//   ON_2fPoint
//
class ON_CLASS ON_2fPoint
{
public:
  float x, y;

  static const ON_2fPoint Origin; // (0.0f,0.0f)

  // use implicit destructor, copy constructor
  ON_2fPoint();                       // x,y not initialized
  ON_2fPoint(float x,float y);

  ON_2fPoint(const ON_3fPoint& );     // from 3f point
  ON_2fPoint(const ON_4fPoint& );     // from 4f point
  ON_2fPoint(const ON_2fVector& );    // from 2f vector
  ON_2fPoint(const ON_3fVector& );    // from 3f vector
  ON_2fPoint(const float*);           // from float[2] array

  ON_2fPoint(const ON_2dPoint& );     // from 2d point
  ON_2fPoint(const ON_3dPoint& );     // from 3d point
  ON_2fPoint(const ON_4dPoint& );     // from 4d point
  ON_2fPoint(const ON_2dVector& );    // from 2d vector
  ON_2fPoint(const ON_3dVector& );    // from 3d vector
  ON_2fPoint(const double*);          // from double[2] array

  // (float*) conversion operators
  operator float*();
  operator const float*() const;

  // use implicit operator=(const ON_2fPoint&)
  ON_2fPoint& operator=(const ON_3fPoint&);
  ON_2fPoint& operator=(const ON_4fPoint&);
  ON_2fPoint& operator=(const ON_2fVector&);
  ON_2fPoint& operator=(const ON_3fVector&);
  ON_2fPoint& operator=(const float*);  // point = float[2] support

  ON_2fPoint& operator=(const ON_2dPoint&);
  ON_2fPoint& operator=(const ON_3dPoint&);
  ON_2fPoint& operator=(const ON_4dPoint&);
  ON_2fPoint& operator=(const ON_2dVector&);
  ON_2fPoint& operator=(const ON_3dVector&);
  ON_2fPoint& operator=(const double*); // point = double[2] support

  ON_2fPoint& operator*=(float);
  ON_2fPoint& operator/=(float);
  ON_2fPoint& operator+=(const ON_2fPoint&);
  ON_2fPoint& operator+=(const ON_2fVector&);
  ON_2fPoint& operator+=(const ON_3fVector&);
  ON_2fPoint& operator-=(const ON_2fPoint&);
  ON_2fPoint& operator-=(const ON_2fVector&);
  ON_2fPoint& operator-=(const ON_3fVector&);

  ON_2fPoint  operator*(int) const;
  ON_2fPoint  operator/(int) const;
  ON_2fPoint  operator*(float) const;
  ON_2fPoint  operator/(float) const;
  ON_2dPoint  operator*(double) const;
  ON_2dPoint  operator/(double) const;

  ON_2fPoint  operator+(const ON_2fPoint&) const;
  ON_2fPoint  operator+(const ON_2fVector&) const;
  ON_2fVector operator-(const ON_2fPoint&) const;
  ON_2fPoint  operator-(const ON_2fVector&) const;
  ON_3fPoint  operator+(const ON_3fPoint&) const;
  ON_3fPoint  operator+(const ON_3fVector&) const;
  ON_3fVector operator-(const ON_3fPoint&) const;
  ON_3fPoint  operator-(const ON_3fVector&) const;

  ON_2dPoint  operator+(const ON_2dPoint&) const;
  ON_2dPoint  operator+(const ON_2dVector&) const;
  ON_2dVector operator-(const ON_2dPoint&) const;
  ON_2dPoint  operator-(const ON_2dVector&) const;
  ON_3dPoint  operator+(const ON_3dPoint&) const;
  ON_3dPoint  operator+(const ON_3dVector&) const;
  ON_3dVector operator-(const ON_3dPoint&) const;
  ON_3dPoint  operator-(const ON_3dVector&) const;

  float operator*(const ON_2fPoint&) const; // for points acting as vectors
  float operator*(const ON_2fVector&) const; // for points acting as vectors
  float operator*(const ON_4fPoint&) const;

  bool operator==(const ON_2fPoint&) const;
  bool operator!=(const ON_2fPoint&) const;

  // dictionary order comparisons
  bool operator<=(const ON_2fPoint&) const;
  bool operator>=(const ON_2fPoint&) const;
  bool operator<(const ON_2fPoint&) const;
  bool operator>(const ON_2fPoint&) const;

  // index operators mimic float[2] behavior
  float& operator[](int);
  float operator[](int) const;
  float& operator[](unsigned int);
  float operator[](unsigned int) const;

  // set 2d point value
  void Set(float,float);

  double DistanceTo( const ON_2fPoint& ) const;

  int MaximumCoordinateIndex() const;
  double MaximumCoordinate() const; // absolute value of maximum coordinate

  void Zero(); // set all coordinates to zero;

  // These transform the point in place. The transformation matrix acts on
  // the left of the point; i.e., result = transformation*point
  void Transform( 
        const ON_Xform&
        );

  void Rotate( // rotatation in XY plane
        double,              // angle in radians
        const ON_2fPoint&   // center of rotation
        );

  void Rotate( // rotatation in XY plane
        double,              // sin(angle)
        double,              // cos(angle)
        const ON_2fPoint&   // center of rotation
        );
};

ON_DECL
ON_2fPoint operator*(int, const ON_2fPoint&);

ON_DECL
ON_2fPoint operator*(float, const ON_2fPoint&);

ON_DECL
ON_2dPoint operator*(double, const ON_2fPoint&);

////////////////////////////////////////////////////////////////
//
//   ON_3fPoint
//
class ON_CLASS ON_3fPoint
{
public:
  float x, y, z;

  static const ON_3fPoint Origin; // (0.0f,0.0f,0.0f)

  // use implicit destructor, copy constructor
  ON_3fPoint();                       // x,y,z not initialized
  ON_3fPoint(float x,float y,float z);
  ON_3fPoint(const ON_2fPoint& );     // from 2f point
  ON_3fPoint(const ON_4fPoint& );     // from 4f point
  ON_3fPoint(const ON_2fVector& );    // from 2f vector
  ON_3fPoint(const ON_3fVector& );    // from 3f vector
  ON_3fPoint(const float*);           // from float[3] array

  ON_3fPoint(const ON_2dPoint& );     // from 2d point
  ON_3fPoint(const ON_3dPoint& );     // from 3d point
  ON_3fPoint(const ON_4dPoint& );     // from 4d point
  ON_3fPoint(const ON_2dVector& );    // from 2d vector
  ON_3fPoint(const ON_3dVector& );    // from 3d vector
  ON_3fPoint(const double*);          // from double[3] array

  // (float*) conversion operators
  operator float*();
  operator const float*() const;

  // use implicit operator=(const ON_3fPoint&)
  ON_3fPoint& operator=(const ON_2fPoint&);
  ON_3fPoint& operator=(const ON_4fPoint&);
  ON_3fPoint& operator=(const ON_2fVector&);
  ON_3fPoint& operator=(const ON_3fVector&);
  ON_3fPoint& operator=(const float*);  // point = float[3] support

  ON_3fPoint& operator=(const ON_2dPoint&);
  ON_3fPoint& operator=(const ON_3dPoint&);
  ON_3fPoint& operator=(const ON_4dPoint&);
  ON_3fPoint& operator=(const ON_2dVector&);
  ON_3fPoint& operator=(const ON_3dVector&);
  ON_3fPoint& operator=(const double*); // point = double[3] support

  ON_3fPoint& operator*=(float);
  ON_3fPoint& operator/=(float);
  ON_3fPoint& operator+=(const ON_3fPoint&);
  ON_3fPoint& operator+=(const ON_3fVector&);
  ON_3fPoint& operator-=(const ON_3fPoint&);
  ON_3fPoint& operator-=(const ON_3fVector&);

  ON_3fPoint  operator*(int) const;
  ON_3fPoint  operator/(int) const;
  ON_3fPoint  operator*(float) const;
  ON_3fPoint  operator/(float) const;
  ON_3dPoint  operator*(double) const;
  ON_3dPoint  operator/(double) const;

  ON_3fPoint  operator+(const ON_3fPoint&) const;
  ON_3fPoint  operator+(const ON_3fVector&) const;
  ON_3fVector operator-(const ON_3fPoint&) const;
  ON_3fPoint  operator-(const ON_3fVector&) const;
  ON_3fPoint  operator+(const ON_2fPoint&) const;
  ON_3fPoint  operator+(const ON_2fVector&) const;
  ON_3fVector operator-(const ON_2fPoint&) const;
  ON_3fPoint  operator-(const ON_2fVector&) const;

  ON_3dPoint  operator+(const ON_3dPoint&) const;
  ON_3dPoint  operator+(const ON_3dVector&) const;
  ON_3dVector operator-(const ON_3dPoint&) const;
  ON_3dPoint  operator-(const ON_3dVector&) const;
  ON_3dPoint  operator+(const ON_2dPoint&) const;
  ON_3dPoint  operator+(const ON_2dVector&) const;
  ON_3dVector operator-(const ON_2dPoint&) const;
  ON_3dPoint  operator-(const ON_2dVector&) const;

  float operator*(const ON_3fPoint&) const; // for points acting as vectors
  float operator*(const ON_3fVector&) const; // for points acting as vectors
  float operator*(const ON_4fPoint&) const;

  bool operator==(const ON_3fPoint&) const;
  bool operator!=(const ON_3fPoint&) const;

  // dictionary order comparisons
  bool operator<=(const ON_3fPoint&) const;
  bool operator>=(const ON_3fPoint&) const;
  bool operator<(const ON_3fPoint&) const;
  bool operator>(const ON_3fPoint&) const;

  // index operators mimic float[3] behavior
  float& operator[](int);
  float operator[](int) const;
  float& operator[](unsigned int);
  float operator[](unsigned int) const;

  // set 3d point value
  void Set(float,float,float);

  double DistanceTo( const ON_3fPoint& ) const;

  int MaximumCoordinateIndex() const;
  double MaximumCoordinate() const; // absolute value of maximum coordinate
  double Fuzz( double = ON_ZERO_TOLERANCE ) const; // tolerance to use when comparing 3d points

  void Zero(); // set all coordinates to zero;

  // These transform the point in place. The transformation matrix acts on
  // the left of the point; i.e., result = transformation*point
  void Transform( 
        const ON_Xform&
        );

  void Rotate( 
        double,               // angle in radians
        const ON_3fVector&, // axis of rotation
        const ON_3fPoint&   // center of rotation
        );

  void Rotate( 
        double,               // sin(angle)
        double,               // cos(angle)
        const ON_3fVector&, // axis of rotation
        const ON_3fPoint&   // center of rotation
        );
};

ON_DECL
ON_3fPoint operator*(int, const ON_3fPoint&);

ON_DECL
ON_3fPoint operator*(float, const ON_3fPoint&);

ON_DECL
ON_3dPoint operator*(double, const ON_3fPoint&);

////////////////////////////////////////////////////////////////
//
//   ON_4fPoint (homogeneous coordinates)
//
class ON_CLASS ON_4fPoint
{
public:
  float x, y, z, w;

  // use implicit destructor, copy constructor
  ON_4fPoint();                        // x,y,z,w not initialized
  ON_4fPoint(float x,float y,float z,float w);

  ON_4fPoint(const ON_2fPoint& );     // from 2f point
  ON_4fPoint(const ON_3fPoint& );     // from 3f point
  ON_4fPoint(const ON_2fVector& );    // from 2f vector
  ON_4fPoint(const ON_3fVector& );    // from 3f vector
  ON_4fPoint(const float*);           // from float[4] array

  ON_4fPoint(const ON_2dPoint& );     // from 2d point
  ON_4fPoint(const ON_3dPoint& );     // from 3d point
  ON_4fPoint(const ON_4dPoint& );     // from 4d point
  ON_4fPoint(const ON_2dVector& );    // from 2d vector
  ON_4fPoint(const ON_3dVector& );    // from 3d vector
  ON_4fPoint(const double*);          // from double[4] array

  // (float*) conversion operators
  operator float*();
  operator const float*() const;

  // use implicit operator=(const ON_4fPoint&)
  ON_4fPoint& operator=(const ON_2fPoint&);
  ON_4fPoint& operator=(const ON_3fPoint&);
  ON_4fPoint& operator=(const ON_2fVector&);
  ON_4fPoint& operator=(const ON_3fVector&);
  ON_4fPoint& operator=(const float*);  // point = float[4] support

  ON_4fPoint& operator=(const ON_2dPoint&);
  ON_4fPoint& operator=(const ON_3dPoint&);
  ON_4fPoint& operator=(const ON_4dPoint&);
  ON_4fPoint& operator=(const ON_2dVector&);
  ON_4fPoint& operator=(const ON_3dVector&);
  ON_4fPoint& operator=(const double*); // point = double[4] support

  ON_4fPoint& operator*=(float);
  ON_4fPoint& operator/=(float);
  ON_4fPoint& operator+=(const ON_4fPoint&);
  ON_4fPoint& operator-=(const ON_4fPoint&);

  ON_4fPoint  operator*(float) const;
  ON_4fPoint  operator/(float) const;
  ON_4fPoint  operator+(const ON_4fPoint&) const; // sum w = sqrt(w1*w2)
  ON_4fPoint  operator-(const ON_4fPoint&) const; // difference w = sqrt(w1*w2)

  float operator*(const ON_4fPoint&) const;

  // projective comparison 
  // (i.e., [x,y,z,w] == [c*x,c*y,c*z,c*w] is true for nonzero c)
  bool operator==(ON_4fPoint) const;
  bool operator!=(const ON_4fPoint&) const;

  // index operators mimic float[4] behavior
  float& operator[](int);
  float operator[](int) const;
  float& operator[](unsigned int);
  float operator[](unsigned int) const;

  // set 4d point value
  void Set(float,float,float,float);

  int MaximumCoordinateIndex() const;
  double MaximumCoordinate() const; // absolute value of maximum coordinate

  void Zero();      // set all 4 coordinates to zero;
  bool Normalize(); // set so x^2 + y^2 + z^2 + w^2 = 1

  // These transform the point in place. The transformation matrix acts on
  // the left of the point; i.e., result = transformation*point
  void Transform( 
        const ON_Xform&
        );
};

ON_DECL
ON_4fPoint operator*(float, const ON_4fPoint&);

ON_DECL
ON_4dPoint operator*(double, const ON_4fPoint&);

////////////////////////////////////////////////////////////////
//
//   ON_2fVector
//
class ON_CLASS ON_2fVector
{
public:
  float x, y;

  static const ON_2fVector ZeroVector; // (0.0f,0.0f)
  static const ON_2fVector XAxis;      // (1.0f,0.0f)
  static const ON_2fVector YAxis;      // (0.0f,1.0f)

  // Description:
  //   A index driven function to get unit axis vectors.
  // Parameters:
  //   index - [in] 0 returns (1,0), 1 returns (0,1)
  // Returns:
  //   Unit 3d vector with vector[i] = (i==index)?1:0;
  static const ON_2fVector& UnitVector(
    int // index
    );

  // use implicit destructor, copy constructor
  ON_2fVector();                      // x,y not initialized
  ON_2fVector(float x,float y);
  ON_2fVector(const ON_2fPoint& );     // from 2f point
  ON_2fVector(const ON_3fPoint& );     // from 3f point
  ON_2fVector(const ON_4fPoint& );     // from 4f point
  ON_2fVector(const ON_3fVector& );    // from 3f vector
  ON_2fVector(const float*);           // from float[2] array

  ON_2fVector(const ON_2dPoint& );     // from 2d point
  ON_2fVector(const ON_3dPoint& );     // from 3d point
  ON_2fVector(const ON_4dPoint& );     // from 4d point
  ON_2fVector(const ON_2dVector& );    // from 2d vector
  ON_2fVector(const ON_3dVector& );    // from 3d vector
  ON_2fVector(const double*);          // from double[2] array

  // (float*) conversion operators
  operator float*();
  operator const float*() const;

  // use implicit operator=(const ON_2fVector&)
  ON_2fVector& operator=(const ON_2fPoint&);
  ON_2fVector& operator=(const ON_3fPoint&);
  ON_2fVector& operator=(const ON_3fVector&);
  ON_2fVector& operator=(const float*);  // point = float[2] support

  ON_2fVector& operator=(const ON_2dPoint&);
  ON_2fVector& operator=(const ON_3dPoint&);
  ON_2fVector& operator=(const ON_2dVector&);
  ON_2fVector& operator=(const ON_3dVector&);
  ON_2fVector& operator=(const double*); // point = double[2] support

  ON_2fVector  operator-() const;

  ON_2fVector& operator*=(float);
  ON_2fVector& operator/=(float);
  ON_2fVector& operator+=(const ON_2fVector&);
  ON_2fVector& operator-=(const ON_2fVector&);

  float operator*(const ON_2fVector&) const; // inner (dot) product
  float operator*(const ON_2fPoint&) const; // inner (dot) product point acting as a vector
  double operator*(const ON_2dVector&) const; // inner (dot) product
  
  ON_2fVector  operator*(int) const;
  ON_2fVector  operator/(int) const;
  ON_2fVector  operator*(float) const;
  ON_2fVector  operator/(float) const;
  ON_2dVector  operator*(double) const;
  ON_2dVector  operator/(double) const;

  ON_2fVector  operator+(const ON_2fVector&) const;
  ON_2fPoint   operator+(const ON_2fPoint&) const;
  ON_2fVector  operator-(const ON_2fVector&) const;
  ON_2fPoint   operator-(const ON_2fPoint&) const;
  ON_3fVector  operator+(const ON_3fVector&) const;
  ON_3fPoint   operator+(const ON_3fPoint&) const;
  ON_3fVector  operator-(const ON_3fVector&) const;
  ON_3fPoint   operator-(const ON_3fPoint&) const;

  ON_2dVector  operator+(const ON_2dVector&) const;
  ON_2dPoint   operator+(const ON_2dPoint&) const;
  ON_2dVector  operator-(const ON_2dVector&) const;
  ON_2dPoint   operator-(const ON_2dPoint&) const;
  ON_3dVector  operator+(const ON_3dVector&) const;
  ON_3dPoint   operator+(const ON_3dPoint&) const;
  ON_3dVector  operator-(const ON_3dVector&) const;
  ON_3dPoint   operator-(const ON_3dPoint&) const;

  float operator*(const ON_4fPoint&) const;

  bool operator==(const ON_2fVector&) const;
  bool operator!=(const ON_2fVector&) const;

  // dictionary order comparisons
  bool operator<=(const ON_2fVector&) const;
  bool operator>=(const ON_2fVector&) const;
  bool operator<(const ON_2fVector&) const;
  bool operator>(const ON_2fVector&) const;

  // index operators mimic float[2] behavior
  float& operator[](int);
  float operator[](int) const;
  float& operator[](unsigned int);
  float operator[](unsigned int) const;

  // set 2d vector value
  void Set(float,float);

  int MaximumCoordinateIndex() const;
  double MaximumCoordinate() const; // absolute value of maximum coordinate

  double LengthSquared() const;
  double Length() const;

  bool Decompose( // Computes a, b such that this vector = a*X + b*Y
         // Returns false if unable to solve for a,b.  This happens
         // when X,Y is not really a basis.
         //
         // If X,Y is known to be an orthonormal frame,
         // then a = V*X, b = V*Y will compute
         // the same result more quickly.
         const ON_2fVector&, // X
         const ON_2fVector&, // Y
         double*, // a
         double*  // b
         ) const;

  int IsParallelTo( 
        // returns  1: this and other vectors are parallel
        //         -1: this and other vectors are anti-parallel
        //          0: this and other vectors are not parallel
        //             or at least one of the vectors is zero
        const ON_2fVector&,                 // other vector     
        double = ON_DEFAULT_ANGLE_TOLERANCE // optional angle tolerance (radians)
        ) const;

  bool IsPerpendicularTo(
        // returns true:  this and other vectors are perpendicular
        //         false: this and other vectors are not perpendicular
        //                or at least one of the vectors is zero
        const ON_2fVector&,                 // other vector     
        double = ON_DEFAULT_ANGLE_TOLERANCE // optional angle tolerance (radians)
        ) const;

  void Zero(); // set all coordinates to zero;
  void Reverse(); // negate all coordinates
  bool Unitize();  // returns false if vector has zero length
  bool IsUnitVector() const;

  // Description:
  //   Test a vector to see if it is very short
  //
  // Parameters:
  //   tiny_tol - [in] (default = ON_ZERO_TOLERANCE) a nonzero
  //              value used as the coordinate zero tolerance.
  //
  // Returns:
  //   ( fabs(x) <= tiny_tol && fabs(y) <= tiny_tol )
  //
  bool IsTiny(
         double = ON_ZERO_TOLERANCE // tiny_tol
         ) const;

  // Returns:
  //   true if vector is the zero vector.
  bool IsZero() const;

  // set this vector to be perpendicular to another vector
  bool PerpendicularTo( // Result is not unitized. 
                        // returns false if input vector is zero
        const ON_2fVector& 
        );

  // set this vector to be perpendicular to a line defined by 2 points
  bool PerpendicularTo( 
        const ON_2fPoint&, 
        const ON_2fPoint& 
        );
};

ON_DECL
ON_2fVector operator*(int, const ON_2fVector&);

ON_DECL
ON_2fVector operator*(float, const ON_2fVector&);

ON_DECL
ON_2dVector operator*(double, const ON_2fVector&);

///////////////////////////////////////////////////////////////
//
// ON_2fVector utilities
//

ON_DECL
float 
ON_DotProduct( 
    const ON_2fVector&, 
    const ON_2fVector& 
    );

ON_DECL
ON_3fVector 
ON_CrossProduct(
    const ON_2fVector&, 
    const ON_2fVector& 
    );

ON_DECL
bool 
ON_IsOrthogonalFrame( // true if X, Y are nonzero and mutually perpendicular
    const ON_2fVector&, // X
    const ON_2fVector&  // Y
    );

ON_DECL
bool 
ON_IsOrthonormalFrame( // true if X, Y are orthogonal and unit length
    const ON_2fVector&, // X
    const ON_2fVector&  // Y
    );

ON_DECL
bool 
ON_IsRightHandFrame( // true if X, Y are orthonormal and right handed
    const ON_2fVector&, // X
    const ON_2fVector&  // Y
    );

////////////////////////////////////////////////////////////////
//
//   ON_3fVector
//
class ON_CLASS ON_3fVector
{
public:
  float x, y, z;

  static const ON_3fVector ZeroVector; // (0.0f,0.0f,0.0f)
  static const ON_3fVector XAxis;      // (1.0f,0.0f,0.0f)
  static const ON_3fVector YAxis;      // (0.0f,1.0f,0.0f)
  static const ON_3fVector ZAxis;      // (0.0f,0.0f,1.0f)

  // Description:
  //   A index driven function to get unit axis vectors.
  // Parameters:
  //   index - [in] 0 returns (1,0,0), 1 returns (0,1,0)
  //                2 returns (0,0,1)
  // Returns:
  //   Unit 3d vector with vector[i] = (i==index)?1:0;
  static const ON_3fVector& UnitVector(
    int // index
    );

  // use implicit destructor, copy constructor
  ON_3fVector();                      // x,y,z not initialized
  ON_3fVector(float x,float y,float z);

  ON_3fVector(const ON_2fPoint& );     // from 2f point
  ON_3fVector(const ON_3fPoint& );     // from 3f point
  ON_3fVector(const ON_4fPoint& );     // from 4f point
  ON_3fVector(const ON_2fVector& );    // from 2f vector
  ON_3fVector(const float*);           // from float[3] array

  ON_3fVector(const ON_2dPoint& );     // from 2d point
  ON_3fVector(const ON_3dPoint& );     // from 3d point
  ON_3fVector(const ON_4dPoint& );     // from 4d point
  ON_3fVector(const ON_2dVector& );    // from 2d vector
  ON_3fVector(const ON_3dVector& );    // from 3d vector
  ON_3fVector(const double*);          // from double[3] array

  // (float*) conversion operators
  operator float*();
  operator const float*() const;

  // use implicit operator=(const ON_3fVector&)
  ON_3fVector& operator=(const ON_2fPoint&);
  ON_3fVector& operator=(const ON_3fPoint&);
  ON_3fVector& operator=(const ON_4fPoint&);
  ON_3fVector& operator=(const ON_2fVector&);
  ON_3fVector& operator=(const float*);  // point = float[3] support

  ON_3fVector& operator=(const ON_2dPoint&);
  ON_3fVector& operator=(const ON_3dPoint&);
  ON_3fVector& operator=(const ON_4dPoint&);
  ON_3fVector& operator=(const ON_2dVector&);
  ON_3fVector& operator=(const ON_3dVector&);
  ON_3fVector& operator=(const double*); // point = double[3] support
  
  ON_3fVector  operator-() const;

  ON_3fVector& operator*=(float);
  ON_3fVector& operator/=(float);
  ON_3fVector& operator+=(const ON_3fVector&);
  ON_3fVector& operator-=(const ON_3fVector&);

  float operator*(const ON_3fVector&) const; // inner (dot) product
  float operator*(const ON_3fPoint&) const; // inner (dot) product (point acting as a vector)
  double operator*(const ON_3dVector&) const; // inner (dot) product

  ON_3fVector  operator*(int) const;
  ON_3fVector  operator/(int) const;
  ON_3fVector  operator*(float) const;
  ON_3fVector  operator/(float) const;
  ON_3dVector  operator*(double) const;
  ON_3dVector  operator/(double) const;

  ON_3fVector  operator+(const ON_3fVector&) const;
  ON_3fPoint   operator+(const ON_3fPoint&) const;
  ON_3fVector  operator-(const ON_3fVector&) const;
  ON_3fPoint   operator-(const ON_3fPoint&) const;
  ON_3fVector  operator+(const ON_2fVector&) const;
  ON_3fPoint   operator+(const ON_2fPoint&) const;
  ON_3fVector  operator-(const ON_2fVector&) const;
  ON_3fPoint   operator-(const ON_2fPoint&) const;

  ON_3dVector  operator+(const ON_3dVector&) const;
  ON_3dPoint   operator+(const ON_3dPoint&) const;
  ON_3dVector  operator-(const ON_3dVector&) const;
  ON_3dPoint   operator-(const ON_3dPoint&) const;
  ON_3dVector  operator+(const ON_2dVector&) const;
  ON_3dPoint   operator+(const ON_2dPoint&) const;
  ON_3dVector  operator-(const ON_2dVector&) const;
  ON_3dPoint   operator-(const ON_2dPoint&) const;

  float operator*(const ON_4fPoint&) const;

  bool operator==(const ON_3fVector&) const;
  bool operator!=(const ON_3fVector&) const;

  // dictionary order comparisons
  bool operator<=(const ON_3fVector&) const;
  bool operator>=(const ON_3fVector&) const;
  bool operator<(const ON_3fVector&) const;
  bool operator>(const ON_3fVector&) const;

  // index operators mimic float[3] behavior
  float& operator[](int);
  float operator[](int) const;
  float& operator[](unsigned int);
  float operator[](unsigned int) const;

  // set 3d vector value
  void Set(float,float,float);

  int MaximumCoordinateIndex() const;
  double MaximumCoordinate() const; // absolute value of maximum coordinate

  double LengthSquared() const;
  double Length() const;

  bool IsPerpendicularTo(
        // returns true:  this and other vectors are perpendicular
        //         false: this and other vectors are not perpendicular
        //                or at least one of the vectors is zero
        const ON_3fVector&,                 // other vector     
        double = ON_DEFAULT_ANGLE_TOLERANCE // optional angle tolerance (radians)
        ) const;

  double Fuzz( double = ON_ZERO_TOLERANCE ) const; // tolerance to use when comparing 3d vectors

  void Zero(); // set all coordinates to zero
  void Reverse(); // negate all coordinates
  bool Unitize();  // returns false if vector has zero length
  bool IsUnitVector() const;

  // Description:
  //   Test a vector to see if it is very short
  //
  // Parameters:
  //   tiny_tol - [in] (default = ON_ZERO_TOLERANCE) a nonzero
  //              value used as the coordinate zero tolerance.
  //
  // Returns:
  //   ( fabs(x) <= tiny_tol && fabs(y) <= tiny_tol && fabs(z) <= tiny_tol )
  //
  bool IsTiny(
         double = ON_ZERO_TOLERANCE // tiny_tol
         ) const;

  // Returns:
  //   true if vector is the zero vector.
  bool IsZero() const;

  // set this vector to be perpendicular to another vector
  bool PerpendicularTo( // Result is not unitized. 
                        // returns false if input vector is zero
        const ON_3fVector& 
        );

  // These transform the vector in place. The transformation matrix acts on
  // the left of the vector; i.e., result = transformation*vector
  void Transform( 
        const ON_Xform& // can use ON_Xform here
        );

  void Rotate( 
        double,             // angle in radians
        const ON_3fVector&  // axis of rotation
        );

  void Rotate( 
        double,             // sin(angle)
        double,             // cos(angle)
        const ON_3fVector&  // axis of rotation
        );
};

ON_DECL
ON_3fVector operator*(int, const ON_3fVector&);

ON_DECL
ON_3fVector operator*(float, const ON_3fVector&);

ON_DECL
ON_3dVector operator*(double, const ON_3fVector&);

///////////////////////////////////////////////////////////////
//
// ON_3fVector utilities
//

ON_DECL
float 
ON_DotProduct( 
    const ON_3fVector&, 
    const ON_3fVector& 
    );


ON_DECL
ON_3fVector 
ON_CrossProduct(
    const ON_3fVector&, 
    const ON_3fVector& 
    );

ON_DECL
ON_3fVector 
ON_CrossProduct( // 3d cross product for old fashioned arrays
    const float*, // array of 3d floats
    const float*  // array of 3d floats
    );

ON_DECL
float 
ON_TripleProduct( 
    const ON_3fVector&,
    const ON_3fVector&,
    const ON_3fVector&
    );

ON_DECL
float 
ON_TripleProduct(  // 3d triple product for old fashioned arrays
    const float*, // array of 3d floats
    const float*, // array of 3d floats
    const float*  // array of 3d floats
    );

ON_DECL
bool 
ON_IsOrthogonalFrame( // true if X, Y, Z are nonzero and mutually perpendicular
    const ON_3fVector&, // X
    const ON_3fVector&, // Y
    const ON_3fVector&  // Z 
    );

ON_DECL
bool 
ON_IsOrthonormalFrame( // true if X, Y, Z are orthogonal and unit length
    const ON_3fVector&, // X
    const ON_3fVector&, // Y
    const ON_3fVector&  // Z 
    );

ON_DECL
bool 
ON_IsRightHandFrame( // true if X, Y, Z are orthonormal and right handed
    const ON_3fVector&, // X
    const ON_3fVector&, // Y
    const ON_3fVector&  // Z 
    );

///////////////////////////////////////////////////////////////
//
// common points and vectors
//

// ON_forigin is OBSOLETE - use ON_3fPoint::Origin
extern ON_EXTERN_DECL const ON_3fPoint ON_forigin; // (0.0, 0.0, 0.0)

// ON_fxaxis is OBSOLETE - use ON_3fPoint::XAxis
extern ON_EXTERN_DECL const ON_3fVector ON_fxaxis; // (1.0, 0.0, 0.0)

// ON_fyaxis is OBSOLETE - use ON_3fVector::YAxis
extern ON_EXTERN_DECL const ON_3fVector ON_fyaxis; // (0.0, 1.0, 0.0)

// ON_fzaxis is OBSOLETE - use ON_3fVector::ZAxis
extern ON_EXTERN_DECL const ON_3fVector ON_fzaxis; // (0.0, 0.0, 1.0)


#endif
