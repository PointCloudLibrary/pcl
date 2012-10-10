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
//   defines double precision point, vector, and array classes
//
////////////////////////////////////////////////////////////////
#if !defined(ON_POINT_INC_)
#define ON_POINT_INC_

#include <pcl/pcl_exports.h>

class ON_BoundingBox;
class ON_Xform;
class ON_Line;
class ON_Plane;

class ON_2dPoint;
class ON_3dPoint;
class ON_4dPoint;

class ON_2dVector;
class ON_3dVector;

class ON_2fVector;
class ON_3fVector;

class ON_Interval;

////////////////////////////////////////////////////////////////
//
//   ON_Interval
//
class PCL_EXPORTS ON_CLASS ON_Interval
{
public:

  static const ON_Interval EmptyInterval; // (ON_UNSET_VALUE,ON_UNSET_VALUE)

  ////////
  // The default constructor creates an empty interval (ON_UNSET_VALUE,ON_UNSET_VALUE)
  ON_Interval();

  ON_Interval(double t0,double t1);

  ~ON_Interval();

  bool operator!=(const ON_Interval&) const;
  bool operator==(const ON_Interval&) const;

  // Interval = [m_t[0], m_t[1]]
  double m_t[2];

  /*
  Description:
    Sets interval to (ON_UNSET_VALUE,ON_UNSET_VALUE)
  See Also:
    ON_Interval::Set
  */
  void Destroy();

  /*
  Description:
    Sets interval to [t0,t1]
  Parameters:
    t0 - [in]
    t1 - [in]
  See Also:
    ON_Interval::ON_Interval( double, double )
  */
  void Set(
    double t0, 
    double t1
    );

  /*
  Description:
    Convert normalized parameter to interval value, or pair of values.
  Parameters:
    normalized_parameter - [in] 
  Returns:
    Interval parameter
    min*(1.0-normalized_parameter) + max*normalized_parameter
  See Also:
    ON_Interval::NormalizedParameterAt
  */
  double ParameterAt (
    double normalized_parameter
    ) const; 
  ON_Interval ParameterAt (
    ON_Interval normalized_interval
    ) const; 
  
  /*
  Description:
    Convert interval value, or pair of values, to normalized parameter.
  Parameters:
    interval_parameter - [in] value in interval
  Returns:
    Normalized parameter x so that 
    min*(1.0-x) + max*x = interval_parameter.
  See Also:
    ON_Interval::ParameterAt
  */
  double NormalizedParameterAt (
    double interval_parameter
    ) const;
  ON_Interval NormalizedParameterAt (
    ON_Interval interval_parameter
    ) const;

  double& operator[](int); // returns (index<=0) ? m_t[0] : m_t[1]
  double operator[](int) const; // returns (index<=0) ? m_t[0] : m_t[1]
  double& operator[](unsigned int); // returns (index<=0) ? m_t[0] : m_t[1]
  double operator[](unsigned int) const; // returns (index<=0) ? m_t[0] : m_t[1]

  double Min() const; // returns smaller of m_t[0] and m_t[1]
  double Max() const; // returns larger of m_t[0] and m_t[1]
  double Mid() const; // returns 0.5*(m_t[0] + m_t[1])
  double Length() const;

  bool IsIncreasing() const; // returns true if m_t[0] < m_t[1]
  bool IsDecreasing() const; // returns true if m_t[0] > m_t[0];
  bool IsInterval() const;   // returns truc if m_t[0] != m_t[1]
  bool IsSingleton() const;  // returns true if m_t[0] == m_t[1] != ON_UNSET_VALUE
  bool IsEmptyInterval() const;   // returns true if m_t[0] == m_t[1] == ON_UNSET_VALUE
  bool IsValid() const;      // returns ON_IsValid(m_t[0]) && ON_IsValid(m_t[1])

  // OBSOLETE - Use IsEmptyInterval()
  bool IsEmptySet() const;   // returns true if m_t[0] == m_t[1] == ON_UNSET_VALUE

	bool MakeIncreasing();		// returns true if resulting interval IsIncreasing() 

  /*
  Returns:
    @untitled table
     0      this is idential to other
    -1      this[0] < other[0]
    +1      this[0] > other[0]
    -1      this[0] == other[0] and this[1] < other[1]
    +1      this[0] == other[0] and this[1] > other[1]
  */
  int Compare( const ON_Interval& other ) const;

  /* 
  Description:
    Test a value t to see if it is inside the interval.
  Parameters:
    t - [in] value to test
    bTestOpenInterval - [in] 
        If false, t is tested to see if it satisfies min <= t <= max.
        If true, t is tested to see if it satisfies min < t < max.
  Returns:
    true if t is in the interval and false if t is not
    in the interval.
  */
  bool Includes(
    double t,
    bool bTestOpenInterval = false
    ) const;

  /* 
  Description:
    Test an interval to see if it is contained in this interval.
  Parameters:
    other - [in] interval to test
    bProperSubSet - [in] if true, then the test is for a proper subinterval.
  Returns:
    If bProperSubSet is false, then the result is true when
    this->Min() <= other.Min() and other.Max() <= this->Max().
    If bProperSubSet is true, then the result is true when
    this->Min() <= other.Min() and other.Max() <= this->Max()
    and at least one of the inequalites is strict.
  */
  bool Includes( 
    const ON_Interval& other,
    bool bProperSubSet = false
    ) const;

  /*
  Description:
    Changes interval to [-m_t[1],-m_t[0]].
  */
  void Reverse();

  /*
  Description:
    Swaps m_t[0] and m_t[1].
  */
  void Swap();

  //////////
  // If the intersection is not empty, then 
  // intersection = [max(this.Min(),arg.Min()), min(this.Max(),arg.Max())]
  // Intersection() returns true if the intersection is not empty.
  // The interval [ON_UNSET_VALUE,ON_UNSET_VALUE] is considered to be
  // the empty set interval.  The result of any intersection involving an
  // empty set interval or disjoint intervals is the empty set interval.
  bool Intersection( // this = this intersect arg
         const ON_Interval&
         );

  //////////
  // If the intersection is not empty, then 
  // intersection = [max(argA.Min(),argB.Min()), min(argA.Max(),argB.Max())]
  // Intersection() returns true if the intersection is not empty.
  // The interval [ON_UNSET_VALUE,ON_UNSET_VALUE] is considered to be
  // the empty set interval.  The result of any intersection involving an
  // empty set interval or disjoint intervals is the empty set interval.
  bool Intersection( // this = intersection of two args
         const ON_Interval&, 
         const ON_Interval&
         );

  //////////
  // The union of an empty set and an increasing interval is the increasing
  // interval.  The union of two empty sets is empty. The union of an empty
  // set an a non-empty interval is the non-empty interval.
  // The union of two non-empty intervals is
  // union = [min(this.Min(),arg.Min()), max(this.Max(),arg.Max()),]
  // Union() returns true if the union is not empty.
  bool Union( // this = this union arg
         const ON_Interval&
         );

  bool Union( // this = this union arg
         double t
         );

  bool Union( // this = this union arg
         int count,
         const double* t
         );

  //////////
  // The union of an empty set and an increasing interval is the increasing
  // interval.  The union of two empty sets is empty. The union of an empty
  // set an a non-empty interval is the non-empty interval.
  // The union of two non-empty intervals is
  // union = [min(argA.Min(),argB.Min()), max(argA.Max(),argB.Max()),]
  // Union() returns true if the union is not empty.
  bool Union( // this = union of two args
         const ON_Interval&, 
         const ON_Interval&
         );
};

////////////////////////////////////////////////////////////////
//
//   ON_2dPoint
//
class PCL_EXPORTS ON_CLASS ON_2dPoint
{
public:
  double x, y;

  static const ON_2dPoint Origin;     // (0.0,0.0)
  static const ON_2dPoint UnsetPoint; // (ON_UNSET_VALUE,ON_UNSET_VALUE)

  // use implicit destructor, copy constructor
  ON_2dPoint();                         // x,y not initialized
  ON_2dPoint(double x,double y);
  ON_2dPoint(const ON_3dPoint& );       // from 3d point
  ON_2dPoint(const ON_4dPoint& );       // from 4d point
  ON_2dPoint(const ON_2dVector& );      // from 2d vector
  ON_2dPoint(const ON_3dVector& );      // from 3d vector
  ON_2dPoint(const double*);            // from double[2] array

  ON_2dPoint(const class ON_2fPoint&);  // from 2f point
  ON_2dPoint(const class ON_3fPoint&);  // from 3f point
  ON_2dPoint(const class ON_4fPoint&);  // from 4f point
  ON_2dPoint(const class ON_2fVector&); // from 2f point
  ON_2dPoint(const class ON_3fVector&); // from 3f point
  ON_2dPoint(const float*);             // from float[2] array

  // (double*) conversion operators
  operator double*();
  operator const double*() const;

  // use implicit operator=(const ON_2dPoint&)
  ON_2dPoint& operator=(const ON_3dPoint&);
  ON_2dPoint& operator=(const ON_4dPoint&);
  ON_2dPoint& operator=(const ON_2dVector&);
  ON_2dPoint& operator=(const ON_3dVector&);
  ON_2dPoint& operator=(const double*); // point = double[2] support

  ON_2dPoint& operator=(const ON_2fPoint&);
  ON_2dPoint& operator=(const ON_3fPoint&);
  ON_2dPoint& operator=(const ON_4fPoint&);
  ON_2dPoint& operator=(const ON_2fVector&);
  ON_2dPoint& operator=(const ON_3fVector&);
  ON_2dPoint& operator=(const float*);  // point = float[2] support

  ON_2dPoint& operator*=(double);
  ON_2dPoint& operator/=(double);
  ON_2dPoint& operator+=(const ON_2dPoint&);  // Adding this was a mistake - cannot remove without breaking SDK
  ON_2dPoint& operator+=(const ON_2dVector&);
  ON_2dPoint& operator+=(const ON_3dVector&); // Adding this was a mistake - cannot remove without breaking SDK
  ON_2dPoint& operator-=(const ON_2dPoint&);  // Adding this was a mistake - cannot remove without breaking SDK
  ON_2dPoint& operator-=(const ON_2dVector&);
  ON_2dPoint& operator-=(const ON_3dVector&); // Adding this was a mistake - cannot remove without breaking SDK

  ON_2dPoint  operator*(int) const;
  ON_2dPoint  operator/(int) const;
  ON_2dPoint  operator*(float) const;
  ON_2dPoint  operator/(float) const;
  ON_2dPoint  operator*(double) const;
  ON_2dPoint  operator/(double) const;

  ON_2dPoint  operator+(const ON_2dPoint&) const;
  ON_2dPoint  operator+(const ON_2dVector&) const;
  ON_2dVector operator-(const ON_2dPoint&) const;
  ON_2dPoint  operator-(const ON_2dVector&) const;
  ON_3dPoint  operator+(const ON_3dPoint&) const;
  ON_3dPoint  operator+(const ON_3dVector&) const;
  ON_3dVector operator-(const ON_3dPoint&) const;
  ON_3dPoint  operator-(const ON_3dVector&) const;

  ON_2dPoint  operator+(const ON_2fPoint&) const;
  ON_2dPoint  operator+(const ON_2fVector&) const;
  ON_2dVector operator-(const ON_2fPoint&) const;
  ON_2dPoint  operator-(const ON_2fVector&) const;
  ON_3dPoint  operator+(const ON_3fPoint&) const;
  ON_3dPoint  operator+(const ON_3fVector&) const;
  ON_3dVector operator-(const ON_3fPoint&) const;
  ON_3dPoint  operator-(const ON_3fVector&) const;

  double operator*(const ON_2dPoint&) const; // dot product for points acting as vectors
  double operator*(const ON_2dVector&) const; // dot product for points acting as vectors
  double operator*(const ON_4dPoint&) const;
  ON_2dPoint operator*(const ON_Xform&) const;

  bool operator==(const ON_2dPoint&) const;
  bool operator!=(const ON_2dPoint&) const;

  // dictionary order comparisons
  bool operator<=(const ON_2dPoint&) const;
  bool operator>=(const ON_2dPoint&) const;
  bool operator<(const ON_2dPoint&) const;
  bool operator>(const ON_2dPoint&) const;

  // index operators mimic double[2] behavior
  double& operator[](int);
  double operator[](int) const;
  double& operator[](unsigned int);
  double operator[](unsigned int) const;

  /*
  Returns:
    False if any coordinate is infinte, a nan, or ON_UNSET_VALUE.
  */
  bool IsValid() const;

  /*
  Returns:
    True if every coordinate is ON_UNSET_VALUE.
  */
  bool IsUnsetPoint() const;

  // set 2d point value
  void Set(double x,double y);

  double DistanceTo( const ON_2dPoint& ) const;

  int MaximumCoordinateIndex() const;
  double MaximumCoordinate() const; // absolute value of maximum coordinate

  int MinimumCoordinateIndex() const;
  double MinimumCoordinate() const; // absolute value of minimum coordinate

  void Zero(); // set all coordinates to zero;

  // These transform the point in place. The transformation matrix acts on
  // the left of the point; i.e., result = transformation*point
  void Transform( 
        const ON_Xform&
        );

  void Rotate( // rotatation in XY plane
        double angle,              // angle in radians
        const ON_2dPoint& center   // center of rotation
        );

  void Rotate( // rotatation in XY plane
        double sin_angle,          // sin(angle)
        double cos_angle,          // cos(angle)
        const ON_2dPoint& center   // center of rotation
        );
};

ON_DECL
ON_2dPoint operator*(int, const ON_2dPoint&);

ON_DECL
ON_2dPoint operator*(float, const ON_2dPoint&);

ON_DECL
ON_2dPoint operator*(double, const ON_2dPoint&);

////////////////////////////////////////////////////////////////
//
//   ON_3dPoint
//
class PCL_EXPORTS ON_CLASS ON_3dPoint
{
public:
  double x, y, z;

  static const ON_3dPoint Origin;     // (0.0,0.0,0.0)
  static const ON_3dPoint UnsetPoint; // (ON_UNSET_VALUE,ON_UNSET_VALUE,ON_UNSET_VALUE)

  // use implicit destructor, copy constructor
  ON_3dPoint();                         // x,y,z not initialized
  ON_3dPoint(double x,double y,double z);
  ON_3dPoint(const ON_2dPoint& );       // from 2d point
  ON_3dPoint(const ON_4dPoint& );       // from 4d point
  ON_3dPoint(const ON_2dVector& );      // from 2d vector
  ON_3dPoint(const ON_3dVector& );      // from 3d vector
  ON_3dPoint(const double*);            // from double[3] array

  ON_3dPoint(const class ON_2fPoint&);  // from 2f point
  ON_3dPoint(const class ON_3fPoint&);  // from 3f point
  ON_3dPoint(const class ON_4fPoint&);  // from 4f point
  ON_3dPoint(const class ON_2fVector&); // from 2f point
  ON_3dPoint(const class ON_3fVector&); // from 3f point
  ON_3dPoint(const float*);             // from float[3] array

  // (double*) conversion operators
  operator double*();
  operator const double*() const;

  // use implicit operator=(const ON_3dPoint&)
  ON_3dPoint& operator=(const ON_2dPoint&);
  ON_3dPoint& operator=(const ON_4dPoint&);
  ON_3dPoint& operator=(const ON_2dVector&);
  ON_3dPoint& operator=(const ON_3dVector&);
  ON_3dPoint& operator=(const double*); // point = double[3] support

  ON_3dPoint& operator=(const class ON_2fPoint&);
  ON_3dPoint& operator=(const class ON_3fPoint&);
  ON_3dPoint& operator=(const class ON_4fPoint&);
  ON_3dPoint& operator=(const class ON_2fVector&);
  ON_3dPoint& operator=(const class ON_3fVector&);
  ON_3dPoint& operator=(const float*);  // point = float[3] support

  ON_3dPoint& operator*=(double);
  ON_3dPoint& operator/=(double);
  ON_3dPoint& operator+=(const ON_3dPoint&);  // Adding this was a mistake - cannot remove without breaking SDK
  ON_3dPoint& operator+=(const ON_3dVector&);
  ON_3dPoint& operator-=(const ON_3dPoint&);  // Adding this was a mistake - cannot remove without breaking SDK
  ON_3dPoint& operator-=(const ON_3dVector&);

  ON_3dPoint  operator*(int) const;
  ON_3dPoint  operator/(int) const;
  ON_3dPoint  operator*(float) const;
  ON_3dPoint  operator/(float) const;
  ON_3dPoint  operator*(double) const;
  ON_3dPoint  operator/(double) const;

  ON_3dPoint  operator+(const ON_3dPoint&) const;
  ON_3dPoint  operator+(const ON_3dVector&) const;
  ON_3dVector operator-(const ON_3dPoint&) const;
  ON_3dPoint  operator-(const ON_3dVector&) const;
  ON_3dPoint  operator+(const ON_2dPoint&) const;
  ON_3dPoint  operator+(const ON_2dVector&) const;
  ON_3dVector operator-(const ON_2dPoint&) const;
  ON_3dPoint  operator-(const ON_2dVector&) const;

  ON_3dPoint  operator+(const ON_3fPoint&) const;
  ON_3dPoint  operator+(const ON_3fVector&) const;
  ON_3dVector operator-(const ON_3fPoint&) const;
  ON_3dPoint  operator-(const ON_3fVector&) const;
  ON_3dPoint  operator+(const ON_2fPoint&) const;
  ON_3dPoint  operator+(const ON_2fVector&) const;
  ON_3dVector operator-(const ON_2fPoint&) const;
  ON_3dPoint  operator-(const ON_2fVector&) const;

  double operator*(const ON_3dPoint&) const; // dot product for points acting as vectors
  double operator*(const ON_3dVector&) const; // dot product for points acting as vectors
  double operator*(const ON_4dPoint&) const;
  ON_3dPoint operator*(const ON_Xform&) const;

  bool operator==(const ON_3dPoint&) const;
  bool operator!=(const ON_3dPoint&) const;

  // dictionary order comparisons
  bool operator<=(const ON_3dPoint&) const;
  bool operator>=(const ON_3dPoint&) const;
  bool operator<(const ON_3dPoint&) const;
  bool operator>(const ON_3dPoint&) const;

  // index operators mimic double[3] behavior
  double& operator[](int);
  double operator[](int) const;
  double& operator[](unsigned int);
  double operator[](unsigned int) const;

  /*
  Returns:
    False if any coordinate is infinte, a nan, or ON_UNSET_VALUE.
  */
  bool IsValid() const;

  /*
  Returns:
    True if every coordinate is ON_UNSET_VALUE.
  */
  bool IsUnsetPoint() const;

  // set 3d point value
  void Set(double x,double y,double z);

  double DistanceTo( const ON_3dPoint& ) const;

  int MaximumCoordinateIndex() const;
  double MaximumCoordinate() const; // absolute value of maximum coordinate
  
  int MinimumCoordinateIndex() const;
  double MinimumCoordinate() const; // absolute value of minimum coordinate

  double Fuzz( double tolerance = ON_ZERO_TOLERANCE ) const; // tolerance to use when comparing 3d points

  void Zero(); // set all coordinates to zero;

  // These transform the point in place. The transformation matrix acts on
  // the left of the point; i.e., result = transformation*point
  void Transform( 
        const ON_Xform&
        );

  void Rotate( 
        double angle,             // angle in radians
        const ON_3dVector& axis,  // axis of rotation
        const ON_3dPoint& center  // center of rotation
        );

  void Rotate( 
        double sin_angle,         // sin(angle)
        double cos_angle,         // cos(angle)
        const ON_3dVector& axis,  // axis of rotation
        const ON_3dPoint& center  // center of rotation
        );
};

ON_DECL
ON_3dPoint operator*(int, const ON_3dPoint&);

ON_DECL
ON_3dPoint operator*(float, const ON_3dPoint&);

ON_DECL
ON_3dPoint operator*(double, const ON_3dPoint&);

////////////////////////////////////////////////////////////////
//
//   ON_4dPoint (homogeneous coordinates)
//
class PCL_EXPORTS ON_CLASS ON_4dPoint
{
public:
  double x, y, z, w;
  
  // use implicit destructor, copy constructor
  ON_4dPoint();                       // x,y,z,w not initialized
  ON_4dPoint(double x,double y,double z,double w);

  ON_4dPoint(const ON_2dPoint& );     // from 2d point
  ON_4dPoint(const ON_3dPoint& );     // from 3d point
  ON_4dPoint(const ON_2dVector& );    // from 2d vector
  ON_4dPoint(const ON_3dVector& );    // from 3d vector
  ON_4dPoint(const double*);          // from double[4] array

  ON_4dPoint(const ON_2fPoint& );     // from 2f point
  ON_4dPoint(const ON_3fPoint& );     // from 3f point
  ON_4dPoint(const ON_4fPoint& );     // from 3f point
  ON_4dPoint(const ON_2fVector& );    // from 2f vector
  ON_4dPoint(const ON_3fVector& );    // from 3f vector
  ON_4dPoint(const float*);           // from float[4] array

  // (double*) conversion operators
  operator double*();
  operator const double*() const;

  // use implicit operator=(const ON_4dPoint&)
  ON_4dPoint& operator=(const ON_2dPoint&);
  ON_4dPoint& operator=(const ON_3dPoint&);
  ON_4dPoint& operator=(const ON_2dVector&);
  ON_4dPoint& operator=(const ON_3dVector&);
  ON_4dPoint& operator=(const double*); // point = double[4] support

  ON_4dPoint& operator=(const class ON_2fPoint&);
  ON_4dPoint& operator=(const class ON_3fPoint&);
  ON_4dPoint& operator=(const class ON_4fPoint&);
  ON_4dPoint& operator=(const class ON_2fVector&);
  ON_4dPoint& operator=(const class ON_3fVector&);
  ON_4dPoint& operator=(const float*);  // point = float[4] support

  ON_4dPoint& operator*=(double);
  ON_4dPoint& operator/=(double);
  ON_4dPoint& operator+=(const ON_4dPoint&); // sum w = sqrt(|w1*w2|)
  ON_4dPoint& operator-=(const ON_4dPoint&); // difference w = sqrt(|w1*w2|)

  ON_4dPoint  operator*(double) const;
  ON_4dPoint  operator/(double) const;
  ON_4dPoint  operator+(const ON_4dPoint&) const; // sum w = sqrt(|w1*w2|)
  ON_4dPoint  operator-(const ON_4dPoint&) const; // difference w = sqrt(|w1*w2|)

  double operator*(const ON_4dPoint&) const;
  ON_4dPoint operator*(const ON_Xform&) const;

  // projective comparison 
  // (i.e., [x,y,z,w] == [c*x,c*y,c*z,c*w] is true for nonzero c)
  bool operator==(ON_4dPoint) const;
  bool operator!=(const ON_4dPoint&) const;

  // index operators mimic double[4] behavior
  double& operator[](int);
  double operator[](int) const;
  double& operator[](unsigned int);
  double operator[](unsigned int) const;

  /*
  Returns:
    False if any coordinate is infinte, a nan, or ON_UNSET_VALUE.
  */
  bool IsValid() const;

  /*
  Returns:
    True if every coordinate is ON_UNSET_VALUE.
  */
  bool IsUnsetPoint() const;

  // set 4d point value
  void Set(double x,double y,double z,double w);

  int MaximumCoordinateIndex() const;
  double MaximumCoordinate() const; // absolute value of maximum coordinate

  int MinimumCoordinateIndex() const;
  double MinimumCoordinate() const; // absolute value of minimum coordinate

  void Zero();      // set all 4 coordinates to zero;
  bool Normalize(); // set so x^2 + y^2 + z^2 + w^2 = 1

  // These transform the point in place. The transformation matrix acts on
  // the left of the point; i.e., result = transformation*point
  void Transform( 
        const ON_Xform&
        );
};

ON_DECL
ON_4dPoint operator*(double, const ON_4dPoint&);

////////////////////////////////////////////////////////////////
//
//   ON_2dVector
//
class PCL_EXPORTS ON_CLASS ON_2dVector
{
public:
  double x, y;

  static const ON_2dVector ZeroVector;  // (0.0,0.0)
  static const ON_2dVector XAxis;       // (1.0,0.0)
  static const ON_2dVector YAxis;       // (0.0,1.0)
  static const ON_2dVector UnsetVector; // (ON_UNSET_VALUE,ON_UNSET_VALUE)

  // Description:
  //   A index driven function to get unit axis vectors.
  // Parameters:
  //   index - [in] 0 returns (1,0), 1 returns (0,1)
  // Returns:
  //   Unit 2d vector with vector[i] = (i==index)?1:0;
  static const ON_2dVector& UnitVector(
    int // index
    );

  // use implicit destructor, copy constructor
  ON_2dVector();                     // x,y not initialized
  ON_2dVector(double x,double y);

  ON_2dVector(const ON_3dVector& ); // from 3d vector
  ON_2dVector(const ON_2dPoint& );  // from 2d point
  ON_2dVector(const ON_3dPoint& );  // from 3d point
  ON_2dVector(const double*);       // from double[2] array

  ON_2dVector(const ON_2fVector& ); // from 2f vector
  ON_2dVector(const ON_3fVector& ); // from 3f vector
  ON_2dVector(const ON_2fPoint& );  // from 2f point
  ON_2dVector(const ON_3fPoint& );  // from 3f point
  ON_2dVector(const float*);        // from double[2] array

  // (double*) conversion operators
  operator double*();
  operator const double*() const;

  // use implicit operator=(const ON_2dVector&)
  ON_2dVector& operator=(const ON_3dVector&);
  ON_2dVector& operator=(const ON_2dPoint&);
  ON_2dVector& operator=(const ON_3dPoint&);
  ON_2dVector& operator=(const double*); // vector = double[2] support

  ON_2dVector& operator=(const ON_2fVector&);
  ON_2dVector& operator=(const ON_3fVector&);
  ON_2dVector& operator=(const ON_2fPoint&);
  ON_2dVector& operator=(const ON_3fPoint&);
  ON_2dVector& operator=(const float*);  // vector = float[2] support

  ON_2dVector  operator-() const;

  ON_2dVector& operator*=(double);
  ON_2dVector& operator/=(double);
  ON_2dVector& operator+=(const ON_2dVector&);
  ON_2dVector& operator-=(const ON_2dVector&);
  // DO NOT ADD ANY MORE overrides of += or -=

  double operator*(const ON_2dVector&) const; // inner (dot) product
  double operator*(const ON_2dPoint&) const; // inner (dot) product (point acting as vector)
  double operator*(const ON_2fVector&) const; // inner (dot) product	

  ON_2dVector  operator*(int) const;
  ON_2dVector  operator/(int) const;
  ON_2dVector  operator*(float) const;
  ON_2dVector  operator/(float) const;
  ON_2dVector  operator*(double) const;
  ON_2dVector  operator/(double) const;

  ON_2dVector  operator+(const ON_2dVector&) const;
  ON_2dPoint   operator+(const ON_2dPoint&) const;
  ON_2dVector  operator-(const ON_2dVector&) const;
  ON_2dPoint   operator-(const ON_2dPoint&) const;
  ON_3dVector  operator+(const ON_3dVector&) const;
  ON_3dPoint   operator+(const ON_3dPoint&) const;
  ON_3dVector  operator-(const ON_3dVector&) const;
  ON_3dPoint   operator-(const ON_3dPoint&) const;

  ON_2dVector  operator+(const ON_2fVector&) const;
  ON_2dPoint   operator+(const ON_2fPoint&) const;
  ON_2dVector  operator-(const ON_2fVector&) const;
  ON_2dPoint   operator-(const ON_2fPoint&) const;
  ON_3dVector  operator+(const ON_3fVector&) const;
  ON_3dPoint   operator+(const ON_3fPoint&) const;
  ON_3dVector  operator-(const ON_3fVector&) const;
  ON_3dPoint   operator-(const ON_3fPoint&) const;

  double operator*(const ON_4dPoint&) const;
  ON_2dVector operator*(const ON_Xform&) const;

  bool operator==(const ON_2dVector&) const;
  bool operator!=(const ON_2dVector&) const;

  // dictionary order comparisons
  bool operator<=(const ON_2dVector&) const;
  bool operator>=(const ON_2dVector&) const;
  bool operator<(const ON_2dVector&) const;
  bool operator>(const ON_2dVector&) const;

  // index operators mimic double[2] behavior
  double& operator[](int);
  double operator[](int) const;
  double& operator[](unsigned int);
  double operator[](unsigned int) const;

  /*
  Returns:
    False if any coordinate is infinte, a nan, or ON_UNSET_VALUE.
  */
  bool IsValid() const;

  /*
  Returns:
    True if every coordinate is ON_UNSET_VALUE.
  */
  bool IsUnsetVector() const;

  // set 2d vector value
  void Set(double x,double y);

  int MaximumCoordinateIndex() const;
  double MaximumCoordinate() const; // absolute value of maximum coordinate

  int MinimumCoordinateIndex() const;
  double MinimumCoordinate() const; // absolute value of minimum coordinate

  double LengthSquared() const;
  double Length() const;

	// Signed area of the parallelagram.  The volume element.
	// returns x*B.y - y*B.x
	double WedgeProduct(const ON_2dVector& B) const;

  bool Decompose( // Computes a, b such that this vector = a*X + b*Y
         // Returns false if unable to solve for a,b.  This happens
         // when X,Y is not really a basis.
         //
         // If X,Y is known to be an orthonormal frame,
         // then a = V*X, b = V*Y will compute
         // the same result more quickly.
         const ON_2dVector&, // X
         const ON_2dVector&, // Y
         double*, // a
         double*  // b
         ) const;

  int IsParallelTo( 
        // returns  1: this and other vectors are parallel
        //         -1: this and other vectors are anti-parallel
        //          0: this and other vectors are not parallel
        //             or at least one of the vectors is zero
        const ON_2dVector& other,                           // other vector     
        double angle_tolerance = ON_DEFAULT_ANGLE_TOLERANCE // optional angle tolerance (radians)
        ) const;

  bool IsPerpendicularTo(
        // returns true:  this and other vectors are perpendicular
        //         false: this and other vectors are not perpendicular
        //                or at least one of the vectors is zero
        const ON_2dVector& other,                           // other vector     
        double angle_tolerance = ON_DEFAULT_ANGLE_TOLERANCE // optional angle tolerance (radians)
        ) const;

  void Zero(); // set all coordinates to zero;
  void Reverse(); // negate all coordinates
  bool Unitize();  // returns false if vector has zero length

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
         double tiny_tol = ON_ZERO_TOLERANCE // tiny_tol
         ) const;

  // Returns:
  //   true if vector is the zero vector.
  bool IsZero() const;

  // Returns:
  //   true if vector is valid and has length 1.
  bool IsUnitVector() const;

  // set this vector to be perpendicular to another vector
  bool PerpendicularTo( // Result is not unitized. 
                        // returns false if input vector is zero
        const ON_2dVector& 
        );

  // set this vector to be perpendicular to a line defined by 2 points
  bool PerpendicularTo( 
        const ON_2dPoint&, 
        const ON_2dPoint& 
        );

  // These transform the vector in place. The transformation matrix acts on
  // the left of the vector; i.e., result = transformation*vector
  void Transform( 
        const ON_Xform& // can use ON_Xform here
        );

  void Rotate( 
        double angle            // angle in radians
        );

  void Rotate( 
        double sin_angle,       // sin(angle)
        double cos_angle        // cos(angle)
        );
};

ON_DECL
ON_2dVector operator*(int, const ON_2dVector&);

ON_DECL
ON_2dVector operator*(float, const ON_2dVector&);

ON_DECL
ON_2dVector operator*(double, const ON_2dVector&);

///////////////////////////////////////////////////////////////
//
// ON_2dVector utilities
//

ON_DECL
double 
ON_DotProduct( 
    const ON_2dVector&, 
    const ON_2dVector& 
    );

ON_DECL
ON_3dVector 
ON_CrossProduct(
    const ON_2dVector&, 
    const ON_2dVector& 
    );

ON_DECL
double			 
ON_WedgeProduct(		// signed area of the parallelagram.  Volume element.
    const ON_2dVector& A, // returns A.x * B.y - A.y * B.x 
    const ON_2dVector& B 
    );

ON_DECL
bool 
ON_IsOrthogonalFrame( // true if X, Y are nonzero and mutually perpendicular
    const ON_2dVector&, // X
    const ON_2dVector&  // Y
    );

ON_DECL
bool 
ON_IsOrthonormalFrame( // true if X, Y are orthogonal and unit length
    const ON_2dVector&, // X
    const ON_2dVector&  // Y
    );

ON_DECL
bool 
ON_IsRightHandFrame( // true if X, Y are orthonormal and right handed
    const ON_2dVector&, // X
    const ON_2dVector&  // Y
    );

////////////////////////////////////////////////////////////////
//
//   ON_3dVector
//
class PCL_EXPORTS ON_CLASS ON_3dVector
{
public:
  double x, y, z;

  static const ON_3dVector ZeroVector;  // (0.0,0.0,0.0)
  static const ON_3dVector XAxis;       // (1.0,0.0,0.0)
  static const ON_3dVector YAxis;       // (0.0,1.0,0.0)
  static const ON_3dVector ZAxis;       // (0.0,0.0,1.0)
  static const ON_3dVector UnsetVector; // (ON_UNSET_VALUE,ON_UNSET_VALUE,ON_UNSET_VALUE)

  // Description:
  //   A index driven function to get unit axis vectors.
  // Parameters:
  //   index - [in] 0 returns (1,0,0), 1 returns (0,1,0), 
  //                2 returns (0,0,1)
  // Returns:
  //   Unit 3d vector with vector[i] = (i==index)?1:0;
  static const ON_3dVector& UnitVector(
    int // index
    );

  // use implicit destructor, copy constructor
  ON_3dVector();                     // x,y,z not initialized
  ON_3dVector(double x,double y,double z);
  ON_3dVector(const ON_2dVector& );  // from 2d vector
  ON_3dVector(const ON_2dPoint& );   // from 2d point
  ON_3dVector(const ON_3dPoint& );   // from 3d point
  ON_3dVector(const double*);        // from double[3] array

  ON_3dVector(const ON_2fVector& );  // from 2f vector
  ON_3dVector(const ON_3fVector& );  // from 3f vector
  ON_3dVector(const ON_2fPoint& );   // from 2f point
  ON_3dVector(const ON_3fPoint& );   // from 3f point
  ON_3dVector(const float*);         // from float[3] array

  // (double*) conversion operators
  operator double*();
  operator const double*() const;

  // use implicit operator=(const ON_3dVector&)
  ON_3dVector& operator=(const ON_2dVector&);
  ON_3dVector& operator=(const ON_2dPoint&);
  ON_3dVector& operator=(const ON_3dPoint&);
  ON_3dVector& operator=(const double*); // vector = double[3] support
  
  ON_3dVector& operator=(const ON_2fVector&);
  ON_3dVector& operator=(const ON_3fVector&);
  ON_3dVector& operator=(const ON_2fPoint&);
  ON_3dVector& operator=(const ON_3fPoint&);
  ON_3dVector& operator=(const float*);  // vector = float[3] support

  ON_3dVector  operator-() const;

  ON_3dVector& operator*=(double);
  ON_3dVector& operator/=(double);
  ON_3dVector& operator+=(const ON_3dVector&);
  ON_3dVector& operator-=(const ON_3dVector&);
  // DO NOT ADD ANY MORE overrides of += or -=

  double operator*(const ON_3dVector&) const; // inner (dot) product
  double operator*(const ON_3dPoint&) const; // inner (dot) product
  double operator*(const ON_3fVector&) const; // inner (dot) product

  ON_3dVector  operator*(int) const;
  ON_3dVector  operator/(int) const;
  ON_3dVector  operator*(float) const;
  ON_3dVector  operator/(float) const;
  ON_3dVector  operator*(double) const;
  ON_3dVector  operator/(double) const;

  ON_3dVector  operator+(const ON_3dVector&) const;
  ON_3dPoint   operator+(const ON_3dPoint&) const;
  ON_3dVector  operator-(const ON_3dVector&) const;
  ON_3dPoint   operator-(const ON_3dPoint&) const;
  ON_3dVector  operator+(const ON_2dVector&) const;
  ON_3dPoint   operator+(const ON_2dPoint&) const;
  ON_3dVector  operator-(const ON_2dVector&) const;
  ON_3dPoint   operator-(const ON_2dPoint&) const;

  ON_3dVector  operator+(const ON_3fVector&) const;
  ON_3dPoint   operator+(const ON_3fPoint&) const;
  ON_3dVector  operator-(const ON_3fVector&) const;
  ON_3dPoint   operator-(const ON_3fPoint&) const;
  ON_3dVector  operator+(const ON_2fVector&) const;
  ON_3dPoint   operator+(const ON_2fPoint&) const;
  ON_3dVector  operator-(const ON_2fVector&) const;
  ON_3dPoint   operator-(const ON_2fPoint&) const;

  double operator*(const ON_4dPoint&) const;
  ON_3dVector operator*(const ON_Xform&) const;

  bool operator==(const ON_3dVector&) const;
  bool operator!=(const ON_3dVector&) const;

  // dictionary order comparisons
  bool operator<=(const ON_3dVector&) const;
  bool operator>=(const ON_3dVector&) const;
  bool operator<(const ON_3dVector&) const;
  bool operator>(const ON_3dVector&) const;

  // index operators mimic double[3] behavior
  double& operator[](int);
  double operator[](int) const;
  double& operator[](unsigned int);
  double operator[](unsigned int) const;

  /*
  Returns:
    False if any coordinate is infinte, a nan, or ON_UNSET_VALUE.
  */
  bool IsValid() const;

  /*
  Returns:
    True if every coordinate is ON_UNSET_VALUE.
  */
  bool IsUnsetVector() const;

  // set 3d vector value
  void Set(double x,double y,double z);

  int MaximumCoordinateIndex() const;
  double MaximumCoordinate() const; // absolute value of maximum coordinate

  int MinimumCoordinateIndex() const;
  double MinimumCoordinate() const; // absolute value of minimum coordinate

  double LengthSquared() const;
  double Length() const;

  bool Decompose( // Computes a, b, c such that this vector = a*X + b*Y + c*Z
         // Returns false if unable to solve for a,b,c.  This happens
         // when X,Y,Z is not really a basis.
         //
         // If X,Y,Z is known to be an orthonormal frame,
         // then a = V*X, b = V*Y, c = V*Z will compute
         // the same result more quickly.
         const ON_3dVector&, // X
         const ON_3dVector&, // Y
         const ON_3dVector&, // Z
         double*, // a
         double*, // b
         double*  // c
         ) const;

  int IsParallelTo( 
        // returns  1: this and other vectors are parallel
        //         -1: this and other vectors are anti-parallel
        //          0: this and other vectors are not parallel
        //             or at least one of the vectors is zero
        const ON_3dVector& other,                           // other vector     
        double angle_tolerance = ON_DEFAULT_ANGLE_TOLERANCE // optional angle tolerance (radians)
        ) const;

  bool IsPerpendicularTo(
        // returns true:  this and other vectors are perpendicular
        //         false: this and other vectors are not perpendicular
        //                or at least one of the vectors is zero
        const ON_3dVector& other,                           // other vector     
        double angle_tolerance = ON_DEFAULT_ANGLE_TOLERANCE // optional angle tolerance (radians)
        ) const;

  double Fuzz( double tolerance = ON_ZERO_TOLERANCE ) const; // tolerance to use when comparing 3d vectors

  void Zero(); // set all coordinates to zero;
  void Reverse(); // negate all coordinates
  bool Unitize();  // returns false if vector has zero length
  double LengthAndUnitize(); // unitizes and returns initial length

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
         double tiny_tol = ON_ZERO_TOLERANCE // tiny_tol
         ) const;

  // Returns:
  //   true if vector is the zero vector.
  bool IsZero() const;

  // Returns:
  //   true if vector is valid and has length 1.
  bool IsUnitVector() const;

  // set this vector to be perpendicular to another vector
  bool PerpendicularTo( // Result is not unitized. 
                        // returns false if input vector is zero
        const ON_3dVector& 
        );

  // set this vector to be perpendicular to a plane defined by 3 points
  bool PerpendicularTo(
               // about 3 times slower than
               //    ON_3dVector N = ON_CrossProduct(P1-P0,P2-P0); 
               //    N.Unitize();
               // returns false if points are coincident or colinear
         const ON_3dPoint&, const ON_3dPoint&, const ON_3dPoint& 
         );

  // These transform the vector in place. The transformation matrix acts on
  // the left of the vector; i.e., result = transformation*vector
  void Transform( 
        const ON_Xform& // can use ON_Xform here
        );

  void Rotate( 
        double angle,           // angle in radians
        const ON_3dVector& axis // axis of rotation
        );

  void Rotate( 
        double sin_angle,        // sin(angle)
        double cos_angle,        // cos(angle)
        const ON_3dVector& axis  // axis of rotation
        );
};

class PCL_EXPORTS ON_CLASS ON_3dRay
{
public:
  ON_3dRay();
  ~ON_3dRay();

  ON_3dPoint  m_P;
  ON_3dVector m_V;
};

/*
Description:
  Typically the vector portion is a unit vector and
  m_d = -(x*P.x + y*P.y + z*P.z) for a point P on the plane.
*/
class PCL_EXPORTS ON_CLASS ON_PlaneEquation : public ON_3dVector
{
public:
  // C++ defaults for construction, destruction, copys, and operator=
  // work fine.

  static const ON_PlaneEquation UnsetPlaneEquation; // (ON_UNSET_VALUE,ON_UNSET_VALUE,ON_UNSET_VALUE,ON_UNSET_VALUE)
  static const ON_PlaneEquation ZeroPlaneEquation; // (0.0,0.0,0.0,0.0)

  ON_PlaneEquation();

  ON_PlaneEquation(double xx, double yy, double zz, double dd);

  /*
  Description:
    returns true if x, y, z, d are valid, finite doubles.
  Remarks:
    this function will return true if x, y and z are all zero.
  See Also:
    ON_PlaneEquation::IsSet().
  */
  bool IsValid() const;

  /*
  Description:
    returns true if x, y, z, d are valid, finite doubles and
    at least one of x, y or z is not zero.
  */
  bool IsSet() const;

  /*
  Description:
    Sets (x,y,z) to a unitized N and then sets
    d = -(x*P.x + y*P.y + z*P.z).
  Parameters:
    P - [in] point on the plane
    N - [in] vector perpendicular to the plane
  Returns:
     true if input is valid.
  */
  bool Create( ON_3dPoint P, ON_3dVector N );

  /*
  Description:
    Evaluate the plane at a point.
  Parameters:
    P - [in]
  Returns:
    x*P.x + y*P.y + z*P.z + d;
  */
  double ValueAt(ON_3dPoint P) const;
  double ValueAt(ON_4dPoint P) const;
  double ValueAt(ON_3dVector P) const;
  double ValueAt(double x, double y, double z) const;

  /*
  Description:
    Evaluate the plane at a list of point values.
  Parameters:
    Pcount - [in]
      number of points
    P - [in]
      points
    value - [in]
      If not null, value[] must be an array of length at least Pcount.
      The values will be stored in this array.  If null, the an array
      will be allocated with onmalloc() and returned.
    value_range - [out]
      If not null, the range of values will be returned here.
  Returns:
    An array of Pcount values.  If the input parameter value was null,
    then the array is allocated on the heap using onmalloc() and the 
    caller is responsible for calling onfree() when finished.  If the 
    input is not valid, null is returned.
  */
  double* ValueAt(
        int Pcount,
        const ON_3fPoint* P,
        double* value,
        double value_range[2]
        ) const;

  double* ValueAt(
        int Pcount,
        const ON_3dPoint* P,
        double* value,
        double value_range[2]
        ) const;

  /*
  Description:
    This function calculates and evalutes points that 
    would be exactly on the plane if double precision
    aritmetic were mathematically perfect and returns
    the largest value of the evaluations.
  */
  double ZeroTolerance() const;

  /*
  Description:
    Transform the plane equation so that, if e0 is the initial
    equation, e1 is transformed equation and P is a point,
    then e0.ValueAt(P) = e1.ValueAt(xform*P).
  Parameters:
    xform - [in]
      Invertable transformation.
  Returns:
    True if the plane equation was successfully transformed.
    False if xform is not invertable or the equation is not
    valid.
  Remarks:
    This function has to invert xform.  If you have apply the
    same transformation to a bunch of planes, then it will be
    more efficient to calculate xform's inverse transpose
    and apply the resultingt transformation to the equation's
    coefficients as if they were 4d point coordinates.
  */
  bool Transform( const ON_Xform& xform );

  /*
  Description:
    Get point on plane that is closest to a given point.
  Parameters:
    point - [in]
  Returns:
    A 3d point on the plane that is closest to the input point.
  */
  ON_3dPoint ClosestPointTo( ON_3dPoint point ) const;

  /*
  Description:
    Get the minimum value of the plane equation
    on a bounding box.
  Parameters:
    bbox - [in] 
  Returns:
    Minimum value of the plane equation on the bounding box.
  */
  double MinimumValueAt(const ON_BoundingBox& bbox) const;

  /*
  Description:
    Get the maximum value of the plane equation
    on a bounding box.
  Parameters:
    bbox - [in] 
  Returns:
    Maximum value of the plane equation on the bounding box.
  */
  double MaximumValueAt(const ON_BoundingBox& bbox) const;

  /*
  Description:
    Get the maximum value of the plane equation on a set of 3d points.
  Parameters:
    bRational - [in]
      False if the points are euclidean (x,y,z)
      True if the points are homogenous rational (x,y,z,w)
      (x/w,y/w,z/w) is used to evaluate the value.
    point_count - [in]
    point_stride - [in]
      i-th point's x coordinate = points[i*point_stride]
    points - [in]
      coordinates of points
    stop_value - [in]
      If stop_value is valid and not ON_UNSET_VALUE, then the 
      evaulation stops if a value > stop_value is found. 
      If stop_value = ON_UNSET_VALUE, then stop_value is ignored.
  Returns:
    Maximum value of the plane equation on the point list.
    If the input is not valid, then ON_UNSET_VALUE is returned.
  */
  double MaximumValueAt(
    bool bRational,
    int point_count,
    int point_stride,
    const double* points,
    double stop_value
    ) const;

  /*
  Description:
    Get the minimum value of the plane equation on a set of 3d points.
  Parameters:
    bRational - [in]
      False if the points are euclidean (x,y,z)
      True if the points are homogenous rational (x,y,z,w)
      (x/w,y/w,z/w) is used to evaluate the value.
    point_count - [in]
    point_stride - [in]
      i-th point's x coordinate = points[i*point_stride]
    points - [in]
      coordinates of points
    stop_value - [in]
      If stop_value is valid and not ON_UNSET_VALUE, then the 
      evaulation stops if a value < stop_value is found. 
      If stop_value = ON_UNSET_VALUE, then stop_value is ignored.
  Returns:
    Maximum value of the plane equation on the point list.
    If the input is not valid, then ON_UNSET_VALUE is returned.
  */
  double MinimumValueAt(
    bool bRational,
    int point_count,
    int point_stride,
    const double* points,
    double stop_value
    ) const;

  /*
  Description:
    Get the maximum absolute value of the plane equation 
    on a set of 3d points.
  Parameters:
    bRational - [in]
      False if the points are euclidean (x,y,z)
      True if the points are homogenous rational (x,y,z,w)
      (x/w,y/w,z/w) is used to evaluate the value.
    point_count - [in]
    point_stride - [in]
      i-th point's x coordinate = points[i*point_stride]
    points - [in]
      coordinates of points
    stop_value - [in]
      If stop_value >= 0.0, then the evaulation stops if an
      absolute value > stop_value is found. If stop_value < 0.0 
      or stop_value is invalid, then stop_value is ignored.
  Returns:
    Maximum value of the plane equation on the point list.
    If the input is not valid, then ON_UNSET_VALUE is returned.
  */
  double MaximumAbsoluteValueAt(
    bool bRational,
    int point_count,
    int point_stride,
    const double* points,
    double stop_value
    ) const;

  /*
  Description:
    Test points on a bezier curve to see if they are near the plane.
  Parameters:
    bezcrv - [in]
    s0 - [in]
    s1 - [in] the interval from s0 to s1 is tested (s0 < s1)
    sample_count - [in] number of interior points to test.  
                Numbers like 1, 3, 7, 15, ... work best.
    endpoint_tolerance - [in] If >= 0, then the end points are 
              tested to see if the distance from the endpoints 
              is <= endpoint_tolerance.
    interior_tolerance - [in] (>=0 and >=endpoint_tolerance) 
              This tolerance is used to test the interior sample points.
    smin - [put]  If not NULL, *smin = bezier parameter of nearest
                  test point.
    smax - [put]  If not NULL, *smax = bezier parameter of farthest
                  test point.  If false is returned, this is the
                  parameter of the test point that failed.
  Returns:
    True if all the tested points passed the tolerance test.
    False if at least one tested point failed the tolerance test.
    (The test terminates when the first failure is encountered.)
  */
  bool IsNearerThan( 
          const class ON_BezierCurve& bezcrv,
          double s0,
          double s1,
          int sample_count,
          double endpoint_tolerance,
          double interior_tolerance,
          double* smin,
          double* smax
          ) const;
  
  bool operator==(const ON_PlaneEquation&) const;
  bool operator!=(const ON_PlaneEquation&) const;

  double d; // 4th coefficient of the plane equation.
};

ON_DECL
ON_3dVector operator*(int, const ON_3dVector&);

ON_DECL
ON_3dVector operator*(float, const ON_3dVector&);

ON_DECL
ON_3dVector operator*(double, const ON_3dVector&);

///////////////////////////////////////////////////////////////
//
// ON_3dVector utilities
//

ON_DECL
double 
ON_DotProduct( 
    const ON_3dVector&, 
    const ON_3dVector& 
    );


ON_DECL
ON_3dVector 
ON_CrossProduct(
    const ON_3dVector&, 
    const ON_3dVector& 
    );

ON_DECL
ON_3dVector 
ON_CrossProduct( // 3d cross product for old fashioned arrays
    const double*, // array of 3d doubles
    const double*  // array of 3d doubles
    );

ON_DECL
double 
ON_TripleProduct( 
    const ON_3dVector&,
    const ON_3dVector&,
    const ON_3dVector&
    );

ON_DECL
double 
ON_TripleProduct(  // 3d triple product for old fashioned arrays
    const double*, // array of 3d doubles
    const double*, // array of 3d doubles
    const double*  // array of 3d doubles
    );

ON_DECL
bool 
ON_IsOrthogonalFrame( // true if X, Y, Z are nonzero and mutually perpendicular
    const ON_3dVector&, // X
    const ON_3dVector&, // Y
    const ON_3dVector&  // Z 
    );

ON_DECL
bool 
ON_IsOrthonormalFrame( // true if X, Y, Z are orthogonal and unit length
    const ON_3dVector&, // X
    const ON_3dVector&, // Y
    const ON_3dVector&  // Z 
    );

ON_DECL
bool 
ON_IsRightHandFrame( // true if X, Y, Z are orthonormal and right handed
    const ON_3dVector&, // X
    const ON_3dVector&, // Y
    const ON_3dVector&  // Z 
    );

///////////////////////////////////////////////////////////////
//
// common points and vectors
//
// ON_unset_point is obsolete - use ON_3dPoint::UnsetPoint
#define ON_unset_point ON_UNSET_POINT

// ON_UNSET_POINT is OBSOLETE - use ON_3dPoint::UnsetPoint
extern ON_EXTERN_DECL const ON_3dPoint  ON_UNSET_POINT; // (ON_UNSET_VALUE,ON_UNSET_VALUE,ON_UNSET_VALUE)

// ON_UNSET_VECTOR is OBSOLETE - use ON_3dPoint::UnsetVector
extern ON_EXTERN_DECL const ON_3dVector ON_UNSET_VECTOR; // (ON_UNSET_VALUE,ON_UNSET_VALUE,ON_UNSET_VALUE)

// ON_origin is OBSOLETE - use ON_3dPoint::Origin
extern ON_EXTERN_DECL const ON_3dPoint  ON_origin; // (0.0, 0.0, 0.0)

// ON_xaxis is OBSOLETE - use ON_3dPoint::XAxis
extern ON_EXTERN_DECL const ON_3dVector ON_xaxis; // (1.0, 0.0, 0.0)

// ON_yaxis is OBSOLETE - use ON_3dPoint::YAxis
extern ON_EXTERN_DECL const ON_3dVector ON_yaxis; // (0.0, 1.0, 0.0)

// ON_zaxis is OBSOLETE - use ON_3dPoint::ZAxis
extern ON_EXTERN_DECL const ON_3dVector ON_zaxis; // (0.0, 0.0, 1.0)

#include "opennurbs_fpoint.h"

////////////////////////////////////////////////////////////////
//
//   ON_SurfaceCurvature
//
class PCL_EXPORTS ON_CLASS ON_SurfaceCurvature
{
public:
  double k1, k2; // principal curvatures

  double GaussianCurvature() const;
  double MeanCurvature() const;
  double MinimumRadius() const;
  double MaximumRadius() const;
};

#endif

