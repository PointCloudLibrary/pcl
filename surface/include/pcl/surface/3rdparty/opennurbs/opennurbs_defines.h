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
//   Includes all openNURBS toolkit defines and enums.
//
////////////////////////////////////////////////////////////////

#if !defined(OPENNURBS_DEFINES_INC_)
#define OPENNURBS_DEFINES_INC_

#include <pcl/pcl_exports.h>

#if defined (cplusplus) || defined(_cplusplus) || defined(__cplusplus) || defined(ON_CPLUSPLUS)
// C++ extern "C" declaration for C linkage

#if !defined(ON_CPLUSPLUS)
#define ON_CPLUSPLUS
#endif
#define ON_EXTERNC extern "C"
#define ON_BEGIN_EXTERNC extern "C" {
#define ON_END_EXTERNC   }

#else

/* C file - no extern declaration required or permitted */

#define ON_EXTERNC
#define ON_BEGIN_EXTERNC
#define ON_END_EXTERNC  

#endif


#if defined(_DEBUG)
/* enable OpenNurbs debugging code */
#if !defined(ON_DEBUG)
#define ON_DEBUG
#endif
#endif

/*
// Declarations in header (.H) files look like
//
//   ON_DECL type function():
//   extern ON_EXTERN_DECL type global_variable;
//   class ON_CLASS classname {};
//   ON_TEMPLATE template class ON_CLASS template<T>;
//
*/

#if defined(OPENNURBS_EXPORTS)
// OPENNURBS_EXPORTS is Microsoft's prefered defined for building an opennurbs DLL.
#if !defined(ON_DLL_EXPORTS)
#define ON_DLL_EXPORTS
#endif
#if !defined(ON_COMPILING_OPENNURBS)
#define ON_COMPILING_OPENNURBS
#endif
#endif

#if defined(OPENNURBS_IMPORTS)
// OPENNURBS_EXPORTS is Microsoft's prefered defined for linking with an opennurbs DLL.
#if !defined(ON_DLL_IMPORTS)
#define ON_DLL_IMPORTS
#endif
#endif

#if defined(ON_DLL_EXPORTS) && defined(ON_DLL_IMPORTS)
#error At most one of ON_DLL_EXPORTS and ON_DLL_IMPORTS can be defined.
#endif

/* export/import */
#if defined(ON_DLL_EXPORTS)

#if !defined(ON_COMPILING_OPENNURBS)
#error When compiling an OpenNURBS DLL, ON_DLL_EXPORTS must be defined
#endif

/* compiling OpenNurbs as a Windows DLL - export classes, functions, templates, and globals */
#define ON_CLASS __declspec(dllexport)
#define ON_DECL __declspec(dllexport)
#define ON_EXTERN_DECL __declspec(dllexport)
#define ON_DLL_TEMPLATE

#elif defined(ON_DLL_IMPORTS)

#if defined(ON_COMPILING_OPENNURBS)
#error When compiling an OpenNURBS DLL, ON_DLL_IMPORTS must NOT be defined
#endif

/* using OpenNurbs as a Windows DLL - import classes, functions, templates, and globals */
#define ON_CLASS __declspec(dllimport)
#define ON_DECL __declspec(dllimport)
#define ON_EXTERN_DECL __declspec(dllimport)
#define ON_DLL_TEMPLATE extern

#else

/* compiling or using OpenNurbs as a static library */
#define ON_CLASS
#define ON_DECL
#define ON_EXTERN_DECL

#if defined(ON_DLL_TEMPLATE)
#undef ON_DLL_TEMPLATE
#endif

#endif


// ON_DEPRECATED is used to mark deprecated functions.
#if defined(ON_COMPILER_MSC)
#define ON_DEPRECATED  __declspec(deprecated)
#else
#define ON_DEPRECATED
#endif

#if defined(PI)
#define ON_PI           PI
#else
#define ON_PI           3.141592653589793238462643
#endif

#define ON_DEGREES_TO_RADIANS ON_PI/180.0
#define ON_RADIANS_TO_DEGREES 180.0/ON_PI

#define ON_SQRT2          1.414213562373095048801689
#define ON_SQRT3          1.732050807568877293527446
#define ON_SQRT3_OVER_2   0.8660254037844386467637230
#define ON_1_OVER_SQRT2   0.7071067811865475244008445
#define ON_SIN_PI_OVER_12 0.2588190451025207623488990
#define ON_COS_PI_OVER_12 0.9659258262890682867497433

#define ON_LOG2         0.6931471805599453094172321
#define ON_LOG10        2.302585092994045684017991

#define ON_ArrayCount(a) (sizeof(a)/sizeof((a)[0]))

#if defined(DBL_MAX)
#define ON_DBL_MAX DBL_MAX
#else
#define ON_DBL_MAX 1.7976931348623158e+308
#endif

#if defined(DBL_MIN)
#define ON_DBL_MIN DBL_MIN
#else
#define ON_DBL_MIN 2.22507385850720200e-308
#endif

// ON_EPSILON = 2^-52
#if defined(DBL_EPSILON)
#define ON_EPSILON DBL_EPSILON
#else
#define ON_EPSILON 2.2204460492503131e-16
#endif
#define ON_SQRT_EPSILON 1.490116119385000000e-8

#if defined(FLT_EPSILON)
#define ON_FLOAT_EPSILON FLT_EPSILON
#else
#define ON_FLOAT_EPSILON 1.192092896e-07
#endif
#define ON_SQRT_FLOAT_EPSILON 3.452669830725202719e-4

/*
// In cases where lazy evaluation of a double value is
// performed, b-rep tolerances being a notable example,
// this value is used to indicate the value has not been
// computed.  This value must be < -1.0e308. and > -ON_DBL_MAX
//
// The reasons ON_UNSET_VALUE is a valid finite number are:
//
//   1) It needs to round trip through fprintf/sscanf.
//   2) It needs to persist unchanged through assigment
/       and not generate exceptions when assigned.
//   3) ON_UNSET_VALUE == ON_UNSET_VALUE needs to be true.
//   4) ON_UNSET_VALUE != ON_UNSET_VALUE needs to be false.
//
// Ideally, it would also have these SNaN attributes
//   * When used in a calculation, a floating point exception
//     occures.
//   * No possibility of a valid calculation would generate
//     ON_UNSET_VALUE.
//   * float f = (float)ON_UNSET_VALUE would create an invalid
//     float and generate an exception.
*/
#define ON_UNSET_VALUE -1.23432101234321e+308

/*
// ON_UNSET_FLOAT is used to indicate a texture coordinate
// value cannot be calculated or is not well defined.  
// In hindsight, this value should have been ON_FLT_QNAN
// because many calculation convert float texture coordinates
// to doubles and the "unset"ness attribute is lost.
*/
#define ON_UNSET_FLOAT -1.234321e+38f


ON_BEGIN_EXTERNC

// IEEE 754 special values
extern ON_EXTERN_DECL const double ON_DBL_QNAN;
extern ON_EXTERN_DECL const double ON_DBL_PINF;
extern ON_EXTERN_DECL const double ON_DBL_NINF;

extern ON_EXTERN_DECL const float  ON_FLT_QNAN;
extern ON_EXTERN_DECL const float  ON_FLT_PINF;
extern ON_EXTERN_DECL const float  ON_FLT_NINF;

/*
Description:
Paramters:
  x - [out] returned value of x is an SNan
            (signalling not a number).
Remarks:
  Any time an SNaN passes through an Intel FPU, the result
  is a QNaN (quiet nan) and the invalid operation excpetion
  flag is set.  If this exception is not masked, then the
  exception handler is invoked.
 
    double x, y;
    ON_DBL_SNAN(&x);
    y = x;     // y = QNAN and invalid op exception occurs
    z = sin(x) // z = QNAN and invalid op exception occurs

  So, if you want to reliably initialize doubles to SNaNs, 
  you must use memcpy() or some other method that does not
  use the Intel FPU.
*/
ON_DECL
void ON_DBL_SNAN( double* x );

ON_DECL
void ON_FLT_SNAN( float* x );

ON_END_EXTERNC

/*
// In cases where lazy evaluation of a color value is
// performed, this value is used to indicate the value 
// has not been computed.
*/
#define ON_UNSET_COLOR 0xFFFFFFFF

/*
// In cases when an absolute "zero" tolerance 
// is required to compare model space coordinates,
// ON_ZERO_TOLERANCE is used.  The value of
// ON_ZERO_TOLERANCE should be no smaller than
// ON_EPSILON and should be several orders of 
// magnitude smaller than ON_SQRT_EPSILON
// 
*/
//#define ON_ZERO_TOLERANCE 1.0e-12
// ON_ZERO_TOLERANCE = 2^-32
#define ON_ZERO_TOLERANCE 2.3283064365386962890625e-10

/*
// In cases when an relative "zero" tolerance is
// required for comparing model space coordinates, 
// (fabs(a)+fabs(b))*ON_RELATIVE_TOLERANCE is used.
// ON_RELATIVE_TOLERANCE should be larger than
// ON_EPSILON and smaller than no larger than 
// ON_ZERO_TOLERANCE*2^-10.
// 
*/
// ON_RELATIVE_TOLERANCE = 2^-42
#define ON_RELATIVE_TOLERANCE 2.27373675443232059478759765625e-13

/*
// Bugs in geometry calculations involving world coordinates 
// values > ON_MAXIMUM_WORLD_COORDINATE_VALUE
// will be a low priority.
*/
// ON_MAXIMUM_VALUE = 2^27
#define ON_MAXIMUM_WORLD_COORDINATE_VALUE 1.34217728e8

/*
// The default test for deciding if a curvature value should be
// treated as zero is 
// length(curvature) <= ON_ZERO_CURVATURE_TOLERANCE.
// ON_ZERO_CURVATURE_TOLERANCE must be set so that
// ON_ZERO_CURVATURE_TOLERANCE >= sqrt(3)*ON_ZERO_TOLERANCE
// so that K.IsTiny() = true implies |K| <= ON_ZERO_CURVATURE_TOLERANCE
*/
#define ON_ZERO_CURVATURE_TOLERANCE 1.0e-8

/* default value for angle tolerances = 1 degree */
#define ON_DEFAULT_ANGLE_TOLERANCE (ON_PI/180.0)
#define ON_DEFAULT_ANGLE_TOLERANCE_COSINE 0.99984769515639123915701155881391
#define ON_MINIMUM_ANGLE_TOLERANCE (ON_DEFAULT_ANGLE_TOLERANCE/10.0)

// pair of integer indices.  This
// is intentionally a struct/typedef
// rather than a class so that it
// can be used in other structs.
struct tagON_2dex
{
  int i;
  int j;
};

typedef struct tagON_2dex ON_2dex;

// triplet of integer indices.  This
// is intentionally a struct/typedef
// rather than a class so that it
// can be used in other structs.
struct tagON_3dex
{
  int i;
  int j;
  int k;
};

typedef struct tagON_3dex ON_3dex;


// quadruplet of integer indices.  This
// is intentionally a struct/typedef
// rather than a class so that it
// can be used in other structs.
struct tagON_4dex
{
  int i;
  int j;
  int k;
  int l;
};

typedef struct tagON_4dex ON_4dex;

union ON_U
{
  char      b[8]; // 8 bytes
  ON__INT64 h;    // 64 bit integer
  ON__INT32 i;    // 32 bit integer
  int       j[2]; // two 32 bit integers
  void*     p;
  double    d;
};

#if defined(ON_CPLUSPLUS)

// OpenNurbs enums
class PCL_EXPORTS ON_CLASS ON
{
public:
  /*
  Description:
    Call before using openNURBS to ensure all class definitions
    are linked.
  */
  static void Begin();


  /*
  Description:
    Call when finished with openNURBS.
  Remarks:
    Currently does nothing.
  */
  static void End();

  //////////
  // Version of opennurbs (YYYYMMDDn)
  static
  int Version();

  //////////
  // McNeel subversion revsion used to build opennurbs
  static
  const char* SourceRevision();

  static
  const char* DocumentationRevision();

  static
  const char* SourceBranch();

  static
  const char* DocumentationBranch();


  //// File open/close for DLL use ///////////////////////////////////////////////

  static
  FILE* OpenFile( // like fopen() - needed when OpenNURBS is used as a DLL
          const char* filename,
          const char* filemode
          );

  static
  FILE* OpenFile( // like fopen() - needed when OpenNURBS is used as a DLL
          const wchar_t* filename,
          const wchar_t* filemode
          );

  static
  int CloseFile( // like fclose() - needed when OpenNURBS is used as a DLL
          FILE* // pointer returned by OpenFile()
          );

  static
  int CloseAllFiles(); // like _fcloseall() - needed when OpenNURBS is used as a DLL

  /*
  Description:
    Uses the flavor of fstat that is appropriate for the platform.
  Parameters:
    filename - [in]
    fp - [in]
    filesize - [out] (can be NULL if you do not want filesize)
    create_time - [out] (can be NULL if you do not want last create time)
    lastmodify_time - [out] (can be NULL if you do not want last modification time)
  Returns:
    True if file exists, can be opened for read, and fstat worked.
  */
  static
  bool GetFileStats( const wchar_t* filename,
                     std::size_t* filesize,
                     time_t* create_time,
                     time_t* lastmodify_time
                    );

  static
  bool GetFileStats( FILE* fp,
                     std::size_t* filesize,
                     time_t* create_time,
                     time_t* lastmodify_time
                    );

  /*
  Returns true if pathname is a directory.
  */
  static bool IsDirectory( const wchar_t* pathname );
  static bool IsDirectory( const char* utf8pathname );

  /*
  Returns
    If the file is an opennurbs file, the version of the file
    is returned (2,3,4,50,...).
    If the file is not an opennurbs file, 0 is returned.
  */
  static int IsOpenNURBSFile( const wchar_t* pathname );
  static int IsOpenNURBSFile( const char* utf8pathname );
  static int IsOpenNURBSFile( FILE* fp );

  //// Dimension Types ///////////////////////////////////////////////////////////
  enum eAnnotationType
  {
    dtNothing,
    dtDimLinear,
    dtDimAligned,
    dtDimAngular,
    dtDimDiameter,
    dtDimRadius,
    dtLeader,
    dtTextBlock,
    dtDimOrdinate,
  };

  static eAnnotationType AnnotationType(int); // convert integer to eAnnotationType enum


  //// Text Display Modes ///////////////////////////////////////////////////////////
  enum eTextDisplayMode
  {
    dtNormal = 0,
    dtHorizontal = 1,
    dtAboveLine = 2,
    dtInLine = 3,
  };

  static eTextDisplayMode TextDisplayMode( int);

  // Defines the current working space.
  enum active_space
  {
    no_space    = 0,
    model_space = 1, // 3d modeling or "world" space
    page_space  = 2  // page/layout/paper/printing space
  };

  static active_space ActiveSpace(int); // convert integer to active_space enum



  //// unit_system ///////////////////////////////////////////////////////////////
  enum unit_system
  {
    // The constant enum values are saved in 3dm files 
    // and must never be changed.  The values > 11 were
    // added 5 April 2006.
    no_unit_system =  0, 

    // atomic distances
    angstroms      = 12,  // 1.0e-10 meters

    // SI units
    nanometers     = 13,  // 1.0e-9 meters
    microns        =  1,  // 1.0e-6 meters
    millimeters    =  2,  // 1.0e-3 meters
    centimeters    =  3,  // 1.0e-2 meters
    decimeters     = 14,  // 1.0e-1 meters
    meters         =  4,
    dekameters     = 15,  // 1.0e+1 meters
    hectometers    = 16,  // 1.0e+2 meters
    kilometers     =  5,  // 1.0e+3 meters
    megameters     = 17,  // 1.0e+6 meters
    gigameters     = 18,  // 1.0e+9 meters

    // english distances
    microinches    =  6,  //    2.54e-8 meters (1.0e-6 inches)
    mils           =  7,  //    2.54e-5 meters (0.001 inches)
    inches         =  8,  //    0.0254  meters
    feet           =  9,  //    0.3408  meters (12 inches)
    yards          = 19,  //    0.9144  meters (36 inches)
    miles          = 10,  // 1609.344   meters (5280 feet)

    // printer distances
    printer_point  = 20,  // 1/72 inches (computer points)
    printer_pica   = 21,  // 1/6 inches  (computer picas)

    // terrestrial distances
    nautical_mile  = 22, // 1852 meters 
                         //    Approximately 1 minute of arc on a terrestrial great circle.
                         //    See http://en.wikipedia.org/wiki/Nautical_mile.

    // astronomical distances
    astronomical   = 23, // 1.4959787e+11 // http://en.wikipedia.org/wiki/Astronomical_unit
                         // 1.495979e+11  // http://units.nist.gov/Pubs/SP811/appenB9.htm  
                         //    An astronomical unit (au) is the mean distance from the 
                         //    center of the earth to the center of the sun.
    lightyears     = 24, // 9.4607304725808e+15 // http://en.wikipedia.org/wiki/Light_year
                         // 9.46073e+15 meters  // http://units.nist.gov/Pubs/SP811/appenB9.htm
                         //    A light year is the distance light travels in one Julian year.
                         //    The speed of light is exactly 299792458 meters/second.
                         //    A Julian year is exactly 365.25 * 86400 seconds and is 
                         //    approximately the time it takes for one earth orbit.
    parsecs        = 25, // 3.08567758e+16  // http://en.wikipedia.org/wiki/Parsec
                         // 3.085678e+16    // http://units.nist.gov/Pubs/SP811/appenB9.htm  

    // Custom unit systems
    custom_unit_system = 11 // x meters with x defined in ON_3dmUnitsAndTolerances.m_custom_unit_scale
  };

  static unit_system UnitSystem(int); // convert integer to unit_system enum

  /*
  Description:
    Scale factor for changing unit "standard" systems.
  Parameters:
    us_from - [in]
    us_to - [in] 
  For example:

          100.0 = ON::UnitScale( ON::meters, ON::centimeters ) 
          2.54  = ON::UnitScale( ON::inches, ON::centimeters ) 
          12.0  = ON::UnitScale( ON::feet,   ON::inches ) 

  Remarks:
    If you are using custom unit systems, use the version
    that takes ON_UnitSystem or ON_3dmUnitsAndTolerances 
    parameters.
  */
  static double UnitScale(
      ON::unit_system us_from,
      ON::unit_system us_to
      );
  static double UnitScale(
      const class ON_UnitSystem& us_from, 
      const class ON_UnitSystem& us_to
      );
  static double UnitScale(
      ON::unit_system us_from,
      const class ON_UnitSystem& us_to
      );
  static double UnitScale(
      const class ON_UnitSystem& us_from, 
      ON::unit_system us_to
      );
  static double UnitScale(
      const class ON_3dmUnitsAndTolerances& us_from, 
      const class ON_3dmUnitsAndTolerances& us_to
      );


  /*
  Description:
    Returns the string " : ".  This is the string Rhino uses
    to separate reference model names from the root name for 
    things like layer, block definition, material, linetype,
    dimstyle and font names.  
  See Also:
    ON::NameReferenceDelimiterLength()
    ON::IsNameReferenceDelimiter()
  */
  static const wchar_t* NameReferenceDelimiter();

  /*
  Description:
    Returns the number of characters in the string returned
    by ON::NameReferenceDelimiter().
  See Also:
    ON::NameReferenceDelimiterLength()
    ON::IsNameReferenceDelimiter()
  */
  static unsigned int NameReferenceDelimiterLength();

  /*
  Description:
    Test a string to see if its beginning matches the 
    string returned by ON::NameReferenceDelimiter().
  Parameters:
    s - [in];
      string to test.
  Returns:
    null:
      The beginning of the string does not match ON::NameReferenceDelimiter().
    non-null:
      The beginning of the string matches ON::NameReferenceDelimiter(). The
      returned pointer is the first character in s after the last character
      of the delimiter.  Put another way, if the beginning of s matches
      the string  ON::NameReferenceDelimiter(), then the returned pointer is
      s + ON::NameReferenceDelimiterLength().
  See Also:
    ON::NameReferenceDelimiter()
    ON::NameReferenceDelimiterLength()
  */
  static const wchar_t* IsNameReferenceDelimiter(const wchar_t* s);

  //// distance_display_mode ///////////////////////////////////
  enum distance_display_mode
  {
    decimal     = 0, 
    fractional  = 1,
    feet_inches = 2
  };

  static distance_display_mode DistanceDisplayMode(int); // convert integer to distance_display_mode enum


  //// point_style ///////////////////////////////////////////////////////////////
  enum point_style 
  {
    unknown_point_style   = 0,
    not_rational          = 1,
    homogeneous_rational  = 2,
    euclidean_rational    = 3,
    intrinsic_point_style = 4, // point format used in definition
    point_style_count     = 5
  };

  static point_style PointStyle(int); // convert integer to point_style enum

  //// knot_style ///////////////////////////////////////////////////////////////
  enum knot_style // if a knot vector meets the conditions of two styles,
  {               // then the style with the lowest value is used
    unknown_knot_style     = 0, // unknown knot style
    uniform_knots          = 1, // uniform knots (ends not clamped)
    quasi_uniform_knots    = 2, // uniform knots (clamped ends, degree >= 2)
    piecewise_bezier_knots = 3, // all internal knots have full multiplicity
    clamped_end_knots      = 4, // clamped end knots (with at least 1 interior non-uniform knot)
    non_uniform_knots      = 5, // known to be none of the above
    knot_style_count       = 6
  };

  static knot_style KnotStyle(int); // convert integer to knot_style enum

  //// continuity ////////////////////////////////////////////////////////////////
  enum continuity
  {
    unknown_continuity = 0,

    // These test for parametric continuity.  In particular,
    // all types of ON_Curves are considered infinitely 
    // continuous at the start/end of the evaluation domain.
    C0_continuous =  1, // continuous function
    C1_continuous =  2, // continuous first derivative
    C2_continuous =  3, // continuous first and second derivative
    G1_continuous =  4, // continuous unit tangent
    G2_continuous =  5, // continuous unit tangent and curvature

    // 20 March 2003 Dale Lear added these.
    //
    // Continuity tests using the following enum values
    // are identical to tests using the preceding enum values
    // on the INTERIOR of a curve's domain.  At the END of
    // a curve a "locus" test is performed in place of a 
    // parametric test. In particular, at the END of a domain,
    // all open curves are locus discontinuous.  At the END of
    // a domain, all closed curves are at least C0_locus_continuous.
    // By convention all ON_Curves are considered 
    // locus continuous at the START of the evaluation domain.
    // This convention is not strictly correct, but is was
    // adopted to make iterative kink finding tools easier to
    // use and so that locus discontinuities are reported once
    // at the end parameter of a curve rather than twice.
    C0_locus_continuous =  6, // locus continuous function
    C1_locus_continuous =  7, // locus continuous first derivative
    C2_locus_continuous =  8, // locus continuous first and second derivative
    G1_locus_continuous =  9, // locus continuous unit tangent
    G2_locus_continuous = 10, // locus continuous unit tangent and curvature

    Cinfinity_continuous = 11, // analytic discontinuity
    Gsmooth_continuous = 12    // aesthetic discontinuity
  };

  /*
  Description:
    Convert int to ON::continuity enum value
  */
  static continuity Continuity(int);

  /*
  Description:
    Convert int to ON::continuity enum value and
    convert the locus flavored values to the parametric
    flavored values.
  */
  static continuity ParametricContinuity(int);

  /*
  Description:
    Convert int to ON::continuity enum value and
    convert the higher order flavored values to 
    the corresponding C1 or G1 values needed to
    test piecewise linear curves.
  */
  static continuity PolylineContinuity(int);

  //// curve_style ///////////////////////////////////////////////////////////////
  enum curve_style 
  {
    unknown_curve_style   =  0,
    line                  =  1,
    circle                =  2,
    ellipse               =  3, // with distinct foci (not a circle)
    parabola              =  4,
    hyperbola             =  5,
    planar_polyline       =  6, // not a line segment
    polyline              =  7, // non-planar polyline
    planar_freeform_curve =  8, // planar but none of the above
    freeform_curve        =  9, // known to be none of the above
    curve_style_count     = 10
  };

  static curve_style CurveStyle(int); // convert integer to curve_style enum

  //// surface_style ///////////////////////////////////////////////////////////////
  enum surface_style 
  {
    unknown_surface_style =  0,
    plane                 =  1,
    circular_cylinder     =  2, // portion of right circular cylinder
    elliptical_cylinder   =  3, // portion of right elliptical cylinder
    circular_cone         =  4, // portion of right circular cone
    elliptical_cone       =  5, // portion of right elliptical cone
    sphere                =  6, // portion of sphere
    torus                 =  7, // portion of torus
    surface_of_revolution =  8, // portion of surface of revolution that is none of the above
    ruled_surface         =  9, // portion of a ruled surface this is none of the above
    freeform_surface      = 10, // known to be none of the above
    surface_style_count   = 11
  };

  static surface_style SurfaceStyle(int); // convert integer to surface_style enum

  //// sort_algorithm ///////////////////////////////////////////////////////////////
  enum sort_algorithm
  {
    heap_sort  = 0,
    quick_sort = 1
  };

  static sort_algorithm SortAlgorithm(int); // convert integer to sort_method enum

  //// endian-ness ///////////////////////////////////////////////////////////////
  enum endian {
    little_endian = 0, // least significant byte first or reverse byte order - Intel x86, ...
    big_endian    = 1  // most significant byte first - Motorola, Sparc, MIPS, ...
  };

  static endian Endian(int); // convert integer to endian enum
  static endian Endian();    // returns endian-ness of current CPU

  //// archive modes //////////////////////////////////////////////////////////////
  enum archive_mode
  {
    unknown_archive_mode = 0,
    read      = 1, // all read modes have bit 0x0001 set
    write     = 2, // all write modes have bit 0x0002 set
    readwrite = 3,
    read3dm   = 5,
    write3dm  = 6
  };
  static archive_mode ArchiveMode(int); // convert integer to endian enum


  //// view projections ///////////////////////////////////////////////////////////

  // The x/y/z_2pt_perspective_view projections are ordinary perspective
  // projection. Using these values insures the ON_Viewport member 
  // fuctions properly constrain the camera up and camera direction vectors
  // to preserve the specified perspective vantage.
  enum view_projection
  { 
    unknown_view       = 0,
    parallel_view      = 1,
    perspective_view   = 2
  };

  /*
  Description:
    Converts integer into ON::view_projection enum value.
  Parameters:
    i - [in]
  Returns:
    ON::view_projection enum with same value as i.
    If i is not an ON::view_projection enum value,
    then ON::unknow_view is returned.
  */
  static view_projection ViewProjection(int i);

  /*
  Parameters:
    projection - [in]
  Returns:
    True if projection is ON::perspective_view.
  */
  static bool IsPerspectiveProjection( ON::view_projection projection );


  /*
  Parameters:
    projection - [in]
  Returns:
    True if projection is ON::parallel_view.
  */
  static bool IsParallelProjection( ON::view_projection projection );

  //// view coordinates ///////////////////////////////////////////////////////////

  enum coordinate_system 
  {
    world_cs  = 0, 
    camera_cs = 1, 
    clip_cs   = 2, 
    screen_cs = 3 
  };

  static coordinate_system CoordinateSystem(int); // convert integer to coordinate_system enum

  //// exception types ///////////////////////////////////////////////////////////
	enum exception_type 
  {
    unknown_exception = 0,
		out_of_memory,  
    corrupt_object,               // invalid object encountered - continuing would crash or
                                  // result in corrupt object being saved in archive.
		unable_to_write_archive,      // write operation failed - out of file space/read only mode/...?
		unable_to_read_archive,       // read operation failed - truncated archive/locked file/... ?
		unable_to_seek_archive,       // seek operation failed - locked file/size out of bounds/... ?
		unexpected_end_of_archive,    // truncated archive
		unexpected_value_in_archive   // corrupt archive?
  };
  static exception_type ExceptionType(int); // convert integer to exception_type enum

  //// layer mode ///////////////////////////////////////////////////////////
  // OBSOLETE 
	enum layer_mode 
  {
    normal_layer       = 0, // visible, objects on layer can be selected and changed
    hidden_layer       = 1, // not visible, objects on layer cannot be selected or changed
    locked_layer       = 2, // visible, objects on layer cannot be selected or changed
    layer_mode_count   = 3
  };
  static layer_mode LayerMode(int); // convert integer to layer_mode enum

  //// object mode ///////////////////////////////////////////////////////////
	enum object_mode 
  {
    normal_object    = 0, // object mode comes from layer
    hidden_object    = 1, // not visible, object cannot be selected or changed
    locked_object    = 2, // visible, object cannot be selected or changed
    idef_object      = 3, // object is part of an ON_InstanceDefinition.  The
                          // ON_InstanceDefinition m_object_uuid[] array will
                          // contain this object attribute's uuid.
    object_mode_count = 4
  };
  static object_mode ObjectMode(int); // convert integer to object_mode enum

  //// object display color /////////////////////////////////////////////////////////
	enum object_color_source
  {
    color_from_layer    = 0, // use color assigned to layer
    color_from_object   = 1, // use color assigned to object
    color_from_material = 2, // use diffuse render material color
    color_from_parent   = 3  // for objects with parents (like objects in instance references, use parent linetype)
                             // if no parent, treat as color_from_layer
  };
  static object_color_source ObjectColorSource(int); // convert integer to object_color_source enum

  //// object plot color /////////////////////////////////////////////////////////
	enum plot_color_source
  {
    plot_color_from_layer   = 0, // use plot color assigned to layer
    plot_color_from_object  = 1, // use plot color assigned to object
    plot_color_from_display = 2, // use display color
    plot_color_from_parent  = 3  // for objects with parents (like objects in instance references, use parent plot color)
                                 // if no parent, treat as plot_color_from_layer
  };
  static plot_color_source PlotColorSource(int); // convert integer to plot_color_source enum

  //// object plot weight /////////////////////////////////////////////////////////
	enum plot_weight_source
  {
    plot_weight_from_layer   = 0, // use plot color assigned to layer
    plot_weight_from_object  = 1, // use plot color assigned to object
    plot_weight_from_parent  = 3  // for objects with parents (like objects in instance references, use parent plot color)
                                  // if no parent, treat as plot_color_from_layer
  };
  static plot_weight_source PlotWeightSource(int); // convert integer to plot_color_source enum

  //// object linetype /////////////////////////////////////////////////////////
	enum object_linetype_source
  {
    linetype_from_layer  = 0, // use line style assigned to layer
    linetype_from_object = 1, // use line style assigned to object
    linetype_from_parent = 3  // for objects with parents (like objects in instance references, use parent linetype)
                              // if not parent, treat as linetype_from_layer.
  };
  static object_linetype_source ObjectLinetypeSource(int); // convert integer to object_linetype_source enum

  //// object material /////////////////////////////////////////////////////////
	enum object_material_source
  {
    material_from_layer  = 0, // use material assigned to layer
    material_from_object = 1, // use material assigned to object
    material_from_parent = 3  // for objects with parents, like 
                              // definition geometry in instance
                              // references and faces in polysurfaces,
                              // this value indicates the material
                              // definition should come from the parent.
                              // If the object does not have an 
                              // obvious "parent", then treat
                              // it the same as material_from_layer.
  };
  static object_material_source ObjectMaterialSource(int); // convert integer to object_color_source enum

  //// light style /////////////////////////////////////////////////////////////
  enum light_style
  {
    unknown_light_style      = 0,
    //view_directional_light   = 1, // light location and direction in clip coordinates
    //view_point_light         = 2,
    //view_spot_light          = 3,
    camera_directional_light = 4, // light location and direction in camera coordinates
    camera_point_light       = 5, //   +x points to right, +y points up, +z points towards camera
    camera_spot_light        = 6,
    world_directional_light  = 7, // light location and direction in world coordinates
    world_point_light        = 8, 
    world_spot_light         = 9,
    ambient_light            = 10, // pure ambient light
    world_linear_light       = 11,
    world_rectangular_light  = 12,
    light_style_count        = 13
  };
  static light_style LightStyle(int); // convert integer to light_style enum

  //// curvature style /////////////////////////////////////////////////////////
  enum curvature_style
  {
    unknown_curvature_style = 0,
    gaussian_curvature = 1,
    mean_curvature = 2, // unsigned mean curvature
    min_curvature  = 3, // minimum unsigned radius of curvature
    max_curvature  = 4, // maximum unsigned radius of curvature
    curvature_style_count = 5
  };
  static curvature_style CurvatureStyle(int); // convert integer to curvature_style enum

  //// view display mode /////////////////////////////////////////////////////////////
  enum display_mode
  {
    default_display       = 0, // default display
    wireframe_display     = 1, // wireframe display
    shaded_display        = 2, // shaded display
    renderpreview_display = 3  // render preview display
  };
  static display_mode DisplayMode(int); // convert integer to display_mode enum


  enum view_type
  {
    model_view_type  = 0,       // standard model space 3d view
    page_view_type   = 1,       // a.k.a "paper space", "plot view", etc.
                                // A page view must be orthographic,
                                // the camera frame x,y,z direction must be
                                // world x,y,z (which means the camera direction
                                // is always (0,0,-1)).  
    nested_view_type = 2,       // This view is a "model" view that is nested
                                // in another view.  The nesting and parent
                                // information is saved in ON_3dmView.
  };
  static view_type ViewType(int); // convert integer to display_mode enum


  //// texture mapping mode ///////////////////////////////////////////////////
  //
  // OBSOLETE 
  enum texture_mode
  {
    no_texture = 0,        // texture disabled
    modulate_texture = 1,  // modulate with material diffuse color
    decal_texture = 2      // decal
  };
  // OBSOLETE 
  static texture_mode TextureMode(int); // convert integer to texture_mode enum
  // OBSOLETE 
  //
  /////////////////////////////////////////////////////////////////////////////

  //// object_type ///////////////////////////////////////////////////
  enum object_type
  {
    // Use with ON_Object::ObjectType() in situations where
    // using a switch() is better than a long string of if else if ...
    // if ( ON_Curve::Cast() ) ... else if ( ON_Surface::Cast() ) ...
    // ...
    unknown_object_type  =          0,

    point_object         =          1, // some type of ON_Point
    pointset_object      =          2, // some type of ON_PointCloud, ON_PointGrid, ...
    curve_object         =          4, // some type of ON_Curve like ON_LineCurve, ON_NurbsCurve, etc.
    surface_object       =          8, // some type of ON_Surface like ON_PlaneSurface, ON_NurbsSurface, etc.
    brep_object          =       0x10, // some type of ON_Brep
    mesh_object          =       0x20, // some type of ON_Mesh
    layer_object         =       0x40, // some type of ON_Layer
    material_object      =       0x80, // some type of ON_Material
    light_object         =      0x100, // some type of ON_Light
    annotation_object    =      0x200, // some type of ON_Annotation
    userdata_object      =      0x400, // some type of ON_UserData
    instance_definition  =      0x800, // some type of ON_InstanceDefinition
    instance_reference   =     0x1000, // some type of ON_InstanceRef
    text_dot             =     0x2000, // some type of ON_TextDot
    grip_object          =     0x4000, // selection filter value - not a real object type
    detail_object        =     0x8000, // some type of ON_DetailView
    hatch_object         =    0x10000, // some type of ON_Hatch
    morph_control_object =    0x20000, // some type of ON_MorphControl
    loop_object          =    0x80000, // some type of ON_BrepLoop
    polysrf_filter       =   0x200000, // selection filter value - not a real object type
    edge_filter          =   0x400000, // selection filter value - not a real object type
    polyedge_filter      =   0x800000, // selection filter value - not a real object type
    meshvertex_object    = 0x01000000, // some type of ON_MeshVertexRef
    meshedge_object      = 0x02000000, // some type of ON_MeshEdgeRef
    meshface_object      = 0x04000000, // some type of ON_MeshFaceRef
    cage_object          = 0x08000000, // some type of ON_NurbsCage
    phantom_object       = 0x10000000,
    clipplane_object     = 0x20000000,
    beam_object          = 0x40000000, // obsolete - use extrusion_object
    extrusion_object     = 0x40000000, // some type of ON_Extrusion
    
    any_object           = 0xFFFFFFFF

    // Please discuss any changes with Dale Lear
  };

  static object_type ObjectType(int); // convert integer to object_type enum

  //// bitmap_type ///////////////////////////////////////////////////
  enum bitmap_type
  {
    unknown_bitmap_type = 0,
    windows_bitmap = 1,     // BITMAPINFO style
    opengl_bitmap = 2,      // unpacked OpenGL RGB or RGBA
    png_bitmap = 3
  };
  static bitmap_type BitmapType(int); // convert integer to bitmap_type enum

  enum object_decoration
  {
    no_object_decoration = 0,
    start_arrowhead      = 0x08, // arrow head at start
    end_arrowhead        = 0x10, // arrow head at end
    both_arrowhead       = 0x18  // arrow heads at start and end
  };
  static object_decoration ObjectDecoration(int); // convert integer to line_pattern enum

  enum mesh_type
  {
    default_mesh  = 0,
    render_mesh   = 1,
    analysis_mesh = 2,
    preview_mesh  = 3,
    any_mesh      = 4
  };
  static mesh_type MeshType(int); // convert integer to mesh_type enum


  // Types of object snapping.  
  // In situations where more than one type of snap applies, 
  // snaps with higher value take precedence.
  // enum values must be a power of 2.
  // ON_ObjRef saves these values in files.  Do not change
  // the values.  The reason for the gaps between the enum
  // values is to leave room for future snaps with prededence
  // falling between existing snaps
  enum osnap_mode
  {
    os_none          =          0,
    os_near          =          2,
    os_focus         =          8,
    os_center        =       0x20,
    os_vertex        =       0x40,
    os_knot          =       0x80,
    os_quadrant      =      0x200,
    os_midpoint      =      0x800,
    os_intersection  =     0x2000,
    os_end           =    0x20000,
    os_perpendicular =    0x80000,
    os_tangent       =   0x200000,
    os_point         = 0x08000000,
    os_all_snaps     = 0xFFFFFFFF
  };
  static osnap_mode OSnapMode(int); // convert integer to osnap_mode enum


  //// Types of Curves ///////////////////////////////////////////////////////////
  enum eCurveType
  {
    ctCurve, // nothing
    ctArc,
    ctCircle,
    ctLine,
    ctNurbs,
    ctOnsurface,
    ctProxy,
    ctPolycurve,
    ctPolyline,
  };


  //// surface_loft_end_condition //////////////////////////////////////////////
  //
  // End condition paramter values for  ON_Curve::CreateCubicLoft() and
  // ON_Surface::CreateCubicLoft().
  enum cubic_loft_end_condition
  {
    cubic_loft_ec_quadratic      = 0,
    cubic_loft_ec_linear         = 1,
    cubic_loft_ec_cubic          = 2,
    cubic_loft_ec_natural        = 3,
    cubic_loft_ec_unit_tangent   = 4,
    cubic_loft_ec_1st_derivative = 5,
    cubic_loft_ec_2nd_derivative = 6,
    cubic_loft_ec_free_cv        = 7
  };

  /*
  Description:
    Convert an integer to cubic_loft_end_condition enum.
  Parameters:
    i - [in]
  Returns:
    corresponding cubic_loft_end_condition enum value.
  Remarks:
    If i does not correspond to a cubic_loft_end_condition
    enum value, then cubic_loft_ec_quadratic is returned.
  */
  static 
  cubic_loft_end_condition CubicLoftEndCondition(int i); 

private:
  // prohibit instantiaion
  //ON();             // no implementation
  //ON( const ON& );  // no implementation
  //~ON();            // no implementation
};


/*
Description:
  Component indices are used to provide a persistent way
  to identify portions of complex objects.

*/
class ON_CLASS ON_COMPONENT_INDEX
{
public:

  // Do not change these values; they are stored in 3dm archives
  // and provide a persistent way to indentify components of
  // complex objects.
  enum TYPE
  {
    invalid_type       =   0,

    brep_vertex        =   1,
    brep_edge          =   2,
    brep_face          =   3,
    brep_trim          =   4,
    brep_loop          =   5,

    mesh_vertex        =  11,
    meshtop_vertex     =  12,
    meshtop_edge       =  13,
    mesh_face          =  14,

    idef_part          =  21,

    polycurve_segment  =  31,

    pointcloud_point   =  41,

    group_member       =  51,


    extrusion_bottom_profile = 61, // 3d bottom profile curves
                                   //   index identifies profile component
    extrusion_top_profile    = 62, // 3d top profile curves
                                   //   index identifies profile component
    extrusion_wall_edge      = 63, // 3d wall edge curve
                                   //   index/2: identifies profile component
                                   //   index%2: 0 = start, 1 = end
    extrusion_wall_surface   = 64, // side wall surfaces
                                   //   index: identifies profile component
    extrusion_cap_surface    = 65, // bottom and top cap surfaces
                                   //   index: 0 = bottom, 1 = top
    extrusion_path           = 66, // extrusion path (axis line)
                                   //   index -1 = entire path, 0 = start point, 1 = endpoint

    dim_linear_point   = 100,
    dim_radial_point   = 101,
    dim_angular_point  = 102,
    dim_ordinate_point = 103,
    dim_text_point     = 104,

    no_type           = 0xFFFFFFFF
  };

  /*
  Description:
    Safe conversion of integer value to TYPE enum.
  Parameters:
    i - [in] integer with value equal to one of the TYPE enums.
  Returns:
    The TYPE enum with the same numeric value 
    or ON_COMPONENT_INDEX::invalid_type if no corresponding enum
    exists.
  */
  static 
  TYPE Type(int i);

  /*
  Description:
    Dictionary compare on m_type, m_index as ints.
  Returns:
    < 0: a < b
    = 0: a = b
    > 0: a > b
  */
  static
  int Compare( const ON_COMPONENT_INDEX* a, const ON_COMPONENT_INDEX* b);

  /*
  Description:
    Sets m_type = invalid_type and m_index = -1.
  */
  ON_COMPONENT_INDEX();

  /*
  Description:
    Sets m_type = type and m_index = index.
  */
  ON_COMPONENT_INDEX(TYPE type,int index);

  bool operator==(const ON_COMPONENT_INDEX& other) const;
  bool operator!=(const ON_COMPONENT_INDEX& other) const;
  bool operator<(const ON_COMPONENT_INDEX& other) const;
  bool operator<=(const ON_COMPONENT_INDEX& other) const;
  bool operator>(const ON_COMPONENT_INDEX& other) const;
  bool operator>=(const ON_COMPONENT_INDEX& other) const;

  void Set(TYPE type,int index);

  /*
  Description:
    Sets m_type = invalid_type and m_index = -1.
  */
  void UnSet();

  /*
  Returns:
    True if m_type is set to a TYPE enum value between
    brep_vertex and polycurve_segment.
  */
  bool IsSet() const;

  /*
  Returns:
    True if m_type is set to one of the mesh or meshtop
    TYPE enum values and m_index >= 0.
  */
  bool IsMeshComponentIndex() const;

  /*
  Returns:
    True if m_type is set to one of the 
    brep TYPE enum values and m_index >= 0.
  */
  bool IsBrepComponentIndex() const;

  /*
  Returns:
    True if m_type = idef_part and m_index >= 0.
  */
  bool IsIDefComponentIndex() const;

  /*
  Returns:
    True if m_type = polycurve_segment and m_index >= 0.
  */
  bool IsPolyCurveComponentIndex() const;

  /*
  Returns:
    True if m_type = group_member and m_index >= 0.
  */
  bool IsGroupMemberComponentIndex() const;

  /*
  Returns:
    True if m_type = extrusion_bottom_profile or extrusion_top_profile
    and m_index >= 0.
  */
  bool IsExtrusionProfileComponentIndex() const;

  /*
  Returns:
    True if m_type = extrusion_path and -1 <= m_index <= 1.
  */
  bool IsExtrusionPathComponentIndex() const;

  /*
  Returns:
    True if m_type = extrusion_wall_edge and m_index >= 0.
  */
  bool IsExtrusionWallEdgeComponentIndex() const;

  /*
  Returns:
    True if m_type = extrusion_wall_surface and m_index >= 0.
  */
  bool IsExtrusionWallSurfaceComponentIndex() const;

  /*
  Returns:
    True if m_type = extrusion_wall_surface or extrusion_wall_edge
    and m_index >= 0.
  */
  bool IsExtrusionWallComponentIndex() const;

  /*
  Returns:
    True if m_type = extrusion_bottom_profile, extrusion_top_profile,
    extrusion_wall_edge, extrusion_wall_surface, extrusion_cap_surface
    or extrusion_path and m_index is reasonable.
  */
  bool IsExtrusionComponentIndex() const;

  /*
  Returns:
    True if m_type = pointcloud_point and m_index >= 0.
  */
  bool IsPointCloudComponentIndex() const;

  /*
  Returns:
    True if m_type = dim_... and m_index >= 0.
  */
  bool IsAnnotationComponentIndex() const;

  TYPE m_type;

  /*
  The interpretation of m_index depends on the m_type value.

    m_type             m_index interpretation (0 based indices)

    no_type            used when context makes it clear what array is being index
    brep_vertex        ON_Brep.m_V[] array index
    brep_edge          ON_Brep.m_E[] array index
    brep_face          ON_Brep.m_F[] array index
    brep_trim          ON_Brep.m_T[] array index
    brep_loop          ON_Brep.m_L[] array index
    mesh_vertex        ON_Mesh.m_V[] array index
    meshtop_vertex     ON_MeshTopology.m_topv[] array index
    meshtop_edge       ON_MeshTopology.m_tope[] array index
    mesh_face          ON_Mesh.m_F[] array index
    idef_part          ON_InstanceDefinition.m_object_uuid[] array index
    polycurve_segment  ON_PolyCurve::m_segment[] array index

    extrusion_bottom_profile  Use ON_Extrusion::Profile3d() to get 3d profile curve
    extrusion_top_profile     Use ON_Extrusion::Profile3d() to get 3d profile curve
    extrusion_wall_edge       Use ON_Extrusion::WallEdge() to get 3d line curve
    extrusion_wall_surface    Use ON_Extrusion::WallSurface() to get 3d wall surface
    extrusion_cap_surface      0 = bottom cap, 1 = top cap
    extrusion_path            -1 = entire path, 0 = start of path, 1 = end of path

    dim_linear_point   ON_LinearDimension2::POINT_INDEX
    dim_radial_point   ON_RadialDimension2::POINT_INDEX
    dim_angular_point  ON_AngularDimension2::POINT_INDEX
    dim_ordinate_point ON_OrdinateDimension2::POINT_INDEX
    dim_text_point     ON_TextEntity2 origin point
  */
  int m_index;
};

#endif

ON_BEGIN_EXTERNC

/*
Description:
  Sets Windows code page used to convert UNICODE (wchar_t) strings
  to multibyte (char) strings and vice verse.
Parameters:
  code_page - [in] code page to use when converting UNICODE strings
       to multibyte strings and vice verse.
Returns:
  previous value of Windows code page.
Remarks:
  For Windows NT/2000/XP, CP_THREAD_ACP will work for all
  locales if your app's thread is correctly configured.
  For Windows 95/98/ME you have to choose the locale.

  Conversions between UNICODE and multibyte strings happens when
  ON_wString converts a char* string to a wchar_t* string and
  when and ON_String converts a wchar_t* string to a char* string.

  All pertinant code is in opennurbs_defines.cpp.

See Also:
  ON_GetStringConversionWindowsCodePage
  on_WideCharToMultiByte
  on_MultiByteToWideChar
  ON_wString::operator=(const char*)
  ON_String::operator=(const wchar_t*)  
*/
ON_DECL
unsigned int ON_SetStringConversionWindowsCodePage( 
                unsigned int code_page 
                );

/*
Description:
  Gets Windows code page used to convert UNICODE (wchar_t) strings
  to multibyte (char) strings and vice verse.
Returns:
  Value of Windows code page used to convert strings.
Remarks:
  For Windows NT/2000/XP, CP_THREAD_ACP will work for all
  locales if your app's thread is correctly configured.
  For Windows 95/98/ME you have to choose the locale.

  Conversions between UNICODE and multibyte strings happens when
  ON_wString converts a char* string to a wchar_t* string and
  when and ON_String converts a wchar_t* string to a char* string.

  All pertinant code is in opennurbs_defines.cpp.

See Also:
  ON_GetStringConversionWindowsCodePage
  on_WideCharToMultiByte
  on_MultiByteToWideChar
  ON_wString::operator=(const char*)
  ON_String::operator=(const wchar_t*)  
*/
ON_DECL
unsigned int ON_GetStringConversionWindowsCodePage();


/*
Description:
  Sets Windows locale id used in case insensitive string
  compares.
Parameters:
  locale_id - [in] Windows locale id to use in case insensitive
                 string compares.
  bWin9X - [in] True if OS is Windows 95/98/ME (which has
                poor UNICODE support).
Returns:
  Previous value of Windows locale id.
Remarks:
  All pertinant code is in opennurbs_defines.cpp.
See Also:
  ON_GetStringConversionWindowsLocaleID
  on_wcsicmp
*/
ON_DECL
unsigned int ON_SetStringConversionWindowsLocaleID( 
                unsigned int locale_id, 
                ON_BOOL32 bWin9X
                );

/*
Description:
  Gets Windows locale id used in case insensitive string
  compares.
Returns:
  Value of Windows locale id used in case insensitive string
  compares.
Remarks:
  All pertinant code is in opennurbs_defines.cpp.
See Also:
  ON_SetStringConversionWindowsLocaleID
  on_wcsicmp
*/
ON_DECL
unsigned int ON_GetStringConversionWindowsLocaleID();

// on_wcsicmp() is a wrapper for case insensitive wide string compare
// and calls one of _wcsicmp() or wcscasecmp() depending on OS.
ON_DECL
int on_wcsicmp( const wchar_t*, const wchar_t* );

// on_wcsupr() calls _wcsupr() or wcsupr() depending on OS
ON_DECL
wchar_t* on_wcsupr(wchar_t*);

// on_wcslwr() calls _wcslwr() or wcslwr() depending on OS
ON_DECL
wchar_t* on_wcslwr(wchar_t*);

// on_wcsrev() calls _wcsrev() or wcsrev() depending on OS
ON_DECL
wchar_t* on_wcsrev(wchar_t*);

// on_stricmp() is a wrapper for case insensitive string compare
// and calls one of _stricmp(), stricmp(), or strcasecmp()
// depending on OS.
ON_DECL
int on_stricmp(const char*, const char*); 

// on_stricmp() is a wrapper for case insensitive string compare
// and calls one of _strnicmp() or strncasecmp()
// depending on OS.
ON_DECL
int on_strnicmp(const char * s1, const char * s2, int n);

// on_strupr() calls _strupr() or strupr() depending on OS
ON_DECL
char* on_strupr(char*);

// on_strlwr() calls _strlwr() or strlwr() depending on OS
ON_DECL
char* on_strlwr(char*);

// on_strrev() calls _strrev() or strrev() depending on OS
ON_DECL
char* on_strrev(char*);

/*
Description:
  Calls ON_ConvertWideCharToUTF8()
*/
ON_DECL
int on_WideCharToMultiByte(
    const wchar_t*, // lpWideCharStr,
    int,            // cchWideChar,
    char*,          // lpMultiByteStr,
    int             // cchMultiByte,
    );

/*
Description:
  Calls ON_ConvertUTF8ToWideChar()
*/
ON_DECL
int on_MultiByteToWideChar(
    const char*, // lpMultiByteStr,
    int,         // cchMultiByte,
    wchar_t*,    // lpWideCharStr,
    int          // cchWideChar
    );

/*
Description:
  Find the locations in a path the specify the drive, directory,
  file name and file extension.
Parameters:
  path - [in]
    UTF-8 encoded string that is a legitimate path to a file.
  drive - [out] (pass null if you don't need the drive)
    If drive is not null and the path parameter begins with 
    an A-Z or a-z followed by a colon ( : ) then the returned
    value of *drive will equal the input value of path.
  dir - [out] (pass null if you don't need the directory)
    If dir is not null and the path parameter contains a
    directory specification, then the returned value of *dir
    will point to the character in path where the directory
    specification begins.
  fname - [out] (pass null if you don't need the file name)
    If fname is not null and the path parameter contains a
    file name specification, then the returned value of *fname
    will point to the character in path where the file name
    specification begins.
  ext - [out] (pass null if you don't need the extension)
    If ext is not null and the path parameter contains a
    file extension specification, then the returned value of
    *ext will point to the '.' character in path where the file
    extension specification begins.
Remarks:
  This function will treat a front slash ( / ) and a back slash
  ( \ ) as directory separators.  Because this function parses
  file names store in .3dm files and the .3dm file may have been
  written on a Windows computer and then read on a another
  computer, it looks for a drive dpecification even when the
  operating system is not Windows.
  This function will not return an directory that does not
  end with a trailing slash.
  This function will not return an empty filename and a non-empty
  extension.
  This function parses the path string according to these rules.
  It does not check the actual file system to see if the answer
  is correct.
See Also:
  ON_String::SplitPath
*/
ON_DECL void on_splitpath(
  const char* path,
  const char** drive,
  const char** dir,
  const char** fname,
  const char** ext
  );

/*
Description:
  Find the locations in a path the specify the drive, directory,
  file name and file extension.
Parameters:
  path - [in]
    UTF-8, UTF-16 or UTF-32 encoded wchar_t string that is a
    legitimate path to a file.
  drive - [out] (pass null if you don't need the drive)
    If drive is not null and the path parameter begins with 
    an A-Z or a-z followed by a colon ( : ) then the returned
    value of *drive will equal the input value of path.
  dir - [out] (pass null if you don't need the directory)
    If dir is not null and the path parameter contains a
    directory specification, then the returned value of *dir
    will point to the character in path where the directory
    specification begins.
  fname - [out] (pass null if you don't need the file name)
    If fname is not null and the path parameter contains a
    file name specification, then the returned value of *fname
    will point to the character in path where the file name
    specification begins.
  ext - [out] (pass null if you don't need the extension)
    If ext is not null and the path parameter contains a
    file extension specification, then the returned value of
    *ext will point to the '.' character in path where the file
    extension specification begins.
Remarks:
  This function will treat a front slash ( / ) and a back slash
  ( \ ) as directory separators.  Because this function parses
  file names store in .3dm files and the .3dm file may have been
  written on a Windows computer and then read on a another
  computer, it looks for a drive dpecification even when the
  operating system is not Windows.
  This function will not return an directory that does not
  end with a trailing slash.
  This function will not return an empty filename and a non-empty
  extension.
  This function parses the path string according to these rules.
  It does not check the actual file system to see if the answer
  is correct.
See Also:
  ON_wString::SplitPath
*/
ON_DECL void on_wsplitpath(
  const wchar_t* path,
  const wchar_t** drive,
  const wchar_t** dir,
  const wchar_t** fname,
  const wchar_t** ext
  );

ON_END_EXTERNC


#endif
