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

#include <pcl/surface/openNURBS/opennurbs.h>

void ON_MassProperties::Dump( ON_TextLog& dump ) const
{
  const char* sMass = 0;
  switch( m_mass_type )
  {
  case 1:
    sMass = "Length";
    break;
  case 2:
    sMass = "Area";
    break;
  case 3:
    sMass = "Volume";
    break;
  }
  if ( 0 == sMass )
  {
    dump.Print("Invalid mass properties.\n");
  }
  else
  {
    dump.Print("%s mass properties:\n",sMass);
    dump.PushIndent();

    if ( m_bValidMass )
      dump.Print("%s = %g (+/- %g)\n",sMass,m_mass,m_mass_err);

    if ( m_bValidCentroid )
      dump.Print("Centroid = (%g,%g,%g) (+/- %g,%g,%g)\n",
                 m_x0,m_y0,m_z0,m_x0_err,m_y0_err,m_z0_err);

    if ( m_bValidFirstMoments )
    {
      dump.Print("First moments:\n");
      dump.PushIndent();
      dump.Print("x: %g (+/- %g)\n",m_world_x,m_world_x_err);
      dump.Print("y: %g (+/- %g)\n",m_world_y,m_world_y_err);
      dump.Print("z: %g (+/- %g)\n",m_world_z,m_world_z_err);
      dump.PopIndent();
    }

    if ( m_bValidSecondMoments )
    {
      dump.Print("Second moments about world coordinate axes\n");
      dump.PushIndent();
      dump.Print("xx: %g (+/- %g)\n",m_world_xx,m_world_xx_err);
      dump.Print("yy: %g (+/- %g)\n",m_world_yy,m_world_yy_err);
      dump.Print("zz: %g (+/- %g)\n",m_world_zz,m_world_zz_err);
      dump.PopIndent();
      dump.Print("Second moments about centroid coordinate axes\n");
      dump.PushIndent();
      dump.Print("xx: %g (+/- %g)\n",m_ccs_xx,m_ccs_xx_err);
      dump.Print("yy: %g (+/- %g)\n",m_ccs_yy,m_ccs_yy_err);
      dump.Print("zz: %g (+/- %g)\n",m_ccs_zz,m_ccs_zz_err);
      dump.PopIndent();
    }

    if ( m_bValidProductMoments )
    {
      dump.Print("Product moments about world coordinate axes\n");
      dump.PushIndent();
      dump.Print("xy: %g (+/- %g)\n",m_world_xy,m_world_xy_err);
      dump.Print("yz: %g (+/- %g)\n",m_world_yz,m_world_yz_err);
      dump.Print("zx: %g (+/- %g)\n",m_world_zx,m_world_zx_err);
      dump.PopIndent();
      dump.Print("Product moments about centroid coordinate axes\n");
      dump.PushIndent();
      dump.Print("xy: %g (+/- %g)\n",m_ccs_xy,m_ccs_xy_err);
      dump.Print("yz: %g (+/- %g)\n",m_ccs_yz,m_ccs_yz_err);
      dump.Print("zx: %g (+/- %g)\n",m_ccs_zx,m_ccs_zx_err);
      dump.PopIndent();
    }

    double Ixx, Iyy, Izz;
    ON_3dVector X, Y, Z;
    if ( CentroidCoordPrincipalMoments( &Ixx, X, &Iyy, Y, &Izz, Z ) )
    {
      dump.Print("Principal moments and axes:\n");
      dump.PushIndent();
      dump.Print("Ixx: %g (%g,%g,%g)\n",Ixx,X.x,X.y,X.z);
      dump.Print("Iyy: %g (%g,%g,%g)\n",Iyy,Y.x,Y.y,Y.z);
      dump.Print("Izz: %g (%g,%g,%g)\n",Izz,Z.x,Z.y,Z.z);
      dump.PopIndent();
    }

    if ( m_bValidSecondMoments && m_bValidMass && m_mass > 0.0 )
    {
      ON_3dVector I, R;
      I = WorldCoordMomentsOfInertia();
      R = WorldCoordRadiiOfGyration();
      dump.Print("Moments of inertia about world coordinate axes\n");
      dump.PushIndent();
      dump.Print("Ix: %g\n",I.x);
      dump.Print("Iy: %g\n",I.y);
      dump.Print("Iz: %g\n",I.z);
      dump.PopIndent();
      dump.Print("Radii of gyration about world coordinate axes\n");
      dump.PushIndent();
      dump.Print("Rx: %g\n",R.x);
      dump.Print("Ry: %g\n",R.y);
      dump.Print("Rz: %g\n",R.z);
      dump.PopIndent();

      I = CentroidCoordMomentsOfInertia();
      R = CentroidCoordRadiiOfGyration();
      dump.Print("Moments of inertia about centroid coordinate axes\n");
      dump.PushIndent();
      dump.Print("Ix: %g\n",I.x);
      dump.Print("Iy: %g\n",I.y);
      dump.Print("Iz: %g\n",I.z);
      dump.PopIndent();
      dump.Print("Radii of gyration about centroid coordinate axes\n");
      dump.PushIndent();
      dump.Print("Rx: %g\n",R.x);
      dump.Print("Ry: %g\n",R.y);
      dump.Print("Rz: %g\n",R.z);
      dump.PopIndent();
    }

    dump.PopIndent();
  }
}




/*
Description:
	QL Algorithm with implict shifts, to determine the eigenvalues and eigenvectors of a 
	symmetric, tridiagonal matrix. 

Parametrers:
	d - [in/out]	On input d[0] to d[n-1] are the diagonal entries of the matrix.
								As output d[0] to d[n-1] are the eigenvalues.
	e - [in/out]  On Input e[0] to e[n-1] are the off-diagonal entries of the matrix.
								with e[n-1] not used, but must be allocated.
								on output e is unpredictable.
	n - [in]      matrix is n by n
	pV - [out]		If not NULL the it should be an n by n matix. 
								The kth column will be a normalized eigenvector of d[k]
*/
static bool TriDiagonalQLImplicit( double* d, double* e, int n, ON_Matrix* pV)
{

	if(pV)
	{	
		if (pV->RowCount()!=n || pV->ColCount()!=n)
			pV = NULL;
	}

	if(pV)
		pV->SetDiagonal(1.0);

	e[n-1]=0.0;

	for( int l=0; l<n; l++)
	{	
		int iter = 0;
		int m;
		do
		{
			for(m=l; m<n-1; m++)
			{
				if( fabs(e[m]) < ON_EPSILON*( fabs( d[m]) + fabs(d[m+1]) ) )
					break;
			}
			if( m!=l)
			{
				if( iter++==30)
					return false;
				double g = ( d[l+1] - d[l])/(2*e[l]);
				double r = sqrt(g*g + 1.0);
				g = d[m] - d[l] + e[l]/((g>=0)? (g + fabs(r)) : (g - fabs(r)) );
				double s = 1.0;
				double c = 1.0;
				double p =0.0;
				int i;
				for( i=m-1; i>=l; i--)
				{
					double f = s * e[i];
					double b = c * e[i];
					r = sqrt( f*f+g*g);
					e[i+1]= r;
					if( r==0.0)
					{
						d[i+1]-= p;
						e[m]=0.0;
						break;
					}
					s = f/r;
					c = g/r;
					g = d[i+1]-p;
					r = (d[i]-g) *s +2.0*c*b;

					p = s*r;
					d[i+1]= g+p;
					g = c*r-b;

					for(int k=0; pV && k<n; k++)
					{
						ON_Matrix & V = *pV;
						f = V[k][i+1];
						V[k][i+1] = s* V[k][i]+c*f;
						V[k][i] =   c* V[k][i]-s*f;
					}
				}
				if( r==0.0 && i>=l) 
					continue;
				d[l] -= p;
				e[l] = g;
				e[m] = 0.0;
			}
		} while( m!=l);
	}
	return true;
}



/*
Description:
  Find the eigen values and eigen vectors of a tri-diagonal
  real symmetric 3x3 matrix

         A D 0
         D B E
         0 E C

Parameters:
  A - [in] matrix entry
  B - [in] matrix entry
  C - [in] matrix entry
  D - [in] matrix entry
  E - [in] matrix entry
  e1 - [out] eigen value
  E1 - [out] eigen vector with eigen value e1
  e2 - [out] eigen value
  E2 - [out] eigen vector with eigen value e2
  e3 - [out] eigen value
  E3 - [out] eigen vector with eigen value e3
Returns:
  True if successful.
*/
bool ON_SymTriDiag3x3EigenSolver( double A, double B, double C,
                           double D, double E,
                           double* e1, ON_3dVector& E1,
                           double* e2, ON_3dVector& E2,
                           double* e3, ON_3dVector& E3
                           )
{
 
	double d[3]={A,B,C};
	double e[3]={D,E,0};

	ON_Matrix V(3,3);
	bool rc = TriDiagonalQLImplicit( d, e, 3, &V);
	if(rc)
	{
		if(e1) *e1 = d[0];
		E1 = ON_3dVector( V[0][0], V[1][0], V[2][0]);
		if(e2) *e2 = d[1];
		E2 = ON_3dVector( V[0][1], V[1][1], V[2][1]);
		if(e3) *e3 = d[2];
		E3 = ON_3dVector( V[0][2], V[1][2], V[2][2]);
	}
	return rc;
}



/*
Description:
  Find the eigen values and eigen vectors of a real symmetric
  3x3 matrix

         A D F
         D B E
         F E C

Parameters:
  A - [in] matrix entry
  B - [in] matrix entry
  C - [in] matrix entry
  D - [in] matrix entry
  E - [in] matrix entry
  F - [in] matrix entry
  e1 - [out] eigen value
  E1 - [out] eigen vector with eigen value e1
  e2 - [out] eigen value
  E2 - [out] eigen vector with eigen value e2
  e3 - [out] eigen value
  E3 - [out] eigen vector with eigen value e3
Returns:
  True if successful.
*/
bool ON_Sym3x3EigenSolver( double A, double B, double C,
                           double D, double E, double F,
                           double* e1, ON_3dVector& E1,
                           double* e2, ON_3dVector& E2,
                           double* e3, ON_3dVector& E3
                           )
{
  // STEP 1: reduce to tri-diagonal form
  double cos_phi = 1.0;
  double sin_phi = 0.0;
  double AA = A, BB = B, CC = C, DD = D, EE = E, FF = F;
  if ( F != 0.0 )
  {
    double theta = 0.5*(C-A)/F;

    double t;
    if ( fabs(theta) > 1.0e154 )
    {
      t = 0.5/fabs(theta);
    }
    else if ( fabs(theta) > 1.0 )
    {
      t = 1.0/(fabs(theta)*(1.0 + sqrt(1.0 + 1.0/(theta*theta))));
    }
    else
    {
      t = 1.0/(fabs(theta) + sqrt(1.0+theta*theta));
    }

    if ( theta < 0.0 )
      t = -t;

    if ( fabs(t) > 1.0 )
    {
      double tt = 1.0/t;
      cos_phi = 1.0/(fabs(t)*sqrt(1.0 + tt*tt));
    }
    else
      cos_phi = 1.0/sqrt(1.0 + t*t);
    
    sin_phi = t*cos_phi;

    double tau = sin_phi/(1.0 + cos_phi);

    AA = A - t*F;
    BB = B;
    CC = C + t*F;
    DD = D - sin_phi*(E + tau*D);
    EE = E + sin_phi*(D + tau*E);

    // debugging test - FF should be close to zero.
    // cos_phi*cos_phi + sin_phi*size_phi should be close to 1
    FF = (cos_phi*cos_phi - sin_phi*sin_phi)*F + sin_phi*cos_phi*(A-C);
  }

  double ee1, ee2, ee3;
  ON_3dVector EE1, EE2, EE3;
  bool rc = ON_SymTriDiag3x3EigenSolver( AA, BB, CC, DD, EE,
                                    &ee1, EE1,
                                    &ee2, EE2,
                                    &ee3, EE3 );

  E1.Set(cos_phi*EE1.x - sin_phi*EE1.z, EE1.y, sin_phi*EE1.x + cos_phi*EE1.z );
  E2.Set(cos_phi*EE2.x - sin_phi*EE2.z, EE2.y, sin_phi*EE2.x + cos_phi*EE2.z );
  E3.Set(cos_phi*EE3.x - sin_phi*EE3.z, EE3.y, sin_phi*EE3.x + cos_phi*EE3.z );
  if ( e1 )
    *e1 = ee1;
  if ( e2 )
    *e2 = ee2;
  if ( e3 )
    *e3 = ee3;

  // debugging check of eigen values
  ON_3dVector err1, err2, err3;
  {
    err1.x = (A*E1.x + D*E1.y + F*E1.z) - ee1*E1.x;
    err1.y = (D*E1.x + B*E1.y + E*E1.z) - ee1*E1.y;
    err1.z = (F*E1.x + E*E1.y + C*E1.z) - ee1*E1.z;

    err2.x = (A*E2.x + D*E2.y + F*E2.z) - ee2*E2.x;
    err2.y = (D*E2.x + B*E2.y + E*E2.z) - ee2*E2.y;
    err2.z = (F*E2.x + E*E2.y + C*E2.z) - ee2*E2.z;

    err3.x = (A*E3.x + D*E3.y + F*E3.z) - ee3*E3.x;
    err3.y = (D*E3.x + B*E3.y + E*E3.z) - ee3*E3.y;
    err3.z = (F*E3.x + E*E3.y + C*E3.z) - ee3*E3.z;
  }

  return rc;
}

ON_MassProperties::ON_MassProperties()
{
  Create();
}

ON_MassProperties::~ON_MassProperties()
{
}

void ON_MassProperties::Create()
{
  memset( this, 0, sizeof(*this) );
}

ON_3dPoint ON_MassProperties::Centroid() const
{
  ON_3dPoint c(0.0,0.0,0.0);
  if ( m_bValidCentroid )
    c.Set( m_x0, m_y0, m_z0 );
  return c;
}

double ON_MassProperties::Length() const
{
  return (m_bValidMass && 1 == m_mass_type) ? m_mass : 0.0;
}

double ON_MassProperties::Area() const
{
  return (m_bValidMass && 2 == m_mass_type) ? m_mass : 0.0;
}

double ON_MassProperties::Volume() const
{
  return (m_bValidMass && 3 == m_mass_type) ? m_mass : 0.0;
}

ON_3dVector ON_MassProperties::WorldCoordFirstMoments() const
{
  ON_3dVector v(0.0,0.0,0.0);
  if ( m_bValidFirstMoments )
    v.Set( m_world_x, m_world_y, m_world_z );
  return v;
}

ON_3dVector ON_MassProperties::WorldCoordSecondMoments() const
{
  ON_3dVector v(0.0,0.0,0.0);
  if ( m_bValidFirstMoments )
    v.Set( m_world_xx, m_world_yy, m_world_zz );
  return v;
}

ON_Matrix* ON_MassProperties::WorldCoordIntertiaMatrix( 
              ON_Matrix* matrix
              ) const
{
  if ( m_bValidSecondMoments && m_bValidProductMoments )
  {
    if ( 0 != matrix )
    {
      if ( 3 != matrix->RowCount() || 3 != matrix->ColCount() )
        matrix->Create(3,3);
    }
    else
    {
      matrix = new ON_Matrix(3,3);
    }
    matrix->m[0][0] = m_world_xx;
    matrix->m[0][1] = m_world_xy;
    matrix->m[0][2] = m_world_zx;
    matrix->m[1][0] = matrix->m[0][1];
    matrix->m[1][1] = m_world_yy;
    matrix->m[1][2] = m_world_yz;
    matrix->m[2][0] = matrix->m[0][2];
    matrix->m[2][1] = matrix->m[1][2];
    matrix->m[2][2] = m_world_zz;
  }
  else
  {
    matrix = 0;
  }
  return matrix;
}

bool ON_MassProperties::WorldCoordPrincipalMoments( 
              double* pxx, ON_3dVector& Ax,
              double* pyy, ON_3dVector& Ay,
              double* pzz, ON_3dVector& Az
              ) const
{
  bool rc = false;
  if ( m_bValidSecondMoments && m_bValidProductMoments )
  {
    rc = ON_Sym3x3EigenSolver( m_world_xx, m_world_yy, m_world_zz,
                               m_world_xy, m_world_yz, m_world_zx,
                               pxx, Ax, pyy, Ay, pzz, Az );
  }
  return rc;
}


ON_3dVector ON_MassProperties::CentroidCoordSecondMoments() const
{
  ON_3dVector v(0.0,0.0,0.0);
  if ( m_bValidSecondMoments )
    v.Set( m_ccs_xx, m_ccs_yy, m_ccs_zz );
  return v;
}


ON_3dVector ON_MassProperties::WorldCoordMomentsOfInertia() const
{
  double Ix = 0.0;
  double Iy = 0.0;
  double Iz = 0.0;
  //double Ix_err = 0.0;
  //double Iy_err = 0.0;
  //double Iz_err = 0.0;
  if ( m_bValidSecondMoments )
  {
    Ix = (m_world_yy + m_world_zz);
    //Ix_err = (m_world_yy_err + m_world_zz_err);
    Iy = (m_world_zz + m_world_xx);
    //Iy_err = (m_world_zz_err + m_world_xx_err);
    Iz = (m_world_xx + m_world_yy);
    //Iz_err = (m_world_xx_err + m_world_yy_err);
  }
  return ON_3dVector(Ix,Iy,Iz);
}


ON_3dVector ON_MassProperties::WorldCoordRadiiOfGyration() const
{
  double Rx = 0.0;
  double Ry = 0.0;
  double Rz = 0.0;
  if ( m_bValidSecondMoments && m_bValidMass && m_mass > 0.0 )
  {
    Rx = sqrt((m_world_yy + m_world_zz)/m_mass);
    Ry = sqrt((m_world_zz + m_world_xx)/m_mass);
    Rz = sqrt((m_world_xx + m_world_yy)/m_mass);
  }
  return ON_3dVector(Rx,Ry,Rz);
}


ON_3dVector ON_MassProperties::CentroidCoordMomentsOfInertia() const
{
  double Ix = 0.0;
  double Iy = 0.0;
  double Iz = 0.0;
  double Ix_err = 0.0;
  double Iy_err = 0.0;
  double Iz_err = 0.0;
  if ( m_bValidSecondMoments )
  {
    Ix = (m_ccs_yy + m_ccs_zz);
    Ix_err = (m_ccs_yy_err + m_ccs_zz_err);
    Iy = (m_ccs_zz + m_ccs_xx);
    Iy_err = (m_ccs_zz_err + m_ccs_xx_err);
    Iz = (m_ccs_xx + m_ccs_yy);
    Iz_err = (m_ccs_xx_err + m_ccs_yy_err);
  }
  return ON_3dVector(Ix,Iy,Iz);
}


ON_3dVector ON_MassProperties::CentroidCoordRadiiOfGyration() const
{
  double Rx = 0.0;
  double Ry = 0.0;
  double Rz = 0.0;
  if ( m_bValidSecondMoments && m_bValidMass && m_mass > 0.0 )
  {
    Rx = sqrt((m_ccs_yy + m_ccs_zz)/m_mass);
    Ry = sqrt((m_ccs_zz + m_ccs_xx)/m_mass);
    Rz = sqrt((m_ccs_xx + m_ccs_yy)/m_mass);
  }
  return ON_3dVector(Rx,Ry,Rz);
}



ON_Matrix* ON_MassProperties::CentroidCoordIntertiaMatrix( 
              ON_Matrix* matrix
              ) const
{
  if ( m_bValidSecondMoments && m_bValidProductMoments )
  {
    if ( 0 != matrix )
    {
      if ( 3 != matrix->RowCount() || 3 != matrix->ColCount() )
        matrix->Create(3,3);
    }
    else
    {
      matrix = new ON_Matrix(3,3);
    }
    matrix->m[0][0] = m_ccs_xx;
    matrix->m[0][1] = m_ccs_xy;
    matrix->m[0][2] = m_ccs_zx;
    matrix->m[1][0] = matrix->m[0][1];
    matrix->m[1][1] = m_ccs_yy;
    matrix->m[1][2] = m_ccs_yz;
    matrix->m[2][0] = matrix->m[0][2];
    matrix->m[2][1] = matrix->m[1][2];
    matrix->m[2][2] = m_ccs_zz;
  }
  else
  {
    matrix = 0;
  }
  return matrix;
}

bool ON_MassProperties::CentroidCoordPrincipalMoments( 
              double* pxx, ON_3dVector& Ax,
              double* pyy, ON_3dVector& Ay,
              double* pzz, ON_3dVector& Az
              ) const
{
  bool rc = false;
  if ( m_bValidSecondMoments && m_bValidProductMoments )
  {
    rc = ON_Sym3x3EigenSolver( m_ccs_xx, m_ccs_yy, m_ccs_zz,
                               m_ccs_xy, m_ccs_yz, m_ccs_zx,
                               pxx, Ax, pyy, Ay, pzz, Az );
  }
  return rc;
}

bool ON_MassProperties::Sum(
    int count,
    const ON_MassProperties* summands,
    bool bAddTo
    )
{
  if ( 0 == m_mass_type )
    bAddTo = false;

  if ( count < 0 )
    return false;

  if( count == 0 )
  {
    if ( !bAddTo )
      Create();
    return true;
  }

  if ( 0 == summands )
    return false;

  if ( bAddTo )
  {
    ON_SimpleArray<ON_MassProperties> mp(count+1);
    mp.Append(*this);
    mp.Append(count,summands);
    return Sum(mp.Count(),mp.Array(),false);
  }

  int mass_type = 0;
  int i;
  for ( i = 0; i < count; i++ )
  {
    if ( summands[i].m_mass_type != 0 )
    {
      if ( 0 == mass_type )
        mass_type = summands[i].m_mass_type;
      else if ( mass_type != summands[i].m_mass_type )
      {
        return false;
      }
    }
  }


  double e;
  ON_Sum x,y,z,ex,ey,ez;

  // mass
  x.Begin();
  ex.Begin();

  //GBA 13-June-08  TRR#35220
  //  Cleaned up error handling.  ON_MassProperties::m_mass_type==0 is treated as an empty record not an error.
  //  If m_bValidMass==false for any of the summands then m_bValidMass==false for the result ( and similary for
  //  m_bValidFirstMoments, m_bValidSecondMoments and m_bValidProductMoments). 

  m_bValidMass = true;
  for ( i = 0; i < count; i++ )
  {
    if ( 0 == summands[i].m_mass_type)
      continue;

    if( summands[i].m_bValidMass )
    {
      if ( 0 == mass_type )
        mass_type = summands[i].m_mass_type;
      else if ( mass_type != summands[i].m_mass_type )
      {
        return false;
      }
      x.Plus(summands[i].m_mass);
      ex.Plus(summands[i].m_mass_err);
    }
    else 
      m_bValidMass = false;
    
  }

  const int c = x.SummandCount();

  if ( 0 == mass_type || 0 == c )
  {
    Create();
    return true;
  }

  m_mass_type = mass_type;

  if(m_bValidMass)
  {
    m_mass = x.Total(&e);
    m_mass_err = ex.Total()+e;
    if ( e > m_mass_err )
      m_mass_err = e;
  }
  else
    return true;

  // first moments;
  x.Begin();
  ex.Begin();
  y.Begin();
  ey.Begin();
  z.Begin();
  ez.Begin();
  
  m_bValidFirstMoments = true;

  for ( i = 0; i < count; i++ )
  {
    if ( 0 == summands[i].m_mass_type )
      continue;
    
    if( summands[i].m_bValidMass
        && summands[i].m_bValidFirstMoments
       )
    {
      x.Plus(summands[i].m_world_x);
      ex.Plus(summands[i].m_world_x_err);
      y.Plus(summands[i].m_world_y);
      ey.Plus(summands[i].m_world_y_err);
      z.Plus(summands[i].m_world_z);
      ez.Plus(summands[i].m_world_z_err);
    }
    else
      m_bValidFirstMoments = false;
  }

  if ( x.SummandCount() == 0 )
    return true;

  if ( x.SummandCount() != c )
    return false;

  if( m_bValidFirstMoments)
  {
    m_world_x = x.Total(&e);
    m_world_x_err = ex.Total()+e;
    m_world_y = y.Total(&e);
    m_world_y_err = ey.Total()+e;
    m_world_z = z.Total(&e);
    m_world_z_err = ez.Total()+e;
  }

  // centroid
  if ( 0.0 == m_mass )
    return false;

  if( m_bValidFirstMoments)
  {
    m_x0 = m_world_x/m_mass;
    m_x0_err = (m_world_x_err + fabs(m_world_x)*m_mass_err/m_mass)/m_mass;
    m_y0 = m_world_y/m_mass;
    m_y0_err = (m_world_y_err + fabs(m_world_y)*m_mass_err/m_mass)/m_mass;
    m_z0 = m_world_z/m_mass;
    m_z0_err = (m_world_z_err + fabs(m_world_z)*m_mass_err/m_mass)/m_mass;
    m_bValidCentroid = true;
  }

  // second moments
  double dx, dy, dz, dx_err, dy_err, dz_err, m, m_err;

  x.Begin();
  ex.Begin();
  y.Begin();
  ey.Begin();
  z.Begin();
  ez.Begin();
  
  m_bValidSecondMoments = true;

  for ( i = 0; i < count; i++ )
  {
    if ( 0 == summands[i].m_mass_type )
      continue;
    
    if( summands[i].m_bValidMass
        && summands[i].m_bValidCentroid
        && summands[i].m_bValidSecondMoments
       )
    {
      dx = summands[i].m_x0 - m_x0;
      dx_err = summands[i].m_x0_err + m_x0_err;
      dy = summands[i].m_y0 - m_y0;
      dy_err = summands[i].m_y0_err + m_y0_err;
      dz = summands[i].m_z0 - m_z0;
      dz_err = summands[i].m_z0_err + m_z0_err;
      m = summands[i].m_mass;
      m_err = summands[i].m_mass_err;

      x.Plus(summands[i].m_ccs_xx + dx*dx*m);
      ex.Plus(summands[i].m_ccs_xx_err + 2.0*dx_err*fabs(dx)*m + dx*dx*m_err);
      y.Plus(summands[i].m_ccs_yy + dy*dy*m);
      ey.Plus(summands[i].m_ccs_yy_err + 2.0*dy_err*fabs(dy)*m + dy*dy*m_err);
      z.Plus(summands[i].m_ccs_zz + dz*dz*m);
      ez.Plus(summands[i].m_ccs_zz_err + 2.0*dz_err*fabs(dz)*m + dz*dz*m_err);
    }
    else
      m_bValidSecondMoments = false;
    
  }

  if ( x.SummandCount() > 0 )
  {
    if ( x.SummandCount() != c )
      return false;
    if( m_bValidSecondMoments)
    {
      m_ccs_xx = x.Total(&e);
      m_ccs_xx_err = ex.Total() + e;
      m_ccs_yy = y.Total(&e);
      m_ccs_yy_err = ey.Total() + e;
      m_ccs_zz = z.Total(&e);
      m_ccs_zz_err = ez.Total() + e;

      m_world_xx = m_ccs_xx + m_x0*m_x0*m_mass;
      m_world_xx_err = m_ccs_xx_err + 2.0*m_x0_err*fabs(m_x0)*m_mass + m_x0*m_x0*m_mass_err;
      m_world_yy = m_ccs_yy + m_y0*m_y0*m_mass;
      m_world_yy_err = m_ccs_yy_err + 2.0*m_y0_err*fabs(m_y0)*m_mass + m_y0*m_y0*m_mass_err;
      m_world_zz = m_ccs_zz + m_z0*m_z0*m_mass;
      m_world_zz_err = m_ccs_zz_err + 2.0*m_z0_err*fabs(m_z0)*m_mass + m_z0*m_z0*m_mass_err;
    }  
  }

  // product moments
  x.Begin();
  ex.Begin();
  y.Begin();
  ey.Begin();
  z.Begin();
  ez.Begin();
  
  m_bValidProductMoments = true;

  for ( i = 0; i < count; i++ )
  {
    if ( 0 == summands[i].m_mass_type )
      continue;
    
    if( summands[i].m_bValidMass
        && summands[i].m_bValidCentroid
        && summands[i].m_bValidSecondMoments
       )
    {
      dx = summands[i].m_x0 - m_x0;
      dx_err = summands[i].m_x0_err + m_x0_err;
      dy = summands[i].m_y0 - m_y0;
      dy_err = summands[i].m_y0_err + m_y0_err;
      dz = summands[i].m_z0 - m_z0;
      dz_err = summands[i].m_z0_err + m_z0_err;
      m = summands[i].m_mass;
      m_err = summands[i].m_mass_err;

      x.Plus(summands[i].m_ccs_xy + dx*dy*m);
      ex.Plus(summands[i].m_ccs_xy_err + fabs(dx_err*dy*m) + fabs(dy_err*dx*m) + fabs(dx*dy*m_err));
      y.Plus(summands[i].m_ccs_yz + dy*dz*m);
      ey.Plus(summands[i].m_ccs_yz_err + fabs(dy_err*dz*m) + fabs(dz_err*dy*m) + fabs(dy*dz*m_err));
      z.Plus(summands[i].m_ccs_zx + dz*dx*m);
      ez.Plus(summands[i].m_ccs_zx_err + fabs(dz_err*dx*m) + fabs(dx_err*dz*m) + fabs(dz*dx*m_err));
    }
    else
      m_bValidProductMoments = false;
    
  }

  if ( x.SummandCount() > 0 )
  {
    if ( x.SummandCount() != c )
      return false;
    if( m_bValidProductMoments)
    {
      m_ccs_xy = x.Total(&e);
      m_ccs_xy_err = ex.Total() + e;
      m_ccs_yz = y.Total(&e);
      m_ccs_yz_err = ey.Total() + e;
      m_ccs_zx = z.Total(&e);
      m_ccs_zx_err = ez.Total() + e;

      m_world_xy = m_ccs_xy + m_x0*m_y0*m_mass;
      m_world_xy_err = m_ccs_xy_err + fabs(m_x0_err*m_y0*m_mass) + fabs(m_y0_err*m_x0*m_mass) + fabs(m_x0*m_y0*m_mass_err);
      m_world_yz = m_ccs_yz + m_y0*m_z0*m_mass;
      m_world_yz_err = m_ccs_yz_err + fabs(m_y0_err*m_z0*m_mass) + fabs(m_z0_err*m_y0*m_mass) + fabs(m_y0*m_z0*m_mass_err);
      m_world_zx = m_ccs_zx + m_z0*m_x0*m_mass;
      m_world_zx_err = m_ccs_zx_err + fabs(m_z0_err*m_x0*m_mass) + fabs(m_x0_err*m_z0*m_mass) + fabs(m_z0*m_x0*m_mass_err);
    } 
  }

  return true;
}
