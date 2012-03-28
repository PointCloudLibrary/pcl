/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2006, Michael Kazhdan and Matthew Bolitho,
 *                      Johns Hopkins University
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <float.h>
#include <math.h>
#include <algorithm>

#include <pcl/surface/poisson/factor.h>


namespace pcl {
  namespace poisson {


    ////////////////
    // Polynomial //
    ////////////////
    template<int Degree>
    Polynomial<Degree>::Polynomial(void){memset(coefficients,0,sizeof(double)*(Degree+1));}
    template<int Degree>
    template<int Degree2>
    Polynomial<Degree>::Polynomial(const Polynomial<Degree2>& P){
      memset(coefficients,0,sizeof(double)*(Degree+1));
      for(int i=0;i<=Degree && i<=Degree2;i++){coefficients[i]=P.coefficients[i];}
    }


    template<int Degree>
    template<int Degree2>
    Polynomial<Degree>& Polynomial<Degree>::operator  = (const Polynomial<Degree2> &p){
      int d=Degree<Degree2?Degree:Degree2;
      memset(coefficients,0,sizeof(double)*(Degree+1));
      memcpy(coefficients,p.coefficients,sizeof(double)*(d+1));
      return *this;
    }

    template<int Degree>
    Polynomial<Degree-1> Polynomial<Degree>::derivative(void) const{
      Polynomial<Degree-1> p;
      for(int i=0;i<Degree;i++){p.coefficients[i]=coefficients[i+1]*(i+1);}
      return p;
    }

    template<int Degree>
    Polynomial<Degree+1> Polynomial<Degree>::integral(void) const{
      Polynomial<Degree+1> p;
      p.coefficients[0]=0;
      for(int i=0;i<=Degree;i++){p.coefficients[i+1]=coefficients[i]/(i+1);}
      return p;
    }
    template<int Degree>
    double Polynomial<Degree>::operator() (const double& t) const{
      double temp=1;
      double v=0;
      for(int i=0;i<=Degree;i++){
        v+=temp*coefficients[i];
        temp*=t;
      }
      return v;
    }
    template<int Degree>
    double Polynomial<Degree>::integral(const double& tMin,const double& tMax) const{
      double v=0;
      double t1,t2;
      t1=tMin;
      t2=tMax;
      for(int i=0;i<=Degree;i++){
        v+=coefficients[i]*(t2-t1)/(i+1);
        if(t1!=-DBL_MAX && t1!=DBL_MAX){t1*=tMin;}
        if(t2!=-DBL_MAX && t2!=DBL_MAX){t2*=tMax;}
      }
      return v;
    }
    template<int Degree>
    int Polynomial<Degree>::operator == (const Polynomial& p) const{
      for(int i=0;i<=Degree;i++){if(coefficients[i]!=p.coefficients[i]){return 0;}}
      return 1;
    }
    template<int Degree>
    int Polynomial<Degree>::operator != (const Polynomial& p) const{
      for(int i=0;i<=Degree;i++){if(coefficients[i]==p.coefficients[i]){return 0;}}
      return 1;
    }
    template<int Degree>
    int Polynomial<Degree>::isZero(void) const{
      for(int i=0;i<=Degree;i++){if(coefficients[i]!=0){return 0;}}
      return 1;
    }
    template<int Degree>
    void Polynomial<Degree>::setZero(void){memset(coefficients,0,sizeof(double)*(Degree+1));}

    template<int Degree>
    Polynomial<Degree>& Polynomial<Degree>::addScaled(const Polynomial& p,const double& s){
      for(int i=0;i<=Degree;i++){coefficients[i]+=p.coefficients[i]*s;}
      return *this;
    }
    template<int Degree>
    Polynomial<Degree>& Polynomial<Degree>::operator += (const Polynomial<Degree>& p){
      for(int i=0;i<=Degree;i++){coefficients[i]+=p.coefficients[i];}
      return *this;
    }
    template<int Degree>
    Polynomial<Degree>& Polynomial<Degree>::operator -= (const Polynomial<Degree>& p){
      for(int i=0;i<=Degree;i++){coefficients[i]-=p.coefficients[i];}
      return *this;
    }
    template<int Degree>
    Polynomial<Degree> Polynomial<Degree>::operator + (const Polynomial<Degree>& p) const{
      Polynomial q;
      for(int i=0;i<=Degree;i++){q.coefficients[i]=(coefficients[i]+p.coefficients[i]);}
      return q;
    }
    template<int Degree>
    Polynomial<Degree> Polynomial<Degree>::operator - (const Polynomial<Degree>& p) const{
      Polynomial q;
      for(int i=0;i<=Degree;i++)	{q.coefficients[i]=coefficients[i]-p.coefficients[i];}
      return q;
    }
    template<int Degree>
    void Polynomial<Degree>::Scale(const Polynomial& p,const double& w,Polynomial& q){
      for(int i=0;i<=Degree;i++){q.coefficients[i]=p.coefficients[i]*w;}
    }
    template<int Degree>
    void Polynomial<Degree>::AddScaled(const Polynomial& p1,const double& w1,const Polynomial& p2,const double& w2,Polynomial& q){
      for(int i=0;i<=Degree;i++){q.coefficients[i]=p1.coefficients[i]*w1+p2.coefficients[i]*w2;}
    }
    template<int Degree>
    void Polynomial<Degree>::AddScaled(const Polynomial& p1,const double& w1,const Polynomial& p2,Polynomial& q){
      for(int i=0;i<=Degree;i++){q.coefficients[i]=p1.coefficients[i]*w1+p2.coefficients[i];}
    }
    template<int Degree>
    void Polynomial<Degree>::AddScaled(const Polynomial& p1,const Polynomial& p2,const double& w2,Polynomial& q){
      for(int i=0;i<=Degree;i++){q.coefficients[i]=p1.coefficients[i]+p2.coefficients[i]*w2;}
    }

    template<int Degree>
    void Polynomial<Degree>::Subtract(const Polynomial &p1,const Polynomial& p2,Polynomial& q){
      for(int i=0;i<=Degree;i++){q.coefficients[i]=p1.coefficients[i]-p2.coefficients[i];}
    }
    template<int Degree>
    void Polynomial<Degree>::Negate(const Polynomial& in,Polynomial& out){
      out=in;
      for(int i=0;i<=Degree;i++){out.coefficients[i]=-out.coefficients[i];}
    }

    template<int Degree>
    Polynomial<Degree> Polynomial<Degree>::operator - (void) const{
      Polynomial q=*this;
      for(int i=0;i<=Degree;i++){q.coefficients[i]=-q.coefficients[i];}
      return q;
    }
    template<int Degree>
    template<int Degree2>
    Polynomial<Degree+Degree2> Polynomial<Degree>::operator * (const Polynomial<Degree2>& p) const{
      Polynomial<Degree+Degree2> q;
      for(int i=0;i<=Degree;i++){for(int j=0;j<=Degree2;j++){q.coefficients[i+j]+=coefficients[i]*p.coefficients[j];}}
      return q;
    }

    template<int Degree>
    Polynomial<Degree>& Polynomial<Degree>::operator += (const double& s){
      coefficients[0]+=s;
      return *this;
    }
    template<int Degree>
    Polynomial<Degree>& Polynomial<Degree>::operator -= (const double& s){
      coefficients[0]-=s;
      return *this;
    }
    template<int Degree>
    Polynomial<Degree>& Polynomial<Degree>::operator *= (const double& s){
      for(int i=0;i<=Degree;i++){coefficients[i]*=s;}
      return *this;
    }
    template<int Degree>
    Polynomial<Degree>& Polynomial<Degree>::operator /= (const double& s){
      for(int i=0;i<=Degree;i++){coefficients[i]/=s;}
      return *this;
    }
    template<int Degree>
    Polynomial<Degree> Polynomial<Degree>::operator + (const double& s) const{
      Polynomial<Degree> q=*this;
      q.coefficients[0]+=s;
      return q;
    }
    template<int Degree>
    Polynomial<Degree> Polynomial<Degree>::operator - (const double& s) const{
      Polynomial q=*this;
      q.coefficients[0]-=s;
      return q;
    }
    template<int Degree>
    Polynomial<Degree> Polynomial<Degree>::operator * (const double& s) const{
      Polynomial q;
      for(int i=0;i<=Degree;i++){q.coefficients[i]=coefficients[i]*s;}
      return q;
    }
    template<int Degree>
    Polynomial<Degree> Polynomial<Degree>::operator / (const double& s) const{
      Polynomial q(this->degree());
      for(int i=0;i<=Degree;i++){q.coefficients[i]=coefficients[i]/s;}
      return q;
    }
    template<int Degree>
    Polynomial<Degree> Polynomial<Degree>::scale(const double& s) const{
      Polynomial q=*this;
      double s2=1.0;
      for(int i=0;i<=Degree;i++){
        q.coefficients[i]*=s2;
        s2/=s;
      }
      return q;
    }
    template<int Degree>
    Polynomial<Degree> Polynomial<Degree>::shift(const double& t) const{
      Polynomial<Degree> q;
      for(int i=0;i<=Degree;i++){
        double temp=1;
        for(int j=i;j>=0;j--){
          q.coefficients[j]+=coefficients[i]*temp;
          temp*=-t*j;
          temp/=(i-j+1);
        }
      }
      return q;
    }
    template<int Degree>
    void Polynomial<Degree>::printnl(void) const{
      for(int j=0;j<=Degree;j++){
        printf("%6.4f x^%d ",coefficients[j],j);
        if(j<Degree && coefficients[j+1]>=0){printf("+");}
      }
      printf("\n");
    }
    template<int Degree>
    void Polynomial<Degree>::getSolutions(const double& c,std::vector<double>& roots,const double& EPS) const {
      double r[4][2];
      int rCount=0;
      roots.clear();
      switch(Degree){
        case 1:
          rCount=Factor(coefficients[1],coefficients[0]-c,r,EPS);
          break;
        case 2:
          rCount=Factor(coefficients[2],coefficients[1],coefficients[0]-c,r,EPS);
          break;
        case 3:
          rCount=Factor(coefficients[3],coefficients[2],coefficients[1],coefficients[0]-c,r,EPS);
          break;
          //	case 4:
          //		rCount=Factor(coefficients[4],coefficients[3],coefficients[2],coefficients[1],coefficients[0]-c,r,EPS);
          //		break;
        default:
          printf("Can't solve polynomial of degree: %d\n",Degree);
      }
      for(int i=0;i<rCount;i++){
        if(fabs(r[i][1])<=EPS){
          roots.push_back(r[i][0]);
          //printf("%d] %f\t%f\n",i,r[i][0],(*this)(r[i][0])-c);
        }
      }
    }


  }
}
