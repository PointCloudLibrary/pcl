/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 * 
 *
 */

#include <pcl/surface/on_nurbs/sparse_mat.h>

using namespace pcl;
using namespace on_nurbs;

void
SparseMat::get (std::vector<int> &i, std::vector<int> &j, std::vector<double> &v)
{
  std::map<int, std::map<int, double> >::iterator it_row;
  std::map<int, double>::iterator it_col;

  i.clear ();
  j.clear ();
  v.clear ();

  it_row = m_mat.begin ();
  while (it_row != m_mat.end ())
  {
    it_col = it_row->second.begin ();
    while (it_col != it_row->second.end ())
    {
      i.push_back (it_row->first);
      j.push_back (it_col->first);
      v.push_back (it_col->second);
      ++it_col;
    }
    ++it_row;
  }

}

double
SparseMat::get (int i, int j)
{
  std::map<int, std::map<int, double> >::iterator it_row;
  std::map<int, double>::iterator it_col;

  it_row = m_mat.find (i);
  if (it_row == m_mat.end ())
    return 0.0;

  it_col = it_row->second.find (j);
  if (it_col == it_row->second.end ())
    return 0.0;

  return it_col->second;

}

void
SparseMat::set (int i, int j, double v)
{

  if (i < 0 || j < 0)
  {
    printf ("[SparseMat::set] Warning index out of bounds (%d,%d)\n", i, j);
    return;
  }

  if (v == 0.0)
  {
    // delete entry

    std::map<int, std::map<int, double> >::iterator it_row;
    std::map<int, double>::iterator it_col;

    it_row = m_mat.find (i);
    if (it_row == m_mat.end ())
      return;

    it_col = it_row->second.find (j);
    if (it_col == it_row->second.end ())
      return;

    it_row->second.erase (it_col);
    if (it_row->second.empty ())
    {}
    m_mat.erase (it_row);

  }
  else
  {
    // update entry
    m_mat[i][j] = v;

  }

}

void
SparseMat::deleteRow (int i)
{

  std::map<int, std::map<int, double> >::iterator it_row;

  it_row = m_mat.find (i);
  if (it_row != m_mat.end ())
    m_mat.erase (it_row);

}

void
SparseMat::deleteColumn (int j)
{
  std::map<int, std::map<int, double> >::iterator it_row;
  std::map<int, double>::iterator it_col;

  it_row = m_mat.begin ();
  while (it_row != m_mat.end ())
  {
    it_col = it_row->second.find (j);
    if (it_col != it_row->second.end ())
      it_row->second.erase (it_col);
    ++it_row;
  }

}

void
SparseMat::size (int &si, int &sj)
{
  std::map<int, std::map<int, double> >::iterator it_row;
  std::map<int, double>::iterator it_col;

  if (m_mat.empty ())
  {
    si = 0;
    sj = 0;
    return;
  }

  si = 0;
  sj = 0;

  it_row = m_mat.begin ();
  while (it_row != m_mat.end ())
  {
    it_col = it_row->second.end ();
    --it_col;
    if (sj < ((*it_col).first + 1))
      sj = (*it_col).first + 1;

    ++it_row;
  }

  it_row = m_mat.end ();
  --it_row;
  si = (*it_row).first + 1;

}

int
SparseMat::nonzeros ()
{
  std::map<int, std::map<int, double> >::iterator it_row;
  int s = 0;

  it_row = m_mat.begin ();
  while (it_row != m_mat.end ())
  {
    s += int (it_row->second.size ());

    ++it_row;
  }

  return s;

}

void
SparseMat::printLong ()
{
  std::map<int, std::map<int, double> >::iterator it_row;
  std::map<int, double>::iterator it_col;

  int si, sj;
  size (si, sj);

  for (int i = 0; i < si; i++)
  {
    for (int j = 0; j < sj; j++)
    {
      printf ("%f ", get (i, j));
    }
    printf ("\n");
  }
}

void
SparseMat::print ()
{
  std::map<int, std::map<int, double> >::iterator it_row;
  std::map<int, double>::iterator it_col;

  it_row = m_mat.begin ();
  while (it_row != m_mat.end ())
  {
    it_col = it_row->second.begin ();
    while (it_col != it_row->second.end ())
    {
      printf ("[%d,%d] %f ", it_row->first, it_col->first, it_col->second);
      ++it_col;
    }
    printf ("\n");
    ++it_row;
  }
}

