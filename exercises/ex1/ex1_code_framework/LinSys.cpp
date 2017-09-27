/*-------------------------------------------------------------------------*
* Copyright (C) 2000, James Arvo                                           *
*                                                                          *
* This program is free software; you can redistribute it and/or modify it  *
* under the terms of the GNU General Public License as published by the    *
* Free Software Foundation.  See http://www.fsf.org/copyleft/gpl.html      *
*                                                                          *
* This program is distributed in the hope that it will be useful, but      *
* WITHOUT EXPRESS OR IMPLIED WARRANTY of merchantability or fitness for    *
* any particular purpose.  See the GNU General Public License for more     *
* details.                                                                 *
*                                                                          *
***************************************************************************/

#include "LinSys.h"
#include <stdlib.h>

Vector::Vector( const double *x, int n )
{
   Create( n );
   for( register int i = 0; i < size; i++ ) elem[i] = x[i];
}

Vector::Vector( const Vector &A )
{
   Create( A.Size() );
   for( register int i = 0; i < A.Size(); i++ ) elem[i] = A(i);
}

Vector::Vector( int n )
{
   Create( n );
   for( register int i = 0; i < n; i++ ) elem[i] = 0.0;
}

void Vector::SetSize( int new_size )
{
   if( size != new_size )
   {
      delete[] elem;
      Create( new_size );
      for( register int i = 0; i < new_size; i++ ) elem[i] = 0.0;
   }
}

// Create a new matrix of the given size.  If n_cols is zero (the default), 
// it is assumed that the matrix is to be square; that is, n_rows x n_rows.  
// The matrix is filled with "value", which defaults to zero.
MatrixMN::MatrixMN( int n_rows, int n_cols, double value ) 
{
   rows = 0;
   cols = 0;
   elem = NULL;
   SetSize( n_rows, n_cols );
   double *e = elem;
   for( register int i = 0; i < rows * cols; i++ ) *e++ = value;
}

// Copy constructor.
MatrixMN::MatrixMN( const MatrixMN &M ) 
{
   rows = 0;
   cols = 0;
   elem = NULL;
   SetSize( M.Rows(), M.Cols() );
   register double *e = elem;
   register double *m = M.Array();
   for( register int i = 0; i < rows * cols; i++ ) *e++ = *m++;
}

MatrixMN::~MatrixMN() 
{
   SetSize( 0, 0 );
}

// Re-shape the matrix.  If the number of elements in the new matrix is
// different from the original matrix, the original data is deleted and
// replaced with a new array.  If new_cols is zero (the default), it is
// assumed to be the same as new_rows -- i.e. a square matrix.
void MatrixMN::SetSize( int new_rows, int new_cols )
{
   if( new_cols == 0 ) new_cols = new_rows;
   int n = new_rows * new_cols;
   if( rows * cols != n )
   {
      if( elem != NULL ) delete[] elem;
      elem = ( n == 0 ) ? NULL : new double[ n ];
   }
   rows = new_rows;
   cols = new_cols;
}

int GaussElimination( const MatrixMN &A, const Vector &b, Vector &x)
{
   MatrixMN B( A );
   Vector c( b );
   x.SetSize( A.Cols() );
   int m = B.Rows();
   register int i, j, k;

   // Perform Gaussian elimination on the copies, B and c.

   for( i = 0; i < m; i++ )
   {
      for( j = i + 1; j < m; j++ )
      {
         double scale = -B(j,i) / B(i,i);
         for( k = i; k < m; k++ )
            B(j,k) += scale * B(i,k);
         B(j,i) = 0.0;
         c(j) += scale * c(i);
      }
   }

   // Now solve by back substitution.

   for( i = m - 1; i >= 0; i-- )
   {
      double a = 0.0;
      for( j = i + 1; j < m; j++ ) a += B(i,j) * x(j);
      x(i) = ( c(i) - a ) / B(i,i);
   }

   return 1;
}

