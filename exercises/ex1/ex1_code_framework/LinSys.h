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

class Vector {
public:
   Vector( int size = 0   );
   Vector( const Vector & );
   Vector( const double *x, int n );
   Vector &operator=( const Vector & );
   void    operator=( double );
   void    SetSize( int );
   static  const Vector Null;

public: // Inlined functions.
   inline double  operator()( int i ) const { return elem[i]; }
   inline double& operator()( int i )       { return elem[i]; }
   inline double* Array() const { return elem; }
   inline int    Size () const { return size; }
   inline ~Vector() { delete[] elem; }

private:
   void   Create( int n = 0 ) { size = n; elem = new double[n]; }
   int    size;
   double* elem;
};

class MatrixMN {
public:
   MatrixMN( const MatrixMN & );
   MatrixMN( int num_rows = 0, int num_cols = 0, double value = 0.0 );
   ~MatrixMN();
   MatrixMN &operator=( const MatrixMN &M );
   MatrixMN &operator=( double s );
   void    SetSize( int rows, int cols = 0 );
   static  const MatrixMN Null;

public: // Inlined functions.
   inline double  operator()( int i, int j ) const { return elem[ i * cols + j ]; }
   inline double &operator()( int i, int j )       { return elem[ i * cols + j ]; }
   inline int    Rows  () const { return rows; }
   inline int    Cols  () const { return cols; }
   inline double *Array () const { return elem; }

private:
   int    rows; // Number of rows in the matrix.
   int    cols; // Number of columns in the matrix.
   double *elem; // Pointer to the actual data.
};

int GaussElimination(const MatrixMN &A, 
                     const Vector &b, // This is the right-hand side.
                     Vector       &x // This is the matrix we are solving for.
                     );


