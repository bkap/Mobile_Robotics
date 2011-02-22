/*
 * Random.cpp
 *
 *  Created on: Nov 13, 2009
 *      Author: wes
 */
#include "MathyStuff.h"
#include "Includes.h"

unsigned int A, B, C, D;

void InitializeRand()
{
	::A = rand();
	cout<<::A<<"\n";
	::B = rand();
	cout<<::B<<"\n";
	::C = rand();
	cout<<::C<<"\n";
	::D = rand();
	cout<<::D<<"\n";
}

unsigned int Rand ()
{

	unsigned int t = A ^ (A << 11);
	::A = ::B;
	::B = ::C;
	::C = ::D;
	::D = (::D ^ (::D >> 19)) ^ (t ^ (t >> 8));
	return ::D;

}


FLOAT Magnitude2 (FLOAT X, FLOAT Y)
{
	return sqrt(X*X+Y*Y);
}
FLOAT Magnitude3 (FLOAT X, FLOAT Y, FLOAT Z)
{
	return sqrt(X*X+Y*Y+Z*Z);
}
FLOAT Magnitude2 (Point2 A)
{
	return sqrt(A.X*A.X+A.Y*A.Y);
}
FLOAT Magnitude3 (Point3 A)
{
	return sqrt(Dot3(A,A));
}
FLOAT Magnitude3 (Particle A)
{
	return sqrt(Dot3(A,A));
}
Point3 Normalize (Point3 A)
{
	return A/Magnitude3(A);
}
FLOAT Distance2 (Point2 A, Point2 B)
{
	return Magnitude2(Point2(A.X-B.X, A.Y-B.Y));
}
FLOAT Distance3 (Point3 A, Point3 B)
{
	return Magnitude3(A-B);
}
FLOAT Distance3 (Particle A, Particle B)
{
	return Magnitude3(A-B);
}
FLOAT Distance3 (Particle A, Point3 B)
{
	return Magnitude3(A-B);
}
FLOAT Distance2(FLOAT X1, FLOAT X2,  FLOAT Y1, FLOAT Y2)
{
	X1= X2-X1;
	Y1= Y2-Y1;
	return sqrt(X1*X1+Y1*Y1);
}
FLOAT Distance3( FLOAT X1, FLOAT X2,FLOAT Y1, FLOAT Y2, FLOAT Z1, FLOAT Z2)
{
	X1 = X2-X1;
	Y1 = Y2-Y1;
	Z1 = Z2-Z1;
	return sqrt(X1*X1+Y1*Y1+Z1*Z1);
}
FLOAT MagnitudeSquared2 (FLOAT X, FLOAT Y)
{
	//3 flop
	return (X*X+Y*Y);
}
FLOAT MagnitudeSquared3 (FLOAT X, FLOAT Y, FLOAT Z)
{
	//5 flop
	Z=(X*X+Y*Y+Z*Z);
	return Z;
}
FLOAT MagnitudeSquared2 (Point2 A)
{
	return A.X*A.X+A.Y*A.Y;
}
FLOAT MagnitudeSquared3 (Point3 A)
{
	return A.X*A.X+A.Y*A.Y+A.Z*A.Z;
}
FLOAT DistanceSquared3(Point3 A, Point3 B)
{
	return Dot3((A-B), (A-B));
}
FLOAT DistanceSquared2(FLOAT X1, FLOAT X2,  FLOAT Y1, FLOAT Y2)
{
	//5 flop
	X2 = X2-X1;
	Y2 = Y2-Y1;
	return X2*X2+Y2*Y2;
}
FLOAT DistanceSquared3( FLOAT X1, FLOAT X2,FLOAT Y1, FLOAT Y2, FLOAT Z1, FLOAT Z2)
{
	//8 flop
	X1 = X2-X1;
	Y1 = Y2-Y1;
	Z1 = Z2-Z1;
	return Z1*Z1+Y1*Y1+X1*X1;
}
FLOAT DistanceSquared3(Particle A, Particle B)
{
	return (A.X-B.X)*(A.X-B.X)+(A.Y-B.Y)*(A.Y-B.Y)+(A.Z-B.Z)*(A.Z-B.Z);
}
FLOAT Dot2 (Point2 A, Point2 B)
{
	return A.X*B.X+A.Y*B.Y;
}
FLOAT Dot3 (Point3 A, Point3 B)
{
	return A.X*B.X+A.Y*B.Y+A.Z*B.Z;
}
FLOAT Dot3 (Particle A, Particle B)
{
	return A.X*B.X+A.Y*B.Y+A.Z*B.Z;
}
FLOAT Dot3 (Particle A, Point3 B)
{
	return A.X*B.X+A.Y*B.Y+A.Z*B.Z;
}
FLOAT Dot3 (Point3 A, Particle B)
{
	return A.X*B.X+A.Y*B.Y+A.Z*B.Z;
}
Point3 Cross3 (Point3 A, Point3 B)
{
	return Point3(A.Y*B.Z-A.Z*B.Y, A.Z*B.X-A.X*B.Z, A.X*B.Y-A.Y*B.X);
}
double StdDev (double SumX, double SumX2, double N)
{
	return sqrt(SumX2/N-SumX/N*SumX/N);
}

FLOAT Vfrac(int NumHydronium, int r)
{
	FLOAT R1 = GetRadius(r);
	FLOAT R2 = GetRadius(r+1);

	return NumHydronium*(2.0/3.0)*(MinSeparation*MinSeparation*MinSeparation)/(LengthOfCylinder*(R2*R2-R1*R1));;
}

//this is the number of indices needed to represent all of the buckets used in data collection
//as a double check, this should be the same as get index for the Radius of the cylinder+1.
int NumberOfIndices ()
{
	if (RSquaredDataCollect)
	{
		return (int)sqrt(RadiusOfCylinder/DataCollectSize)+1;
	}
	else
	{
		return (int)(RadiusOfCylinder/DataCollectSize)+1;
	}
}

//returns the radius of a given index in system units<-this is the greatest r of the index
FLOAT GetRadius(int index)
{
	if (RSquaredDataCollect)
	{
		return (FLOAT)((index+1)*(index+1))*DataCollectSize;
	}
	else
	{
		return (FLOAT)(index+1)*(DataCollectSize);
	}
}

//returns the radius of a given index in nm.<-this is the greatest r of the index
FLOAT GetRadiusinnm(int index)
{
		return GetRadius(index)*deltaD;
}

//returns the index that would store a given radius in system units
int GetIndex (FLOAT R)
{
	if (RSquaredDataCollect)
	{
	return (int)sqrt(R/DataCollectSize);
	}
	else
	{
	return (int)(R/DataCollectSize);
	}
}

//returns the index that would store a given radius in nm
int GetIndexinnm (FLOAT R)
{
	return GetIndex(R)*deltaD;
}

Point3 YfromZ(Point3 Z)
{
	if (Z.Z<.0000001&&Z.Z>-.0000001)
	{
		Z.Z=.00001;
		Point3 R (Z.X, Z.Y, 0);
		R= R/(sqrt(Z.X*Z.X+Z.Y*Z.Y));
		R = Cross3(Z,R);
		return R/Magnitude3(R);

	}
	else if(Z.X*Z.X+Z.Y*Z.Y<.00001)
	{
		Point3 R(1,1,0);
		R= Cross3(Z,R);
		return R/Magnitude3(R);

	}
	else
	{
		Point3 R (Z.X, Z.Y, 0);
		R= R/(sqrt(Z.X*Z.X+Z.Y*Z.Y));
		R = Cross3(Z,R);
		return R/Magnitude3(R);
	}

}
Point3 XfromYZ (Point3 Y, Point3 Z)
{
	Y = Cross3(Y,Z);
	return Y/Magnitude3(Y);
}

ostream& operator<<(ostream& output, Point3 P)
{
	output<<"P(x,y,z) = ("<<P.X<<","<<P.Y<<","<<P.Z<<")\n";
	return output;
}

ostream& operator<<(ostream& output, Particle P)
{
	output<<"P(q,type,x,y,z) = ("<<P.q<<","<<P.Type<<","<<P.X<<","<<P.Y<<","<<P.Z<<")\n";
	return output;
}
Particle operator+(Particle A, Point3 B)
{
	A.X+=B.X;
	A.Y+=B.Y;
	A.Z+=B.Z;
	return A;
}
Particle operator-(Particle A, Point3 B)
{
	A.X-=B.X;
	A.Y-=B.Y;
	A.Z-=B.Z;
	return A;
}
Particle operator+(Particle A, Particle B)
{
	A.X+=B.X;
	A.Y+=B.Y;
	A.Z+=B.Z;
	return A;
}
Particle operator-(Particle A, Particle B)
{
	A.X-=B.X;
	A.Y-=B.Y;
	A.Z-=B.Z;
	return A;
}

bool operator==(Particle A, Particle B)
{
	if (A.X==B.X&&A.Y==B.Y&&A.Z==B.Z) return true;
	return false;
}

Point3 operator+(Point3 A, Point3 B)
{
	return Point3(A.X+B.X, A.Y+B.Y, A.Z+B.Z);
}
Point3 operator-(Point3 A, Point3 B)
{
	return Point3(A.X-B.X, A.Y-B.Y, A.Z-B.Z);
}
Point3 operator*(FLOAT A, Point3 B)
{
	return Point3(A*B.X, A*B.Y, A*B.Z);
}
Point3 operator/(Point3 B, FLOAT A)
{
	return Point3(B.X/A, B.Y/A, B.Z/A);
}

