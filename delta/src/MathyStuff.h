/*
 * Random.h
 *
 *  Created on: Nov 13, 2009
 *      Author: wes
 */

#ifndef RANDOM_H_
#define RANDOM_H_

#include <cstring>
//#include "mpi.h"
#include <time.h>
#include <math.h>
#include <cstdlib>
#include <string.h>
//#include <stdio.h>
#include <iostream>
#include <sstream>
#include <climits>
#include <fstream>
#include <assert.h>

#define FLOAT double

class Point3;

Point3 operator+(Point3 A, Point3 B);
Point3 operator-(Point3 A, Point3 B);
Point3 operator*(FLOAT A, Point3 B);
Point3 operator/(Point3 B, FLOAT A);
Point3 operator*(Point3 A, FLOAT B);
class Point3
{
	public:
		//friend Point3 operator* (FLOAT A, Point3 B);
		FLOAT X;
		FLOAT Y;
		FLOAT Z;

		friend Point3 operator-(Point3 A, Point3 B);
		friend Point3 operator+(Point3 A, Point3 B);

		Point3()
		{
			this->X=0.0;
			this->Y=0.0;
			this->Z=0.0;
		}

		Point3(FLOAT X, FLOAT Y, FLOAT Z)
		{
			this->X=X;
			this->Y=Y;
			this->Z=Z;
		}
};

class Point2
{
	public:
		FLOAT X;
		FLOAT Y;
		Point2()
		{
			this->X=0;
			this->Y=0;
		}
		Point2(FLOAT X, FLOAT Y)
		{
			this->X=X;
			this->Y=Y;
		}
};

void InitializeRand ();
unsigned int Rand ();
Point3 Normalize (Point3 A);
FLOAT Magnitude2 (FLOAT X, FLOAT Y);
FLOAT Magnitude3 (FLOAT X, FLOAT Y, FLOAT Z);
FLOAT Magnitude2 (Point2 A);
FLOAT Magnitude3 (Point3 A);
FLOAT Dot2 (Point2 A, Point2 B);
FLOAT Dot3 (Point3 A, Point3 B);
Point3 Cross3 (Point3 A, Point3 B);
FLOAT Distance2 (FLOAT X1, FLOAT X2,  FLOAT Y1, FLOAT Y2);
FLOAT Distance3 (FLOAT X1, FLOAT X2,FLOAT Y1, FLOAT Y2, FLOAT Z1, FLOAT Z2);
FLOAT Distance3(Point3 P1, Point3 P2);
FLOAT MagnitudeSquared2 (FLOAT X, FLOAT Y);
FLOAT MagnitudeSquared3 (FLOAT X, FLOAT Y, FLOAT Z);
FLOAT MagnitudeSquared2 (Point2 A);
FLOAT MagnutudeSquared3 (Point3 A);
FLOAT DistanceSquared2 (FLOAT X1, FLOAT X2,  FLOAT Y1, FLOAT Y2);
FLOAT DistanceSquared3 (FLOAT X1, FLOAT X2,FLOAT Y1, FLOAT Y2, FLOAT Z1, FLOAT Z2);
FLOAT GetRadius (int r);
FLOAT GetRadiusinnm (int r);
FLOAT Vfrac (int NumHydronium, int r);
int GetIndex (FLOAT R);
int GetIndexinnm (FLOAT R);
int NumberOfIndices ();

Point3 YfromZ(Point3 Z);
Point3 XfromYZ (Point3 Y, Point3 Z);


double StdDev(double SumX, double SumX2, double N);

template <class T>
class BigArray
{
	public:
	int Length;
	int MaxLength;
	int GrowthFactor;
	T* F;
	BigArray(int StartingSize, int GrowthFactor)
	{
		F=NULL;
		this->F = (T*) malloc ((StartingSize)*sizeof(T));
		this->Length = 0;
		this->MaxLength = StartingSize;
		this->GrowthFactor=GrowthFactor;
		assert(F!=NULL);
	}
	void Add(T F)
	{
		this->F[Length] = F;
		//cout<<F<<"\n";
		//scout<<this->F[Length]<<"\n";
		this->Length++;
		if (this->Length == this->MaxLength)
		{
			T* Temp =NULL;
			Temp = (T*)malloc(Length*GrowthFactor*sizeof(T));
			assert (Temp!=NULL);
			memcpy(Temp,  (this->F), (sizeof(T)*this->Length));
			free(this->F);
			this->F = Temp;
			MaxLength*=GrowthFactor;
		}
	}
	void BubbleSort(bool LittleEndian)
	{

		bool Sorted = false;
		while(Sorted == false)
		{
			Sorted = true;
			for (int i =0; i<Length-1; i++)
			{
				if((this->F[i]>this->F[i+1]&&LittleEndian)||(this->F[i]<this->F[i+1]&&!LittleEndian))
				{
					Switch(i, i+1);
					Sorted = false;
				}
			}
		}
	}
	void Switch(int i, int j)
	{
		FLOAT temp = this->F[i];
		this->F[i] = this->F[j];
		this->F[j] = temp;
	}
	~BigArray()
	{
	free(F);
	}
};

#endif /* RANDOM_H_ */
