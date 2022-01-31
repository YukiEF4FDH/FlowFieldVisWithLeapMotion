/// ----------------------------------------------------------------
/// project: DTI Fiber Explorer
/// file: FieldLineProcessor.h
/// source file of the calculation MDS on GPU
/// ----------------------------------------------------------------

// =================================================================
//
// Copyright (c) 2008-2009, all rights reserved by
// - State Key Lab. of CAD&CG, Zhejiang University
//
// More information:
// http://www.cad.zju.edu.cn/home/chenwei/interface/
//
// Contact:
// Wei Chen:    chenwei@cad.zju.edu.cn or shearwarp@gmail.com
// Ziang Ding:  dingziang@cad.zju.edu.cn or dingziang@gmail.com
// Song Zhang:  szhang@cse.msstate.edu
//
// Algorithm Design: Dr. Wei Chen;
// System Development: Zi'ang Ding;
// DTI Datasets: Dr. Song Zhang;
// DTI Feedback: Dr.Song Zhang;
// DTI Evaluation: Anna M. Brandt, Stephen Correia, John Allen Crow
// Project Support: Prof.Qunsheng Peng.
//
// Acknowledgement:
// This work is partially supported by 973 program of China (2009CB320800),
// NSF of China (No.60873123), the Research Initiation Program at 
// Mississippi State University. 
//
//
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; version 2 of the License
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
// 
// =================================================================

// system includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "time.h"
#include "math.h"

// cuda includes
#include "cublas.h"
#include "cuda_runtime_api.h"

// defines
#ifndef DATETYPE
#define DATATYPE float
#endif

// functions
int Index2D(int x, int y, int width)
{
	return y * width + x;
}

double ComputeDeviation(DATATYPE *proximity, DATATYPE *distance, int n)
{
	int x, y;
	double retVal = 0;
	double diff;
	int index;
	for (y = 0; y < n; ++y)
	{
		for (x = y + 1; x < n; ++x)
		{
			index = Index2D(y, x, n);
			diff = proximity[index] - distance[index];
			retVal += diff * diff;
		}
	}

	return retVal;
}

void ComputeDistance(DATATYPE *distance, DATATYPE *configuration, int n, int lowDim)
{
	int x,y,dim;
	int index;
	int t1, t2;
	int p = 0;
	double diff, sum;

	for (y = 0; y < n; ++y)
	{
		distance[Index2D(y, y, n)] = 0;
		for (x = y + 1; x < n; ++x)
		{
			index = Index2D(y, x, n);
			sum = 0;
			t1 = y;
			t2 = x;
			for (dim = 0; dim < lowDim; ++dim, t1 += n, t2 += n)
			{
				diff = configuration[t1] - configuration[t2];
				sum += diff * diff;
			}
			distance[index] = (float)sqrt(sum);
			distance[Index2D(x, y, n)] = distance[index];
		}
	}
}

void UpdateMatrixB(DATATYPE *h_B, DATATYPE *h_proximity, DATATYPE *h_distance, int n)
{
	int x,y,t;
	int index;
	DATATYPE sum;


	/* since matrix h_B is symmetric, we first compute upper triangular part */
	for(y = 0; y < n; ++y)
	{
		for (x = y + 1; x < n; ++x)
		{
			index = Index2D(y, x, n);
			if (h_distance[index] == 0)
			{
				h_B[index] = 0;
			}
			else
			{
				h_B[index] = -h_proximity[index] / h_distance[index];
			}
		}
	}

	/* assign value to lower triangular part of matrix h_B */
	for (y = 0; y < n; ++y)
	{
		for (x = 0; x < y; ++x)
		{
			h_B[Index2D(y, x, n)] = h_B[Index2D(x, y, n)];
		}
	}

	/* At last we compute the diagonal part of matrix */
	for (t = 0; t < n; ++t)
	{
		sum = 0;
		for (x = 0; x < n; ++x)
		{
			if (x != t)
			{
				sum -= h_B[Index2D(t, x, n)];
			}
		}
		h_B[Index2D(t, t, n)] = sum;
	}
}


void OutputConfiguration(DATATYPE* results, DATATYPE* h_X, int n, int lowDim, double epsilon, double deltaDeviation, double deviation)
{
	int x,y;
	int index = 0;
	
	for (y = 0; y < n; ++y)
	{
		for (x = 0; x < lowDim; ++x, index++)
		{
			results[index] = h_X[Index2D(y, x, n)];

		}
	}
}

/*
	return value: iteration times
*/

int SMACOF(DATATYPE *result, DATATYPE *configuration, DATATYPE* proximity, int n, int lowDim, double* GPUtimeCost, 
		   double* CPUtimeCost, int maxIteration, DATATYPE epsilon)
{
	/* CUBLAS uses column-major matrix */
	cublasStatus status;

	/* column-major matrices compatible with CUBLAS */
	DATATYPE *h_X = NULL, *h_distance = NULL, *h_B = NULL;

	/* since proximity matrix is symmetric, its row-major and column-major representations are the same */
	DATATYPE *h_proximity = proximity;

	/* column-major matrix for CUBLAS */
	DATATYPE *d_B = NULL, *d_Z = NULL, *d_X = NULL;

	/* column-major matrix */
	DATATYPE *d_distance = NULL;

	double deviationPrev = 0, deviationNow = 0, deltaDeviation = 0;
	int n2 = n * n;
	int iterationCounter = 0;
	int x, y;
	long long startTime, finishTime;
	//char resultFileName[200];

	(*GPUtimeCost) = 0;
	(*CPUtimeCost) = 0;

	status = cublasInit();
	if (status != CUBLAS_STATUS_SUCCESS)
	{
		fprintf(stderr, "CUBLAS initialization error!\n");
		exit(1);
	}

	h_X = (DATATYPE*)malloc(n * lowDim * sizeof(DATATYPE));
	if (h_X == NULL)
	{
		fprintf(stderr, "fail to allocate host memory(h_X)\n");
		exit(1);
	}

	h_distance = (DATATYPE*)malloc(n * n * sizeof(DATATYPE));
	if (h_distance == NULL)
	{
		fprintf(stderr, "fail to allocate host memory(h_distance)\n");
		exit(1);
	}

	h_B = (DATATYPE*)malloc(n * n * sizeof(DATATYPE));
	if (h_B == NULL)
	{
		fprintf(stderr, "fail to allocate host memory(h_B)\n");
		exit(1);
	}


	/* copy configuration and proximity data from row-major matrix to column-major matrix used in CUBLAS */
	for (y = 0; y < n; ++y)
	{
		for (x = 0; x < lowDim; ++x)
		{
			h_X[Index2D(y, x, n)] = configuration[Index2D(x, y, lowDim)];
		}
	}

	/* allocate device memory for matrices */
	status = cublasAlloc(n * n, sizeof(DATATYPE), (void**)&d_B);
	if (status != CUBLAS_STATUS_SUCCESS)
	{
		//fprintf(stderr, "fail to allocate device memory(d_B)\n");
		exit(1);
	}

	status = cublasAlloc(lowDim * n, sizeof(DATATYPE), (void**)&d_Z);
	if (status != CUBLAS_STATUS_SUCCESS)
	{
		//fprintf(stderr, "fail to allocate device memory(d_Z)\n");
		exit(1);
	}

	status = cublasAlloc(lowDim * n, sizeof(DATATYPE), (void**)&d_X);
	if (status != CUBLAS_STATUS_SUCCESS)
	{
		//fprintf(stderr, "fail to allocate device memory(d_X)\n");
		exit(1);
	}

	status = cublasSetVector(lowDim * n, sizeof(DATATYPE), h_X, 1, d_Z, 1);
	if (status != CUBLAS_STATUS_SUCCESS) 
	{
		//fprintf (stderr, "!!!! device access error (write matrix Z)\n");
		exit(1);
	}

	startTime = clock();
	ComputeDistance(h_distance, h_X, n, lowDim);
	deviationPrev = deviationNow = ComputeDeviation(h_proximity, h_distance, n);
	finishTime = clock();
	(*CPUtimeCost) += (finishTime - startTime) / CLOCKS_PER_SEC;

	while (iterationCounter == 0 || (iterationCounter < maxIteration && deltaDeviation > epsilon))
	{
		iterationCounter++;
		deviationPrev = deviationNow;
		startTime = clock();
		UpdateMatrixB(h_B, h_proximity, h_distance, n);
		finishTime = clock();
		(*CPUtimeCost) += (finishTime - startTime) / CLOCKS_PER_SEC;

		startTime = clock();
		status = cublasSetVector(n2, sizeof(DATATYPE), h_B, 1, d_B, 1);
		if (status != CUBLAS_STATUS_SUCCESS) 
		{
			//fprintf (stderr, "!!!! device access error (write matrix B)\n");
			exit(1);
		}

		status = cublasSetVector(lowDim * n, sizeof(DATATYPE), h_X, 1, d_Z, 1);
		if (status != CUBLAS_STATUS_SUCCESS) 
		{
			//fprintf (stderr, "!!!! device access error (write matrix Z)\n");
			exit(1);
		}

		if (sizeof(DATATYPE) == 4)	// single precision
		{
			cublasSgemm('n', 'n', n, lowDim, n, 1.0f / n, d_B, n, d_Z, n, 0, d_X, n);
		}
		else						// double precision
		{			
			cublasDgemm('n', 'n', n, lowDim, n, 1.0f / n, d_B, n, d_Z, n, 0, d_X, n);
		}


		status = cublasGetVector(lowDim * n, sizeof(DATATYPE), d_X, 1, h_X, 1);
		if (status != CUBLAS_STATUS_SUCCESS) 
		{
			//fprintf (stderr, "!!!! device access error (read matrix X)\n");
			exit(1);
		}

		finishTime = clock();
		(*GPUtimeCost) += ((finishTime - startTime) / CLOCKS_PER_SEC);

		startTime = clock();
		ComputeDistance(h_distance, h_X, n, lowDim);
		deviationNow = ComputeDeviation(h_proximity, h_distance, n);
		finishTime = clock();
		(*CPUtimeCost) += (finishTime - startTime) / CLOCKS_PER_SEC;
		deltaDeviation = deviationPrev - deviationNow;
//		printf("deviationPrev deviationNow: %f %f \n", deviationPrev, deviationNow);
	}
	

	if (lowDim == 2)
	{
		//sprintf(resultFileName, "%s.2Dresult.txt", datasetName);
	}
	else
	{
		//sprintf(resultFileName, "%s.3Dresult.txt", datasetName);
	}

	OutputConfiguration(result, h_X, n, lowDim, epsilon, (double)deltaDeviation, (double)deviationNow);


	/* copy configuration back */
	for (y = 0; y < n; ++y)
	{
		for (x = 0; x < lowDim; ++x)
		{
			configuration[Index2D(x, y, lowDim)] = h_X[Index2D(y, x, n)];
		}
	}

	/* memory clean up */
	free(h_X);
	free(h_distance);
	free(h_B);

	status = cublasFree(d_B);
	if (status != CUBLAS_STATUS_SUCCESS) 
	{
		fprintf (stderr, "!!!! device memory free error (d_B)\n");
		exit(1);
	}

	status = cublasFree(d_Z);
	if (status != CUBLAS_STATUS_SUCCESS) 
	{
		fprintf (stderr, "!!!! device memory free error (d_Z)\n");
		exit(1);
	}

	status = cublasFree(d_X);
	if (status != CUBLAS_STATUS_SUCCESS) 
	{
		fprintf (stderr, "!!!! device memory free error (d_X)\n");
		exit(1);
	}

	/* shut down */
	status = cublasShutdown();
	if (status != CUBLAS_STATUS_SUCCESS) 
	{
		fprintf (stderr, "!!!! cublas shutdown error\n");
		exit(1);
	}

	return iterationCounter;

}