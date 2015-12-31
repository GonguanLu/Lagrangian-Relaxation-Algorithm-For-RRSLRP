
#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <omp.h>
#include <algorithm>

#include "AgentPlus.h"
#include "CSVParser.h"
#ifdef _DEBUG
#define new DEBUG_NEW
#endif


#define _MAX_NUMBER_OF_CELLS 500
#define _MAX_MOVE_STEPS 4
#define _MAX_TRAINS 10
#define _MAX_TIME_STEPS 400


float g_SegmentGrade[_MAX_NUMBER_OF_CELLS] = { 0 };
float g_SegmentCurve[_MAX_NUMBER_OF_CELLS] = { 0 };
int   g_TrainStationFlag[_MAX_TRAINS][_MAX_NUMBER_OF_CELLS];
int   g_TrainDepartureTimeStart[_MAX_TRAINS] = { 0 };
int   g_TrainDepartureTimeEnd[_MAX_TRAINS] = { 0 };
int   g_TrainOriginStation[_MAX_TRAINS] = { 0 };
int   g_TrainEstiantionStation[_MAX_TRAINS] = { 0 };


int   g_TrainArcFlag[_MAX_TRAINS][_MAX_NUMBER_OF_CELLS][_MAX_MOVE_STEPS][_MAX_MOVE_STEPS][_MAX_MOVE_STEPS]; //k, i,j',u,v
float g_TrainArcCost[_MAX_NUMBER_OF_CELLS][_MAX_MOVE_STEPS][_MAX_MOVE_STEPS][_MAX_MOVE_STEPS]; // i,j',u,v

float g_TrainArcLRPrice[_MAX_NUMBER_OF_CELLS][_MAX_MOVE_STEPS][_MAX_MOVE_STEPS][_MAX_MOVE_STEPS]; // i,j',u,v


float g_LabelCost[_MAX_NUMBER_OF_CELLS][_MAX_TIME_STEPS][_MAX_MOVE_STEPS];
int   g_PredecessorNode[_MAX_NUMBER_OF_CELLS][_MAX_TIME_STEPS][_MAX_MOVE_STEPS];
int   g_PredecessorSpeed[_MAX_NUMBER_OF_CELLS][_MAX_TIME_STEPS][_MAX_MOVE_STEPS];


void g_GenerateGAMSInputData()
{
	//read grade data

	CCSVParser parser;
	if (parser.OpenCSVFile("grade.csv", true))
	{
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			int start_node_index = 0;
			int end_node_index = 0;
			parser.GetValueByFieldName("start_node_index", start_node_index);
			parser.GetValueByFieldName("end_node_index", end_node_index);

			float grade = 0;
			parser.GetValueByFieldName("grade", grade, false);

			for (int i = min(_MAX_NUMBER_OF_CELLS - 1, start_node_index); i < min(_MAX_NUMBER_OF_CELLS - 1, end_node_index); i++)
			{
				g_SegmentGrade[i] = grade;
			}
		}


	}

	parser.CloseCSVFile();


	if (parser.OpenCSVFile("station.csv", true))
	{
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			int node_index = 0;
			parser.GetValueByFieldName("node_index", node_index);

			int train_index = 0;
			parser.GetValueByFieldName("train_index", train_index);

			g_TrainStationFlag[train_index][node_index] = 1;


		}


	}

	parser.CloseCSVFile();
	//read curve data


	if (parser.OpenCSVFile("curve.csv", true))
	{
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			int start_node_index = 0;
			int end_node_index = 0;
			parser.GetValueByFieldName("start_node_index", start_node_index);
			parser.GetValueByFieldName("end_node_index", end_node_index);

			float curve = 0;
			parser.GetValueByFieldName("curve", curve);

			for (int i = min(_MAX_NUMBER_OF_CELLS - 1, start_node_index); i < min(_MAX_NUMBER_OF_CELLS - 1, end_node_index); i++)
			{
				g_SegmentCurve[i] = curve;
			}
		}


	}

	parser.CloseCSVFile();

	FILE * st = NULL;

	st = fopen("GAMS_output.txt", "w");

	if (st != NULL)
	{

		for (int i = 0; i < _MAX_NUMBER_OF_CELLS; i++)
		{

			for (int j_p = 1; j_p < _MAX_MOVE_STEPS; j_p++)
			{

				for (int u = 0; u < _MAX_MOVE_STEPS; u++)
				for (int v = 0; v < _MAX_MOVE_STEPS; v++)
				{
					g_TrainArcFlag[0][i][j_p][u][v] = 0;  // init
					g_TrainArcCost[i][j_p][u][v] = 0;


				}

				for (int u = 0; u < _MAX_MOVE_STEPS; u++)
				{
					int v = j_p * 2 - u;
					if (v >= 0 && v < _MAX_MOVE_STEPS && (i + j_p < _MAX_NUMBER_OF_CELLS))
					{  // v feasible
						g_TrainArcFlag[0][i][j_p][u][v] = 1;
						g_TrainArcCost[i][j_p][u][v] = max(0, (v - u) * (u + v) / 2.0);

					}
				}

			}


		}

		// output speed_arc
		fprintf(st, "parameter speed_arcs(i,j,u,v) /\n");

		for (int i = 0; i < _MAX_NUMBER_OF_CELLS; i++)
		{
			for (int j_p = 0; j_p < _MAX_MOVE_STEPS; j_p++)
			{

				for (int u = 0; u < _MAX_MOVE_STEPS; u++)
				for (int v = 0; v < _MAX_MOVE_STEPS; v++)
				{
					if (g_TrainArcFlag[0][i][j_p][u][v] == 1)
						fprintf(st, "%d . %d . %d . %d  1\n", i + 1, i + 1 + j_p, u, v);

				}


			}


		}

		fprintf(st, "/;\n");

		// output arc cost
		fprintf(st, "parameter cost(i,j,u,v) / \n");

		for (int i = 0; i < _MAX_NUMBER_OF_CELLS; i++)
		{
			for (int j_p = 0; j_p < _MAX_MOVE_STEPS; j_p++)
			{

				for (int u = 0; u < _MAX_MOVE_STEPS; u++)
				for (int v = 0; v < _MAX_MOVE_STEPS; v++)
				{
					if (g_TrainArcFlag[0][i][j_p][u][v] == 1)  // only for feasible arcs 
						fprintf(st, "%d . %d . %d . %d %4.2f\n", i + 1, i + 1 + j_p, u, v, g_TrainArcCost[i][j_p][u][v]);

				}
			}

		}
		fprintf(st, "/;\n");


		fclose(st);
	}

}
