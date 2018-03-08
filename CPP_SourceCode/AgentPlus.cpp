// AgentPlus.cpp : Defines the entry point for the console application.
//
#include "time.h" 
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

// The one and only application object

CWinApp theApp;
using namespace std;

void g_ProgramStop()
{
	cout << "Exit program " << endl;
	exit(0);
};

// step 1: read network_trip data
//
#define _MAX_NUMBER_OF_NODES 1000
#define _MAX_NUMBER_OF_LINKS 4000
#define _MAX_NUMBER_OF_TIME_INTERVALS 100
#define _MAX_LABEL_COST 9999

#define _MAX_NUMBER_OF_VEHICLES 10
#define _MAX_NUMBER_OF_PASSENGERS 10


#define _MAX_NUMBER_OF_OUTBOUND_NODES 10

int g_outbound_node_size[_MAX_NUMBER_OF_NODES];
int g_outbound_node_id[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];
int g_outbound_link_no[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];

int g_inbound_node_size[_MAX_NUMBER_OF_NODES];
int g_inbound_node_id[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];
int g_inbound_link_no[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];

int g_link_free_flow_travel_time[_MAX_NUMBER_OF_LINKS];
float g_link_link_length[_MAX_NUMBER_OF_LINKS];
int g_link_number_of_lanes[_MAX_NUMBER_OF_LINKS];
int g_link_mode_code[_MAX_NUMBER_OF_LINKS];
float g_link_capacity_per_time_interval[_MAX_NUMBER_OF_LINKS];
float g_link_speed[_MAX_NUMBER_OF_LINKS];

float g_link_resource_price[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];
float g_link_resource_passenger_usage[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];
float g_link_resource_vehicle_usage[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];
float g_link_resource_capacity[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];


int g_vehicle_origin_node[_MAX_NUMBER_OF_VEHICLES];
int g_vehicle_departure_time_beginning[_MAX_NUMBER_OF_VEHICLES];
int g_vehicle_departure_time_ending[_MAX_NUMBER_OF_VEHICLES];
int g_vehicle_destination_node[_MAX_NUMBER_OF_VEHICLES];
int g_vehicle_arrival_time_beginning[_MAX_NUMBER_OF_VEHICLES];
int g_vehicle_arrival_time_ending[_MAX_NUMBER_OF_VEHICLES];

int g_passenger_origin_node[_MAX_NUMBER_OF_VEHICLES];
int g_passenger_departure_time_beginning[_MAX_NUMBER_OF_VEHICLES];
int g_passenger_departure_time_ending[_MAX_NUMBER_OF_VEHICLES];
int g_passenger_destination_node[_MAX_NUMBER_OF_VEHICLES];
int g_passenger_arrival_time_beginning[_MAX_NUMBER_OF_VEHICLES];
int g_passenger_arrival_time_ending[_MAX_NUMBER_OF_VEHICLES];


int g_vehicle_path_node_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_vehicle_path_time_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_vehicle_path_number_of_nodes[_MAX_NUMBER_OF_VEHICLES] = { 0 };

int g_vehicle_capacity_resource_price[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_vehicle_usage_by_passenger[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_vehicle_capacity[_MAX_NUMBER_OF_VEHICLES] = { 1 };

int g_passenger_assigned_vehicle_id[_MAX_NUMBER_OF_PASSENGERS] = { 0 };
int g_passenger_path_node_sequence[_MAX_NUMBER_OF_PASSENGERS][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_passenger_path_time_sequence[_MAX_NUMBER_OF_PASSENGERS][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_passenger_path_number_of_nodes[_MAX_NUMBER_OF_VEHICLES] = { 0 };


int g_number_of_links = 0;
int g_number_of_nodes = 0;
int g_number_of_agent=0;

int g_number_of_vehicles = 1;
int g_number_of_passengers = 1;

int g_number_of_LR_iterations = 150;
int g_minimum_subgradient_step_size = 0.01;





//get the link number of the arc between from node to to node
int g_get_link_no_based_on_from_node_to_node(int from_node, int to_node)
{
	if (from_node >= _MAX_NUMBER_OF_NODES)
		return -1;

	if (from_node == to_node)
		return -1;

	//scan outbound links from a upstream node 
	for (int i = 0; i < g_outbound_node_size[from_node]; i++)
	{
		if (g_outbound_node_id[from_node][i] == to_node)
			return g_outbound_link_no[from_node][i];
	}
		
	return -1; 

}

// shortest path code

int g_ListFront;
int g_ListTail;
int g_SENodeList[_MAX_NUMBER_OF_NODES];


// SEList: Scan List implementation: the reason for not using STL-like template is to avoid overhead associated pointer allocation/deallocation
void SEList_clear()
{
	g_ListFront = -1;
	g_ListTail = -1;
}

void SEList_push_front(int node)//to node
{
	if (g_ListFront == -1)  // start from empty
	{
		g_SENodeList[node] = -1;
		g_ListFront = node;
		g_ListTail = node;
	}
	else
	{
		g_SENodeList[node] = g_ListFront;
		g_ListFront = node;
	}
}
void SEList_push_back(int node)
{
	if (g_ListFront == -1)  // start from empty
	{
		g_ListFront = node;
		g_ListTail = node;
		g_SENodeList[node] = -1;
	}
	else
	{
		g_SENodeList[g_ListTail] = node;
		g_SENodeList[node] = -1;
		g_ListTail = node;
	}
}

bool SEList_empty()
{
	return(g_ListFront == -1);//if listFront is -1, the the SEList is empty
}

int SEList_front()
{
	return g_ListFront;
}

void SEList_pop_front()
{
	int tempFront = g_ListFront;
	g_ListFront = g_SENodeList[g_ListFront];//??
	g_SENodeList[tempFront] = -1;
}

int g_node_status_array[_MAX_NUMBER_OF_NODES];
float g_node_label_cost[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS];
float g_node_predecessor[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS];
float g_time_predecessor[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS];

float g_arc_cost[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };
int g_path_node_sequence[_MAX_NUMBER_OF_TIME_INTERVALS]={-1};
int g_path_time_sequence[_MAX_NUMBER_OF_TIME_INTERVALS]={-1};
int g_path_number_of_nodes;

float g_optimal_time_dependenet_label_correcting(float arc_cost[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS], 
	int origin_node, int departure_time_beginning, int departure_time_ending, int destination_node, int arrival_time_beginning, 
	int arrival_time_ending,	int &path_number_of_nodes, int path_node_sequence[_MAX_NUMBER_OF_TIME_INTERVALS], 
	int path_time_sequence[_MAX_NUMBER_OF_TIME_INTERVALS])
// time-dependent label correcting algorithm with double queue implementation
{
	float total_cost = _MAX_LABEL_COST;
	if (g_outbound_node_size[origin_node] == 0)//if no arc origin from the origin node
	{
		return _MAX_LABEL_COST;
	}

	for (int i = 1; i <g_number_of_nodes + 1; i++) //Initialization for all nodes
	{

		g_node_status_array[i] = 0;  // not scanned

		for (int t = 0; t <_MAX_NUMBER_OF_TIME_INTERVALS; t ++)
		{
			g_node_label_cost[i][t] = _MAX_LABEL_COST;
			g_node_predecessor[i][t] = -1;  // pointer to previous NODE INDEX from the current label at current node and time
			g_time_predecessor[i][t] = -1;  // pointer to previous TIME INDEX from the current label at current node and time
			//node predecessor indicate the previous node of current node and time label
			//time predecessor indicate the previous time of current node and time label
			
		}

	}

	//Initialization for origin node at the preferred departure time, at departure time, cost = 0, otherwise, the delay at origin node

	for (int t = departure_time_beginning; t <= min(departure_time_ending, _MAX_NUMBER_OF_TIME_INTERVALS); t++)
	{
		g_node_label_cost[origin_node][t] = 0;
		//let "all labels which represent the time between departure begin and end" equal to 0
	}

	SEList_clear();
	SEList_push_front(origin_node);


	while (!SEList_empty())//if SEList is not empty, run below
	{
		int from_node = SEList_front();//pop a node FromID for scanning

		SEList_pop_front();  // remove current node FromID from the SE list

		//scan all outbound nodes of the current node
		for (int i = 0; i<g_outbound_node_size[from_node]; i++)  // for each link (i,j) belong A(i)
		{
			int link_no = g_outbound_link_no[from_node][i];
			int to_node = g_outbound_node_id[from_node][i];

			bool  b_node_updated = false;

			// for each time step, starting from the departure time			
			for (int t = 0; t <_MAX_NUMBER_OF_TIME_INTERVALS; t ++)
			{
				if (g_node_label_cost[from_node][t]<_MAX_LABEL_COST - 1)  // for feasible time-space point only
				{
					//calculate the arrival time from from_node to linked link i(come from  "g_outbound_link_no[from_node][i]")
					//if to node arrival time >Max time interval, it doesn't work 
					int new_to_node_arrival_time = min(t + g_link_free_flow_travel_time[link_no], _MAX_NUMBER_OF_TIME_INTERVALS-1);

					//caculate the temp label cost, which is the sum of ST(Space-Time) node cost and arc cost
					float temporary_label_cost = g_node_label_cost[from_node][t] + arc_cost[link_no][t];

					// if find a smaller cost substitute new temp cost for the old one, update the node.
					if (temporary_label_cost < g_node_label_cost[to_node][new_to_node_arrival_time]) // we only compare cost at the downstream node ToID at the new arrival time t
					{

						// update cost label and node/time predecessor
						g_node_label_cost[to_node][new_to_node_arrival_time] = temporary_label_cost;
						g_node_predecessor[to_node][new_to_node_arrival_time] = from_node;  // pointer to previous physical NODE INDEX from the current label at current node and time
						g_time_predecessor[to_node][new_to_node_arrival_time] = t;  // pointer to previous TIME INDEX from the current label at current node and time

						b_node_updated = true;


						}
				}
				//another condition: in the SELite now: there is no need to put this node to the SEList, since it is already there.
			}

			if (b_node_updated == true)
			{
				SEList_push_front(to_node);//let the to_node be the 
			}
			
				//another condition: in the SELite now: there is no need to put this node to the SEList, since it is already there.
		}

		// waiting arc from the from node 

		bool b_node_updated = false;

		for (int t = 0; t <_MAX_NUMBER_OF_TIME_INTERVALS; t++)
		{
			if (g_node_label_cost[from_node][t]<_MAX_LABEL_COST - 1)  // for feasible time-space point only
			{

				int new_to_node_arrival_time = min(t + 1, _MAX_NUMBER_OF_TIME_INTERVALS - 1);

				float temporary_label_cost = g_node_label_cost[from_node][t] + 0;  // no waiting cost

				//if wait a time interval will make the label cost better then wait a time interval
				if (temporary_label_cost < g_node_label_cost[from_node][new_to_node_arrival_time]) // we only compare cost at the downstream node ToID at the new arrival time t
				{

					// update cost label and node/time predecessor
					g_node_label_cost[from_node][new_to_node_arrival_time] = temporary_label_cost;
					g_node_predecessor[from_node][new_to_node_arrival_time] = from_node;  // pointer to previous physical NODE INDEX from the current label at current node and time
					g_time_predecessor[from_node][new_to_node_arrival_time] = t;  // pointer to previous TIME INDEX from the current label at current node and time

					b_node_updated = true;


				}
			}
		}

		if (b_node_updated == true)
		{
			SEList_push_back(from_node);

		}
	}//end of while

	float min_cost = _MAX_LABEL_COST;
	int min_cost_time_index = -1;

	 // scan each time index in the arrival time window    //select the minimum cost in the solutions
	for (int t = arrival_time_beginning; t <arrival_time_ending; t ++)
	{
		if (g_node_label_cost[destination_node][t] < min_cost)
		{
			min_cost = g_node_label_cost[destination_node][t];
			min_cost_time_index = t;
		}
	}

	total_cost = min_cost;

	//local variables
	int reversed_path_node_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
	int reversed_path_time_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];

	// step 2: backtrack to the origin (based on node and time predecessors)
	int	node_size = 0;
	reversed_path_node_sequence[node_size] = destination_node;//record the first node backward, destination node
	reversed_path_time_sequence[node_size] = min_cost_time_index;


	node_size++;

	int pred_node = g_node_predecessor[destination_node][min_cost_time_index];
	int pred_time = g_time_predecessor[destination_node][min_cost_time_index];

	while (pred_node != -1 && node_size < _MAX_NUMBER_OF_TIME_INTERVALS) // scan backward in the predessor array of the shortest path calculation results
	{
		reversed_path_node_sequence[node_size] = pred_node;
		reversed_path_time_sequence[node_size] = pred_time;

		node_size++;

		//record current values of node and time predecessors, and update PredNode and PredTime

		int pred_node_record = pred_node;
		int pred_time_record = pred_time;

		pred_node = g_node_predecessor[pred_node_record][pred_time_record];
		pred_time = g_time_predecessor[pred_node_record][pred_time_record];
	}

	//get the node and time sequence by normal order
	for (int n = 0; n < node_size; n++)
	{
		path_node_sequence[n] = reversed_path_node_sequence[node_size - n - 1];
		path_time_sequence[n] = reversed_path_time_sequence[node_size - n - 1];

	}

	path_number_of_nodes = node_size;
	return total_cost;

}

void g_ReadInputData(std::string FileName)
{
	int node_id_flag[_MAX_NUMBER_OF_NODES] = { 0 };

	// initialization
	for (int i = 0; i < _MAX_NUMBER_OF_NODES; i++)
	{
		g_outbound_node_size[i] = 0;
		g_inbound_node_size[i] = 0;
	}

	CCSVParser parser;
	if (parser.OpenCSVFile(FileName, false))
	{
		g_number_of_nodes = 0;
		g_number_of_links = 0;  // initialize  the counter to 0
		g_number_of_agent=0;
		std::map<int, int> node_id_map;

		parser.m_bDataHubSingleCSVFile = true;

		while (parser.ReadRecord())
		{

			//Read node info
			if (parser.m_DataHubSectionName.find("Node") != string::npos)
			{

				string name;

				int node_type;
				int node_id;
				double X;
				double Y;
				if (parser.GetValueByFieldName("node_id", node_id) == false)
					continue;

				if (node_id <= 0 || node_id >= _MAX_NUMBER_OF_NODES)
				{
					cout << "node_id " << node_id << " is out of range" << endl;
					g_ProgramStop();
				}
				node_id_flag[node_id] = 1;

				parser.GetValueByFieldName("node_type", node_type);
				parser.GetValueByFieldName("x", X);
				parser.GetValueByFieldName("y", Y);

				g_number_of_nodes++;
			}

			


			//Read Link Info
			if (parser.m_DataHubSectionName.find("Link") != string::npos)
			{
				int from_node_id = 0;
				int to_node_id = 0;
				if (parser.GetValueByFieldName("from_node_id", from_node_id) == false)
					continue;
				if (parser.GetValueByFieldName("to_node_id", to_node_id) == false)
					continue;

				if (from_node_id <= 0 || from_node_id >= _MAX_NUMBER_OF_NODES)
				{
					cout << "from_node_id " << from_node_id << " is out of range" << endl;
					g_ProgramStop();
				}

				if (to_node_id <= 0 || to_node_id >= _MAX_NUMBER_OF_NODES)
				{
					cout << "to_node_id " << to_node_id << " is out of range" << endl;
					g_ProgramStop();
				}

				if (node_id_flag[from_node_id] != 1)
				{
					cout << "from_node_id " << from_node_id << " has not been defined in node block" << endl;
					g_ProgramStop();
				}

				if (node_id_flag[to_node_id] != 1)
				{
					cout << "to_node_id " << to_node_id << " has not been defined in node block" << endl;
					g_ProgramStop();
				}

				// add the to node id into the outbound (adjacent) node list

				int direction = 1;
				parser.GetValueByFieldName("direction", direction);

				if (direction <= -2 || direction >= 2)
				{
					cout << "direction " << direction << " is out of range" << endl;
					g_ProgramStop();
				}

				for (int link_direction = -1; link_direction <= 1; link_direction += 2)  // called twice; -1 direction , 1 direction 
				{
					if (direction == -1 && link_direction == 1)
						continue; // skip

					if (direction == 1 && link_direction == -1)
						continue; // skip

					// then if  direction == 0 or 2 then create the corresponding link

					int directional_from_node_id = from_node_id;
					int directional_to_node_id = to_node_id;

					if (link_direction == -1) // reverse direction;
					{
						directional_from_node_id = to_node_id;
						directional_to_node_id = from_node_id;
					}

					g_outbound_node_id[directional_from_node_id][g_outbound_node_size[directional_from_node_id]] = directional_to_node_id;
					g_outbound_link_no[directional_from_node_id][g_outbound_node_size[directional_from_node_id]] = g_number_of_links;
					g_outbound_node_size[directional_from_node_id]++;

					g_inbound_node_id[directional_to_node_id][g_inbound_node_size[directional_to_node_id]] = directional_from_node_id;
					g_inbound_link_no[directional_to_node_id][g_inbound_node_size[directional_to_node_id]] = g_number_of_links;
					g_inbound_node_size[directional_to_node_id]++;

					float link_length = 1;
					int number_of_lanes = 1;
					int mode_code = 0;
					float capacity_per_time_interval = 1;
					int travel_time = 1;
					float speed = 1;

					parser.GetValueByFieldName("length", link_length);
					parser.GetValueByFieldName("number_of_lanes", number_of_lanes);
					parser.GetValueByFieldName("mode_code", mode_code);
					parser.GetValueByFieldName("lane_capacity_in_vhc_per_hour", capacity_per_time_interval);
					parser.GetValueByFieldName("travel_time", travel_time);
					parser.GetValueByFieldName("speed_limit", speed);

					g_link_free_flow_travel_time[g_number_of_links] = travel_time;
					g_link_link_length[g_number_of_links] = link_length;
					g_link_number_of_lanes[g_number_of_links] = number_of_lanes;
					g_link_mode_code[g_number_of_links] = mode_code;
					g_link_capacity_per_time_interval[g_number_of_links] = capacity_per_time_interval;
					g_link_speed[g_number_of_links] = speed;

					// increase the link counter by 1
					g_number_of_links++;

				}
			}

			//Read Agent Info
			if (parser.m_DataHubSectionName.find("Agent") != string::npos)
			{
				int from_node_id = 0;
				int to_node_id = 0;
				if (parser.GetValueByFieldName("from_node_id", from_node_id) == false)
					continue;
				if (parser.GetValueByFieldName("to_node_id", to_node_id) == false)
					continue;

				if (from_node_id <= 0 || from_node_id >= _MAX_NUMBER_OF_NODES)
				{
					cout << "from_node_id " << from_node_id << " is out of range(Agent Generation process)" << endl;
					g_ProgramStop();
				}

				if (to_node_id <= 0 || to_node_id >= _MAX_NUMBER_OF_NODES)
				{
					cout << "to_node_id " << to_node_id << " is out of range (Agent Generation process)" << endl;
					g_ProgramStop();
				}

				if (node_id_flag[from_node_id] != 1)
				{
					cout << "from_node_id " << from_node_id << " has not been defined in node block" << endl;
					g_ProgramStop();
				}

				if (node_id_flag[to_node_id] != 1)
				{
					cout << "to_node_id " << to_node_id << " has not been defined in node block" << endl;
					g_ProgramStop();
				}

				//read the agents

				parser.GetValueByFieldName("from_node_id", from_node_id);
				parser.GetValueByFieldName("to_node_id", to_node_id);
				int departure_time_start=-1;
				int departure_time_end=-1; 
				int	arrival_time_start=-1;
				int arrival_time_end=-1;
				parser.GetValueByFieldName("departure_time_start", departure_time_start);
				parser.GetValueByFieldName("departure_time_end", departure_time_end);
				parser.GetValueByFieldName("arrival_time_start", arrival_time_start);
				parser.GetValueByFieldName("arrival_time_end", arrival_time_end);
				
				g_vehicle_origin_node[g_number_of_agent]=from_node_id;
				g_vehicle_departure_time_beginning[g_number_of_agent]=departure_time_start;
				g_vehicle_departure_time_ending[g_number_of_agent]=departure_time_end;

				g_vehicle_destination_node[g_number_of_agent]=to_node_id;
				g_vehicle_arrival_time_beginning[g_number_of_agent]=arrival_time_start;
				g_vehicle_arrival_time_ending[g_number_of_agent]=arrival_time_end;

				g_number_of_agent++;
				g_number_of_vehicles++;

			}
		}

		cout << "read " << g_number_of_nodes << ", " << g_number_of_links << " links" << endl;
		parser.CloseCSVFile();

		
	}

}

//select Algorithm type, simultaneously or sequentially
//--->this parameter could be included into the input csv file

int g_PathBasedScheduling=0;//0:simultaneous 1:sequential
int g_NumberOfIterationsWithMemory = 5; //iteration memory, not used till Oct.3
int g_OptimizationHorizon=1440;//一天1440分钟
float g_global_lower_bound_record=-9999;
bool g_Optimization_Lagrangian_Method()
{

	cout << "Preparation......" << endl;

	int NumberOfIterationsWithMemory = g_NumberOfIterationsWithMemory;//迭代历史记忆，5代

	int OptimizationHorizon = g_OptimizationHorizon;//Horizon, Time Axis
	//step 1: initialization 
	
	//reset price of links and link usage by vehicle and passengers

	for (int link = 0; link < g_number_of_links; link++)
	{
		for (int t = 0; t < _MAX_NUMBER_OF_TIME_INTERVALS; t++)
		{
			g_link_resource_price[link][t] = 0;//link在每一个时间点的价格		
			g_link_resource_vehicle_usage[link][t] = 0;//每个link时间点，vehile对link的用量
			//g_link_resource_passenger_usage[link][t] = 0;//每个link时间点，passenger对link的用量
			g_link_resource_capacity[link][t]=g_link_capacity_per_time_interval[link];
			g_arc_cost[link][t] = 0;

		}
	}
	clock_t start, finish;
	start = clock(); 
	cout << "Start scheduling agents by Lagrangian Relaxation method" << endl;
	
	//loop for each LR iteration
	float LR_Update_Ratio=9999;
	for (int LR_iteration = 0; LR_iteration < g_number_of_LR_iterations; LR_iteration++)
	{
		
		double global_lower_bound = 0;

		cout << "Lagrangian Iteration " << LR_iteration + 1 << "/" << g_number_of_LR_iterations << endl;

		float TotalTripPrice = 0;
		float TotalTravelTime = 0;
		double TotalWaitingTimeCost = 0;
		
		//step 1: SP for vehicle
		//
		
		for (int v = 0; v < g_number_of_vehicles-1; v++)//note that the scheduling sequence does not matter  here
		{
			// set arc cost for vehicles
			for (int link = 0; link < g_number_of_links; link++)
			{
				for (int t = 0; t < _MAX_NUMBER_OF_TIME_INTERVALS; t++)
				{
					g_arc_cost[link][t] = g_link_resource_price[link][t] ;//* (-1);  // negative resource price -> profit for vehicles 
				}
			}
			
			float inttest=g_arc_cost[1][3];
			//这个地方每个g_arc_cost是当前车v在三/二维数组中v本身所对应的那个价格矩阵,
			//也就是每一个v都是在对于自己的价格矩阵中计算的最短路，资源对于每一个列车的价格都是不同的
			float path_benefit_collected_by_vehicle_v =
			g_optimal_time_dependenet_label_correcting(
				g_arc_cost,
				g_vehicle_origin_node[v],
				g_vehicle_departure_time_beginning[v],
				g_vehicle_departure_time_ending[v],
				g_vehicle_destination_node[v],
				g_vehicle_arrival_time_beginning[v],
				g_vehicle_arrival_time_ending[v],
				g_vehicle_path_number_of_nodes[v],
				g_vehicle_path_node_sequence[v],
				g_vehicle_path_time_sequence[v]);
			
			global_lower_bound += path_benefit_collected_by_vehicle_v;
		}

		LR_Update_Ratio=abs((global_lower_bound-g_global_lower_bound_record)/g_global_lower_bound_record );
		cout << LR_Update_Ratio<<"\n";
		if (LR_Update_Ratio< 0.0001)
		{  
			cout << "optimial solution!!\n";
			break;
		}

		if(global_lower_bound!=0)
			g_global_lower_bound_record=global_lower_bound;

		// step 2: totally link use 
		//reset usage information first
		for (int link = 0; link < g_number_of_links; link++)
		{
			for (int t = 0; t < _MAX_NUMBER_OF_TIME_INTERVALS; t++)
			{
				g_link_resource_vehicle_usage[link][t] = 0;
		//		g_link_resource_passenger_usage[link][t] = 0;
			}
		}

		// scan all vehicles
		for (int v = 0; v < g_number_of_vehicles-1; v++)
		{
			for (int i = 0; i < g_vehicle_path_number_of_nodes[v] - 1; i++)  // for each link, 
			{
				int from_node = g_vehicle_path_node_sequence[v][i];
				int to_node = g_vehicle_path_node_sequence[v][i + 1];

				int time_index = g_vehicle_path_time_sequence[v][i];

				int link_no = g_get_link_no_based_on_from_node_to_node(from_node, to_node);//get the link id

				if (link_no >= 0) // feasible link frrom node i to node j
				{
					g_link_resource_vehicle_usage[link_no][time_index] += 1;

				}

			}
		}

	
		
		// step 4: update total resource price 
		
		//stepsize is the lagrangian multiplier

		float StepSize =  1.0f / (LR_iteration + 1.0f);
		if (StepSize < g_minimum_subgradient_step_size)  //1.3.1 keep the minimum step size
		{
			StepSize = g_minimum_subgradient_step_size;
		}

		for(int link=0;link<g_number_of_links;link++)
		{
			for(int t=0;t<_MAX_NUMBER_OF_TIME_INTERVALS;t++)
			{
				g_link_resource_price[link][t]+=StepSize*max(0,(g_link_resource_vehicle_usage[link][t]-g_link_resource_capacity[link][t]));
				//g_link_resource_price[link][t]+=StepSize*max(0,(g_link_resource_vehicle_usage[link][t]-g_link_capacity_per_time_interval[link]));
			}
		
		}
		float kkk=0;

	}//for each lagrangian relaxation iteration


	finish =clock();
	cout << "End of Lagrangian Iteration Process " << endl;
	cout << "Running Time:" << (double)(finish - start) / CLOCKS_PER_SEC << endl;

	return true;
}


void g_Generate_Initial_Arc_cost_matrix()
{
	for(int i=0;i<g_number_of_links;i++)
	{
		for(int j=0;j<_MAX_NUMBER_OF_TIME_INTERVALS;j++)
		{
			g_arc_cost[i][j] =g_link_link_length[i];
		}
	}
}

int _tmain(int argc, TCHAR* argv[], TCHAR* envp[])
{
	int nRetCode = 0;

	HMODULE hModule = ::GetModuleHandle(NULL);

	if (hModule != NULL)
	{
		// initialize MFC and print and error on failure
		if (!AfxWinInit(hModule, NULL, ::GetCommandLine(), 0))
		{
			// TODO: change error code to suit your needs
			_tprintf(_T("Fatal Error: MFC initialization failed\n"));
			nRetCode = 1;
		}
		else
		{
			// TODO: code your application's behavior here.
		}
	}
	else
	{
		// TODO: change error code to suit your needs
		_tprintf(_T("Fatal Error: GetModuleHandle failed\n"));
		nRetCode = 1;
	}


	g_ReadInputData("C:\\Code\\ProgramStart\\AgentPlus\\input_data_hub_6_nodes.csv");

	g_Generate_Initial_Arc_cost_matrix();

	float ShortestPathLength=g_optimal_time_dependenet_label_correcting(g_arc_cost, 
	g_vehicle_origin_node[0], g_vehicle_departure_time_beginning[0],  g_vehicle_departure_time_ending[0], g_vehicle_destination_node[0], g_vehicle_arrival_time_beginning[0], 
	g_vehicle_arrival_time_ending[0],	g_path_number_of_nodes, g_path_node_sequence, 
	g_path_time_sequence);

	g_Optimization_Lagrangian_Method();

	return nRetCode;
}

