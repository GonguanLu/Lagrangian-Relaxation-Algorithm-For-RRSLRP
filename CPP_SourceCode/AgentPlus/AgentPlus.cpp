// AgentPlus.cpp : Defines the entry point for the console application.
//
//
//#include "stdafx.h"
//#include <iostream>
//#include <fstream>
//#include <omp.h>
//#include <algorithm>
//
//#include "AgentPlus.h"
//#include "CSVParser.h"
//


#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <omp.h>
#include <algorithm>
#include <time.h>
#include "AgentPlus.h"
#include "CSVParser.h"
#include <windows.h> 



#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// The one and only application object

CWinApp theApp;
using namespace std;

FILE* g_pFileDebugLog = NULL;


void g_ProgramStop()
{
	cout << "Exit program " << endl;
	exit(0);
};

// step 1: read network_trip data
//
#define _MAX_NUMBER_OF_NODES 1000
#define _MAX_NUMBER_OF_LINKS 4000
#define _MAX_NUMBER_OF_TIME_INTERVALS 125
#define _MAX_NUMBER_OF_RESOURCES 100
#define _MAX_NUMBER_OF_BUDGET 600

#define _MAX_NUMBER_OF_VEHICLES 30
#define _MAX_NUMBER_OF_AGENTS 300
#define _MAX_NUMBER_OF_SCENARIOS 10
#define _MAX_NUMBER_OF_ITERATIONS 100
#define _MAX_NUMBER_OF_PASSENGERS 60
#define _MAX_NUMBER_OF_FUEL 100

#define _MAX_NUMBER_OF_OUTBOUND_NODES 60

/*
#define _MAX_NUMBER_OF_NODES 1000
#define _MAX_NUMBER_OF_LINKS 4000
#define _MAX_NUMBER_OF_TIME_INTERVALS 200
#define _MAX_NUMBER_OF_RESOURCES 600
#define _MAX_NUMBER_OF_BUDGET 900

#define _MAX_NUMBER_OF_VEHICLES 50
#define _MAX_NUMBER_OF_AGENTS 340
#define _MAX_NUMBER_OF_SCENARIOS 10
#define _MAX_NUMBER_OF_ITERATIONS 150
#define _MAX_NUMBER_OF_PASSENGERS 60
#define _MAX_NUMBER_OF_FUEL 10

#define _MAX_NUMBER_OF_OUTBOUND_NODES 60
*/

int g_number_of_LR_iterations = 100;

int g_number_of_links = 0;
int g_number_of_nodes = 0;
int g_maximum_node_number = 0;

int ***l_state_node_predecessor=NULL;

int ***l_state_time_predecessor=NULL;

int ***l_state_resource_predecessor=NULL;

float ***l_state_node_label_cost = NULL;


bool g_knap_sack_debugging_flag = true;

bool g_location_routing_debugging_flag = true ;


//depot node
int g_depot_node[_MAX_NUMBER_OF_NODES];
int g_number_of_depot_node;
int superOriginNode, superDestinationNode;
int g_node_type[_MAX_NUMBER_OF_NODES];//=1 normal node, =0 depot node, =2 super origin, =3 super destination 

//depot capacity usage
int g_depot_capacity_usage[_MAX_NUMBER_OF_NODES]={0};
int g_depot_capacity_usage_UB[_MAX_NUMBER_OF_NODES]={0};
int g_depot_capacity[_MAX_NUMBER_OF_NODES];

//depot capacity multiplier
int g_number_of_depot=0;
float g_depot_capacity_price[_MAX_NUMBER_OF_NODES];
float g_depot_capacity_profit_for_Knapsack[_MAX_NUMBER_OF_NODES];
float g_depot_capacity_price_UB[_MAX_NUMBER_OF_NODES];

//float l_state_node_label_cost[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS][_MAX_NUMBER_OF_RESOURCES] = {0};

float g_link_travel_cost[_MAX_NUMBER_OF_LINKS]={0};

int l_link_resource_cost[_MAX_NUMBER_OF_LINKS]={0};
int g_outbound_node_size[_MAX_NUMBER_OF_NODES];
int g_outbound_node_id[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];
int g_outbound_link_no[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];

int g_inbound_node_size[_MAX_NUMBER_OF_NODES];
int g_inbound_node_id[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];
int g_inbound_link_no[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];

int g_link_free_flow_travel_time[_MAX_NUMBER_OF_LINKS];
int g_link_min_travel_time[_MAX_NUMBER_OF_LINKS];
float g_external_link_time_dependent_toll[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };
float g_link_link_length[_MAX_NUMBER_OF_LINKS];
int g_link_number_of_lanes[_MAX_NUMBER_OF_LINKS];
int g_link_mode_code[_MAX_NUMBER_OF_LINKS];
float g_link_capacity_per_time_interval[_MAX_NUMBER_OF_LINKS];
float g_link_speed[_MAX_NUMBER_OF_LINKS];
int g_link_from_node_number[_MAX_NUMBER_OF_LINKS];
int g_link_to_node_number[_MAX_NUMBER_OF_LINKS];
//For Location-Routing
int g_link_type[_MAX_NUMBER_OF_LINKS];
int g_link_departure_time_start[_MAX_NUMBER_OF_LINKS];
int g_link_departure_time_end[_MAX_NUMBER_OF_LINKS];
int g_link_arrival_time_start[_MAX_NUMBER_OF_LINKS];
int g_link_arrival_time_end[_MAX_NUMBER_OF_LINKS];
int g_link_demand[_MAX_NUMBER_OF_LINKS];
int g_link_demand_UB[_MAX_NUMBER_OF_LINKS];

int g_link_resource_cost[_MAX_NUMBER_OF_LINKS];

float g_link_vehicle_use_y_variable[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];
float g_link_agent_use_x_variable[_MAX_NUMBER_OF_AGENTS][_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];

float g_link_resource_capacity[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];

int g_vehicle_origin_node[_MAX_NUMBER_OF_VEHICLES];  // for vehcile routings
int g_vehicle_departure_time_beginning[_MAX_NUMBER_OF_VEHICLES];
int g_vehicle_departure_time_ending[_MAX_NUMBER_OF_VEHICLES];
int g_vehicle_destination_node[_MAX_NUMBER_OF_VEHICLES];
int g_vehicle_arrival_time_beginning[_MAX_NUMBER_OF_VEHICLES];
int g_vehicle_arrival_time_ending[_MAX_NUMBER_OF_VEHICLES];
int g_vehicle_fuel[_MAX_NUMBER_OF_VEHICLES];

int g_agent_origin_node[_MAX_NUMBER_OF_AGENTS];  // traveling agents
int g_agent_departure_time_beginning[_MAX_NUMBER_OF_AGENTS];
int g_agent_departure_time_ending[_MAX_NUMBER_OF_AGENTS];
int g_agent_destination_node[_MAX_NUMBER_OF_AGENTS];
int g_agent_arrival_time_beginning[_MAX_NUMBER_OF_AGENTS];
int g_agent_arrival_time_ending[_MAX_NUMBER_OF_AGENTS];
//for Location-Routing problem
int g_number_of_scenarios = 1;
int g_AgentDemandLink[_MAX_NUMBER_OF_AGENTS];//supply for demand;
int g_agent_demand_from_node[_MAX_NUMBER_OF_AGENTS];
int g_agent_demand_to_node[_MAX_NUMBER_OF_AGENTS];
int g_agent_demand_amount[_MAX_NUMBER_OF_AGENTS][_MAX_NUMBER_OF_SCENARIOS];

int g_agent_demand_assigned_vehicle_id[_MAX_NUMBER_OF_AGENTS]={-1};
int g_link_resource_agent_demand[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];

int g_vehicle_path_node_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_vehicle_path_time_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_vehicle_path_resource_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS];

int g_vehicle_path_number_of_nodes[_MAX_NUMBER_OF_VEHICLES] = { 0 };
int g_vehicle_path_link_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS];

float g_vehicle_path_cost_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS]; 

float g_vehicle_capacity_resource_price[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];

float g_agent_demand_for_vehicle_price[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];

int g_vehicle_time_dependent_capacity[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_number_of_passengers_in_vehicle[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_vehicle_capacity[_MAX_NUMBER_OF_VEHICLES] = { 1 };
int g_vehicle_capacity_UB[_MAX_NUMBER_OF_VEHICLES] = { 1 };

int g_agent_assigned_vehicle_id[_MAX_NUMBER_OF_AGENTS] = { 0 };
int g_agent_path_node_sequence[_MAX_NUMBER_OF_AGENTS][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_agent_path_time_sequence[_MAX_NUMBER_OF_AGENTS][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_agent_path_number_of_nodes[_MAX_NUMBER_OF_VEHICLES] = { 0 };
float l_arc_cost[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];

int g_path_node_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
int g_path_time_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
int g_path_number_of_nodes;
int g_path_travel_time = 0;
float g_path_travel_time_f=0;

int g_total_budget;
int g_depot_cost[_MAX_NUMBER_OF_NODES]={0};



int g_number_of_vehicles = 0;
int g_number_of_agents = 0;
int g_number_of_toll_records = 0;


int g_minimum_subgradient_step_size = 0.01;

float g_upper_bound[_MAX_NUMBER_OF_ITERATIONS]={99999};
float g_lower_bound[_MAX_NUMBER_OF_ITERATIONS]={-99999};
float g_objective_function_value[_MAX_NUMBER_OF_ITERATIONS]={99999};
bool g_depot_solution[_MAX_NUMBER_OF_ITERATIONS][_MAX_NUMBER_OF_NODES]={-1};
float g_gap[_MAX_NUMBER_OF_ITERATIONS]={1.0f};
int g_shortest_path_debugging_flag = 0;

bool g_no_changed_route_for_agents_flag = false;
bool g_no_capacity_multiplier_flag = false;

int bound_adjust= 4111;
float punishing_multiplier=1.5;

int g_vehicle_price= 500 ;
float g_travel_price_per_length = 0.1f;

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
//dynamic programming routing parameters

class CVRState  //class for vehicle scheduling states
{
public:
	int passenger_carrying_state[_MAX_NUMBER_OF_PASSENGERS];//all passengers are listed in the array to mark the status of vehicle

	CVRState()
	{
		m_vehicle_capacity = 1;
		for (int p = 0; p < _MAX_NUMBER_OF_PASSENGERS; p++)
			passenger_carrying_state[p] = 0;// let all passengers are not in the vehicle
	
	}

	std::vector<int> m_outgoing_state_index_vector;//to record the state changing process, index list
	std::vector<int> m_outgoing_state_change_service_code_vector;//to record the state changing process, state list corresponding to the index list

	int m_vehicle_capacity;

	std::string generate_string_key()
	{
		int g_number_of_passengers = _MAX_NUMBER_OF_PASSENGERS;
		std::string string_key;
		for (int p = 1; p <= g_number_of_passengers; p++)  // scan all passengers
		{

			stringstream s;

			s << "_";
			if (passenger_carrying_state[p] == 1)
			{
				s << p;
			}
			else
			{
				s << " ";
			}
		
			string converted(s.str());

			string_key += converted;


		}
	
		return string_key;  //e.g. _ _ _ or _1_2_3
	}
};
std::vector<CVRState> g_VRStateVector;

//time-dependent resource consumption function
int time_dependent_resource_consumption(int link_id, int perspective_travel_time)
{
	float resource_cost=999;
	float max_travel_time= (float)g_link_free_flow_travel_time[link_id];
	float min_travel_time= (float)g_link_min_travel_time[link_id];
	if(g_link_resource_cost[link_id]>=0)
	{
		resource_cost=(1+(max_travel_time - (float)perspective_travel_time)/max_travel_time)*g_link_resource_cost[link_id];
		//suppose vehicle can get faster speed by consuming more resource
	}
	else
	{
		resource_cost=(1-(max_travel_time - perspective_travel_time)/max_travel_time)*g_link_resource_cost[link_id];
		//suppose the recharging process is homogeneous
	}
	
	return (int)resource_cost;
}

//dynamic programming



int g_node_passenger_id[_MAX_NUMBER_OF_NODES] = { -1 };

int g_number_of_time_intervals = _MAX_NUMBER_OF_TIME_INTERVALS;

float g_to_node_cost[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };
float g_to_node_cost_used_for_upper_bound[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };
float g_vertex_waiting_cost[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };
int g_vertex_visit_count[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };


float g_optimal_resource_space_time_dynamic_programming_shortest_path(
	int vehicle_id,
	float arc_cost[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS],
	float to_node_cost[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS],
	float vertex_cost[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS],
	int origin_node, int departure_time_beginning, int departure_time_ending, int destination_node, int arrival_time_beginning, int arrival_time_ending,
	int &path_number_of_nodes,
	int path_node_sequence[_MAX_NUMBER_OF_TIME_INTERVALS], 
	int path_link_sequence[_MAX_NUMBER_OF_TIME_INTERVALS],
	int path_time_sequence[_MAX_NUMBER_OF_TIME_INTERVALS],
	int path_resource_sequence[_MAX_NUMBER_OF_TIME_INTERVALS],
	float path_cost_sequence[_MAX_NUMBER_OF_TIME_INTERVALS],
	int travel_time_calculation_flag,
	int vehicle_capacity,
	float &travel_time_return_value)
	// time-dependent label correcting algorithm with double queue implementation
	{

		//fprintf(g_pFileDebugLog, "----------routing start: vehicle  %d ----------------------------------------\n", vehicle_id);
		bool allowWaiting=true;
		int w=0;
		float total_cost = _MAX_LABEL_COST;
		if (g_outbound_node_size[origin_node] == 0)
		{	
			return _MAX_LABEL_COST;
		}

	// step 1: Initialization for all nodes
	for (int i = 0; i < _MAX_NUMBER_OF_NODES; i++) //Initialization for all nodes
	{
		for (int t = 0; t < _MAX_NUMBER_OF_TIME_INTERVALS; t++)
		{
			for(int r=0;r<_MAX_NUMBER_OF_RESOURCES;r++)
			{
				//for (int w = 0; w < g_VRStateVector.size(); w++)//for all CVRState in this Vector,
				{
					l_state_node_label_cost[i][t][r] = _MAX_LABEL_COST;//i is the node index, t is the time, w is the state
					l_state_node_predecessor[i][t][r] = -1;  // pointer to previous NODE INDEX from the current label at current node and time
					l_state_time_predecessor[i][t][r] = -1;  // pointer to previous TIME INDEX from the current label at current node and time
					l_state_resource_predecessor[i][t][r]=-1;// pointer to previous RESOURCE INDEX from the current label at current node and time
				}
			}
		}
	}

		//step 2: Initialization for origin node at the preferred departure time, at departure time

		int r0 = g_vehicle_fuel[vehicle_id] ;  // start with full fuel tank
		l_state_node_label_cost[origin_node][departure_time_beginning][r0] = 0;

		// step 3: //dynamic programming
		for (int t = departure_time_beginning; t <= arrival_time_ending; t++)  //first loop: time
		{
			if (t % 10 ==0)
			{
				cout << "vehicle " << vehicle_id << " is scanning time " << t << "..." << endl;
			}
			
			for (int link = 0; link < g_number_of_links; link++)  // for each link (i,j)
			{
				for(int r = 0;r < _MAX_NUMBER_OF_RESOURCES; r++)
				{
					int from_node = g_link_from_node_number[link];
					int to_node = g_link_to_node_number[link];
					int upstream_p = g_node_passenger_id[from_node];
					int downsteram_p = g_node_passenger_id[to_node];
					int new_to_node_arrival_time=-1;
					float temporary_label_cost=9999;
					//int travel_time = g_link_free_flow_travel_time[link];

					if (l_state_node_label_cost[from_node][t][r] < _MAX_LABEL_COST - 1)  // for feasible time-space point only
					{
						//scan all possible travel time from min time to max time, using delta_t
						for(int delta_t = g_link_min_travel_time[link];delta_t<=g_link_free_flow_travel_time[link];delta_t++)
						{
							//h(v,delta_t,i,j)
							int resource_consumption = time_dependent_resource_consumption(link, delta_t);

							int r2 = min( _MAX_NUMBER_OF_RESOURCES - 1, r - resource_consumption);//remaining resource after travel on this link
							if(r2 < 0) //if fuel tank become less than empty after travel through this link, this is not a feasible route
							continue;
							// part 1: link based update
							new_to_node_arrival_time = min(t + delta_t, _MAX_NUMBER_OF_TIME_INTERVALS - 1);//use delta_t as travel time
						
							temporary_label_cost = l_state_node_label_cost[from_node][t][r] + arc_cost[link][t] ;//+ to_node_cost[to_node][new_to_node_arrival_time];

							if (temporary_label_cost < l_state_node_label_cost[to_node][new_to_node_arrival_time][r2]) // we only compare cost at the downstream node ToID at the new arrival time t
							{
								/*fprintf(g_pFileDebugLog, "######temporary_label_cost: %d \n"
									,temporary_label_cost);*/
								// update cost label and node/time predecessor
								l_state_node_label_cost[to_node][new_to_node_arrival_time][r2] = temporary_label_cost;
								l_state_node_predecessor[to_node][new_to_node_arrival_time][r2] = from_node;  // pointer to previous NODE INDEX from the current label at current node and time
								l_state_time_predecessor[to_node][new_to_node_arrival_time][r2] = t;  // pointer to previous TIME INDEX from the current label at current node and time	
								l_state_resource_predecessor[to_node][new_to_node_arrival_time][r2] = r;
						/*		fprintf(g_pFileDebugLog, "------from_node: %d, to_node: %d, to node l_state_node_label_cost:%d,  \n"
									,from_node, to_node,l_state_node_label_cost[to_node][new_to_node_arrival_time][r2]);*/
							}
						}
						

							// part 2: same node based update for waiting arcs	
							//if the upstream node is a waiting node
							if(allowWaiting)
							{
								new_to_node_arrival_time = min(t + 1, _MAX_NUMBER_OF_TIME_INTERVALS - 1);
								temporary_label_cost = l_state_node_label_cost[from_node][t][r] + vertex_cost[from_node][t];
								if (temporary_label_cost < l_state_node_label_cost[from_node][new_to_node_arrival_time][r] ) // we only compare cost at the downstream node ToID at the new arrival time t
								{
									// update cost label and node/time predecessor
									l_state_node_label_cost[from_node][new_to_node_arrival_time][r] = temporary_label_cost;
									l_state_node_predecessor[from_node][new_to_node_arrival_time][r] = from_node;  // pointer to previous NODE INDEX from the current label at current node and time
									l_state_time_predecessor[from_node][new_to_node_arrival_time][r] = t;  // pointer to previous TIME INDEX from the current label at current node and time
									l_state_resource_predecessor[from_node][new_to_node_arrival_time][r] = r;
									//l_state_carrying_predecessor[from_node][new_to_node_arrival_time][w1] = w1;
								}	
							}
					}  // feasible vertex label cost
				}// for all resource
			} // for all link
		} // for all time t

		total_cost = _MAX_LABEL_COST;

		int min_cost_time_index = arrival_time_ending;//the latest arrival time is probably not the optimal solution **************
		int reversed_path_node_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
		int reversed_path_time_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
		int reversed_path_resource_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
		float reversed_path_cost_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];

		//find the minimum cost at destination at ending time, different total cost may appear at different resource index, we must find the min total cost at each resource node
		int final_resource_node = -1;
		for(int r=0;r< _MAX_NUMBER_OF_RESOURCES; r++)
		{
			/*fprintf(g_pFileDebugLog, "^^^^^^^^^^^l_state_node_label_cost[%d][%d][%d]: %d \n",
								destination_node,min_cost_time_index,r,l_state_node_label_cost[destination_node][min_cost_time_index][r]);*/
			if(l_state_node_label_cost[destination_node][min_cost_time_index][r]<total_cost)
			{
				total_cost = l_state_node_label_cost[destination_node][min_cost_time_index][r];
				final_resource_node = r;
			}
		}

		// step 2: backtrack to the origin (based on node and time predecessors)
		int	node_size = 0;
		reversed_path_node_sequence[node_size] = destination_node;//record the first node backward, destination node
		reversed_path_time_sequence[node_size] = min_cost_time_index;
		reversed_path_resource_sequence[node_size]=final_resource_node;
		reversed_path_cost_sequence[node_size] = l_state_node_label_cost[destination_node][min_cost_time_index][final_resource_node];

		node_size++;

		int pred_node = l_state_node_predecessor[destination_node][min_cost_time_index][final_resource_node];
		int pred_time = l_state_time_predecessor[destination_node][min_cost_time_index][final_resource_node];
		int pred_resource = l_state_resource_predecessor[destination_node][min_cost_time_index][final_resource_node];
		
		while (pred_node != -1 && node_size < _MAX_NUMBER_OF_TIME_INTERVALS) // scan backward in the predessor array of the shortest path calculation results
		{
			reversed_path_node_sequence[node_size] = pred_node;
			reversed_path_time_sequence[node_size] = pred_time;
			reversed_path_resource_sequence[node_size] = pred_resource;
			reversed_path_cost_sequence[node_size] = l_state_node_label_cost[pred_node][pred_time][pred_resource];

			node_size++;

			//record current values of node and time predecessors, and update PredNode and PredTime

			int pred_node_record = pred_node;
			int pred_time_record = pred_time;
			int pred_resource_record = pred_resource;

			pred_node = l_state_node_predecessor[pred_node_record][pred_time_record][pred_resource_record];
			pred_time = l_state_time_predecessor[pred_node_record][pred_time_record][pred_resource_record];
			pred_resource = l_state_resource_predecessor[pred_node_record][pred_time_record][pred_resource_record];

		}

	//reverse the node sequence 
	for (int n = 0; n < node_size; n++)
	{
		path_node_sequence[n] = reversed_path_node_sequence[node_size - n - 1];
		path_time_sequence[n] = reversed_path_time_sequence[node_size - n - 1];
		path_resource_sequence[n] = reversed_path_resource_sequence[node_size - n - 1];
		path_cost_sequence[n] = reversed_path_cost_sequence[node_size - n - 1];
	}

	for (int i = 0; i < node_size - 1; i++)  // for each link, 
	{
		int link_no = g_get_link_no_based_on_from_node_to_node(path_node_sequence[i], path_node_sequence[i + 1]);
		path_link_sequence[i] = link_no;
	}

	travel_time_return_value = path_time_sequence[node_size - 1] - path_time_sequence[0];
	path_number_of_nodes = node_size;

	if(false)//print Path log
	{
		fprintf(g_pFileDebugLog, "\Vehicle %d path has %d nodes with a transportation cost of %f and travel time of %d: ",
		vehicle_id,
		path_number_of_nodes,
		total_cost,
		travel_time_return_value);

		fprintf(g_pFileDebugLog, "\n  node sequence:  ");
		for (int i = 0; i < path_number_of_nodes; i++)
		{
			if (i != path_number_of_nodes - 1)
				fprintf(g_pFileDebugLog, "%d -> ", path_node_sequence[i]);
			else
				fprintf(g_pFileDebugLog, " %d ", path_node_sequence[i]);
		}

		fprintf(g_pFileDebugLog, "\n  time sequence:  ");
		for (int i = 0; i < path_number_of_nodes; i++)
		{
			if (i != path_number_of_nodes - 1)
				fprintf(g_pFileDebugLog, "%d -> ", path_time_sequence[i]);
			else
				fprintf(g_pFileDebugLog, " %d", path_time_sequence[i]);
		}

		fprintf(g_pFileDebugLog, "\n  resource sequence:  ");
		for (int i = 0; i < path_number_of_nodes; i++)
		{
			if (i != path_number_of_nodes - 1)
				fprintf(g_pFileDebugLog, "%d -> ", path_resource_sequence[i]);
			else
				fprintf(g_pFileDebugLog, " %d", path_resource_sequence[i]);
		}

		fprintf(g_pFileDebugLog, "\n");
	}


	return total_cost;
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

void SEList_push_front(int node)
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
	return(g_ListFront == -1);
}

int SEList_front()
{
	return g_ListFront;
}

void SEList_pop_front()
{
	int tempFront = g_ListFront;
	g_ListFront = g_SENodeList[g_ListFront];
	g_SENodeList[tempFront] = -1;
}

int g_node_status_array[_MAX_NUMBER_OF_NODES];
float g_node_label_cost[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_node_predecessor[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_time_predecessor[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS];

float g_arc_cost[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };



float g_optimal_time_dependenet_label_correcting(float arc_cost[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS],
	int origin_node, int departure_time_beginning, int departure_time_ending, int destination_node, int arrival_time_beginning, int arrival_time_ending,
	int &path_number_of_nodes, 
	int path_node_sequence[_MAX_NUMBER_OF_TIME_INTERVALS], int path_time_sequence[_MAX_NUMBER_OF_TIME_INTERVALS],
	int travel_time_calculation_flag,
	int &travel_time_return_value, bool IfWaiting)
// time-dependent label correcting algorithm with double queue implementation
{
	float total_cost;
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
		g_number_of_depot_node = 0;
		std::map<int, int> node_id_map;

		parser.m_bDataHubSingleCSVFile = true;

		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{

			if (parser.m_DataHubSectionName.find("Node") != string::npos)
			{

				string name;

				int node_type;
				int node_id;
				int node_capacity;
				int node_cost;
				double X;
				double Y;
				if (parser.GetValueByFieldName("node_id", node_id) == false)
					continue;

				if (node_id > g_maximum_node_number)
					g_maximum_node_number = node_id;

				if (node_id <= 0 || g_maximum_node_number >= _MAX_NUMBER_OF_NODES)
				{
					cout << "node_id " << node_id << " is out of range" << endl;
					g_ProgramStop();
				}
				node_id_flag[node_id] = 1;

				parser.GetValueByFieldName("node_type", node_type);
				parser.GetValueByFieldName("capacity", node_capacity);
				parser.GetValueByFieldName("cost", node_cost);
				parser.GetValueByFieldName("x", X);
				parser.GetValueByFieldName("y", Y);

				g_node_type[node_id]=node_type;
				g_depot_capacity[node_id] = node_capacity;
				g_depot_cost[node_id] = node_cost;

				if(node_type==2) superOriginNode=node_id;
				if(node_type==3) superDestinationNode=node_id;

				if(node_type == 0)
				{
					g_depot_node[g_number_of_depot_node] = node_id;
					g_number_of_depot_node ++;
				}
				

				g_number_of_nodes++;
			}

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
					int min_travel_time=1;
					float speed = 1;
					int link_type=-2;
					int link_departure_time_start;
					int link_departure_time_end;
					int link_arrival_time_start;
					int link_arrival_time_end;
					int link_demand;
					int resource_cost;
					int travel_cost;

					parser.GetValueByFieldName("length", link_length);
					parser.GetValueByFieldName("number_of_lanes", number_of_lanes);
					parser.GetValueByFieldName("mode_code", mode_code);
					parser.GetValueByFieldName("lane_capacity_in_vhc_per_hour", capacity_per_time_interval);
					parser.GetValueByFieldName("min_travel_time", min_travel_time);
					parser.GetValueByFieldName("travel_time", travel_time);
					parser.GetValueByFieldName("link_demand", link_demand);
					parser.GetValueByFieldName("travel_cost", travel_cost);
					parser.GetValueByFieldName("speed_limit", speed);
					parser.GetValueByFieldName("link_type", link_type);
					parser.GetValueByFieldName("resource_cost", resource_cost);

					parser.GetValueByFieldName("departure_time_start", link_departure_time_start);
					if(link_departure_time_start<0)
					{
						link_departure_time_start = 0;
						link_departure_time_end = _MAX_NUMBER_OF_TIME_INTERVALS;
						link_arrival_time_start = 0;
						link_arrival_time_end = _MAX_NUMBER_OF_TIME_INTERVALS;
					}
					else
					{
						parser.GetValueByFieldName("departure_time_end", link_departure_time_end);
						parser.GetValueByFieldName("arrival_time_start", link_arrival_time_start);
						parser.GetValueByFieldName("arrival_time_end", link_arrival_time_end);
					}


					g_link_from_node_number[g_number_of_links] = directional_from_node_id;
					g_link_to_node_number[g_number_of_links] = directional_to_node_id;

					g_link_free_flow_travel_time[g_number_of_links] = max(0, travel_time);
					g_link_min_travel_time[g_number_of_links] = max(0, min_travel_time);
					g_link_link_length[g_number_of_links] = link_length;
					g_link_number_of_lanes[g_number_of_links] = number_of_lanes;
					g_link_mode_code[g_number_of_links] = mode_code;
					g_link_capacity_per_time_interval[g_number_of_links] = capacity_per_time_interval;
					g_link_speed[g_number_of_links] = speed;
					g_link_type[g_number_of_links]=link_type;

					g_link_departure_time_start[g_number_of_links]=link_departure_time_start;
					g_link_departure_time_end[g_number_of_links]=link_departure_time_end;
					g_link_arrival_time_start[g_number_of_links]=link_arrival_time_start;
					g_link_arrival_time_end[g_number_of_links] =link_arrival_time_end;
					g_link_demand[g_number_of_links] = link_demand;
					g_link_resource_cost[g_number_of_links] = resource_cost;
					g_link_travel_cost[g_number_of_links] = travel_cost;
					// increase the link counter by 1
					g_number_of_links++;
				}
			}

			if (parser.m_DataHubSectionName.find("Agent") != string::npos)
			{

				int agent_id = 0; 
				parser.GetValueByFieldName("agent_id", agent_id);
				parser.GetValueByFieldName("from_node_id", g_agent_origin_node[agent_id]);
				parser.GetValueByFieldName("to_node_id", g_agent_destination_node[agent_id]);
				parser.GetValueByFieldName("departure_time_start", g_agent_departure_time_beginning[agent_id]);
				parser.GetValueByFieldName("departure_time_end", g_agent_departure_time_ending[agent_id]);

				//OD and demand amount
				parser.GetValueByFieldName("from_node_id", g_agent_demand_from_node[agent_id]);
				parser.GetValueByFieldName("to_node_id", g_agent_demand_to_node[agent_id]);
				parser.GetValueByFieldName("demand_amount", g_agent_demand_amount[agent_id][0]);

				g_agent_departure_time_ending[agent_id] = max(g_agent_departure_time_ending[agent_id], g_agent_departure_time_beginning[agent_id] + 1);
	
				parser.GetValueByFieldName("arrival_time_start", g_agent_arrival_time_beginning[agent_id]);
				parser.GetValueByFieldName("arrival_time_end", g_agent_arrival_time_ending[agent_id]);

				g_agent_arrival_time_ending[agent_id] = max(g_agent_arrival_time_ending[agent_id], g_agent_arrival_time_beginning[agent_id] + 1);
				g_number_of_agents++;

			}

			if (parser.m_DataHubSectionName.find("Vehicle") != string::npos)
			{

				int vehicle_id = 0;
				parser.GetValueByFieldName("vehicle_id", vehicle_id);
				parser.GetValueByFieldName("from_node_id", g_vehicle_origin_node[vehicle_id]);
				parser.GetValueByFieldName("to_node_id", g_vehicle_destination_node[vehicle_id]);
				parser.GetValueByFieldName("departure_time_start", g_vehicle_departure_time_beginning[vehicle_id]);
				parser.GetValueByFieldName("departure_time_end", g_vehicle_departure_time_ending[vehicle_id]);

				parser.GetValueByFieldName("arrival_time_start", g_vehicle_arrival_time_beginning[vehicle_id]);
				parser.GetValueByFieldName("arrival_time_end", g_vehicle_arrival_time_ending[vehicle_id]);
				parser.GetValueByFieldName("capacity", g_vehicle_capacity[vehicle_id]);
				parser.GetValueByFieldName("fuel", g_vehicle_fuel[vehicle_id]);

				g_number_of_vehicles++;

			}

			if (parser.m_DataHubSectionName.find("Toll") != string::npos)
			{
				int from_node_id = 0;
				int to_node_id = 0;
				int start_time = 0;
				int end_time = 0;
				float toll = 0;
				parser.GetValueByFieldName("from_node_id", from_node_id);
				parser.GetValueByFieldName("to_node_id", to_node_id);
				parser.GetValueByFieldName("start_time", start_time);
				parser.GetValueByFieldName("end_time", end_time);
				parser.GetValueByFieldName("toll", toll,false);

				if (start_time > end_time)
				{
					cout << "Error: start_time > end_time in Toll data " << endl;
				}
				else
				{
								int link = g_get_link_no_based_on_from_node_to_node(from_node_id, to_node_id);

								for (int t = start_time; t <= end_time; t++)
								{
									g_external_link_time_dependent_toll[link][t] = toll;
								}

								g_number_of_toll_records++;
				}
			}

			if (parser.m_DataHubSectionName.find("Budget") != string::npos)
			{
				int vehicle_id = 0;
				parser.GetValueByFieldName("total_budget", g_total_budget);
			}


		}

		if (parser.m_DataHubSectionName.find("Configuration") != string::npos)
		{
			parser.GetValueByFieldName("number_of_iterations", g_number_of_LR_iterations);
		}
		
		cout << "read " << g_number_of_nodes << " nodes, " << g_number_of_links << " links" << ", " << g_number_of_agents << " agents, " << g_number_of_vehicles << "vehicles" << endl;
		fprintf(g_pFileDebugLog, "network has %d nodes, %d links, %d toll records, %d  agents, %d vehicles\n", 
			g_number_of_nodes, g_number_of_links, g_number_of_toll_records, g_number_of_agents, g_number_of_vehicles);
		parser.CloseCSVFile();

	}
}

bool g_best_solution_so_far[_MAX_NUMBER_OF_NODES] = {0};
float g_value_of_best_solution = 0;
float g_temp_function_value=0;
bool g_temp_solution[_MAX_NUMBER_OF_NODES] = {0};
bool g_predecessor_solution[_MAX_NUMBER_OF_NODES] = {0};
bool keep[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_BUDGET];//keep[][]
float benefit[_MAX_NUMBER_OF_NODES]={0};//benefit of this depot,v[]
int depot_set[_MAX_NUMBER_OF_NODES];//
int budget[_MAX_NUMBER_OF_NODES]={0};//w[]
float g_status_transition_matrix[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_BUDGET];//V[][]
int real_depot_id;

//sub function of knapsack, find solution
void g_find_current_depot_solution(int bigK)
{
	for(int i=0;i<_MAX_NUMBER_OF_NODES;i++)
	{
		g_temp_solution[i]=0;
	}
	for(int i=g_number_of_depot_node;i>=1;i--)
	{
		if(keep[i][bigK]==true)
		{
			g_temp_solution[depot_set[i]] = true;
			if(g_knap_sack_debugging_flag)	fprintf(g_pFileDebugLog, "	depot:%d,cost:%d; ",depot_set[i],budget[i]);
			bigK = bigK-budget[i];
		}
	}
}


float g_knapsack_problem()
{
	int interval=g_total_budget/10;
	int d=0;
	bool calculated=false;
	float MVRP_value=99999;
	//for(int i=0;i<_MAX_NUMBER_OF_NODES;i++)
	//{
	//	for(int j=0;i<_MAX_NUMBER_OF_BUDGET;j+=interval)
	//	{
	//		keep[i][j]=false;
	//	}
	//}
	//
	//test weight
	for(int i=0;i<g_number_of_depot_node;i++)
	{
		budget[i+1] = g_depot_cost[g_depot_node[i]];
		benefit[i+1] = g_depot_capacity[g_depot_node[i]];
		depot_set[i+1] = g_depot_node[i];//depot set is adopted to make regular array for knapsack DP
	}

	//step 1.1 initialize the first row of state transition matrix
	for(int b = 0; b <= g_total_budget; b += 1)
	{
		g_status_transition_matrix[0][b]=99999;
	}

	if(g_knap_sack_debugging_flag)	fprintf(g_pFileDebugLog, "++++++++++++++++Knapsack progress-->\n");
	//step 1.2 Recurision
	for(int k = 1; k <= g_number_of_depot_node; k++)
	{ 
		//d = g_depot_node[k];//find depot index in node list.this varialbe leads a little confusion, will be modified later
		for(int b = 0;b <= g_total_budget; b ++)
		{
			if(g_knap_sack_debugging_flag)		fprintf(g_pFileDebugLog, "depot:%d, budget:%d \n",k,b);
			if(budget[k]<b)// && benefit[k] + g_status_transition_matrix[k-1][b-budget[k]] > g_status_transition_matrix[k-1][b])
			{
				//find last solution
				//if(g_knap_sack_debugging_flag)	
					fprintf(g_pFileDebugLog, "	depots in solution:");
				g_find_current_depot_solution(b - budget[k]);//find last solution (g_status_transition_matrix[k-1][b-budget[k]]), this is hard to understand
				//get current depot solution
				g_temp_solution[depot_set[k]] = true;
				//if(g_knap_sack_debugging_flag)	
					fprintf(g_pFileDebugLog, "depot:%d,cost:%d; \n",depot_set[k],budget[k]);
				//solve MVRP using current depot solution, g_temp_solution must be use in MVRP, the cost of depot links which are not in the solution must be modified to 9999
				MVRP_value = g_Optimization_Lagrangian_Method_Resource_Constrained_Location_Routing_Problem();
				if(g_knap_sack_debugging_flag)	fprintf(g_pFileDebugLog, "	total transportation cost:%.2f \n",MVRP_value);
				if( MVRP_value < g_status_transition_matrix[k-1][b])//if got better solution
				{
					g_status_transition_matrix[k][b] = MVRP_value;
					keep[k][b] = true;
				}
			}
			else
			{
				g_status_transition_matrix[k][b] = g_status_transition_matrix[k-1][b];
				keep[k][b] = false;
			}
		}		
	}
		fprintf(g_pFileDebugLog, "&&&&&&& knapsack final solution:\n");
		int bigK=g_total_budget;
		for(int i=g_number_of_depot_node;i>=1;i--)
		{
			if(keep[i][bigK]==true)
			{
				g_best_solution_so_far[depot_set[i]]=true;
				bigK = bigK-budget[i];
				fprintf(g_pFileDebugLog, "Knap Sack solution: keep[%d][%d]--->depot: %d \n",i,bigK,depot_set[i]);
				
			}
		}
	return 0;
}

//pure Knapsack Problem
float g_knapsack_problem(float station_capacity_price[_MAX_NUMBER_OF_NODES],float total_budget,int g_number_of_depot)
{
	int interval=total_budget/10;
	int d=0;
	bool calculated=false;
	float MVRP_value=99999;
	g_knap_sack_debugging_flag=false;
	//for(int i=0;i<_MAX_NUMBER_OF_NODES;i++)
	//{
	//	for(int j=0;i<_MAX_NUMBER_OF_BUDGET;j+=interval)
	//	{
	//		keep[i][j]=false;
	//	}
	//}
	//


	//g_depot_node[i] indicate the node id of ith depot
	

	for(int i=0;i<g_number_of_depot_node;i++)
	{
		budget[i+1] = g_depot_cost[g_depot_node[i]];
		benefit[i+1] = -1 * station_capacity_price[g_depot_node[i]];
		depot_set[i+1] = g_depot_node[i];//depot set is adopted to make regular indexed array for knapsack DP
	}
	for(int p=0;p<_MAX_NUMBER_OF_NODES;p++)
	{
		g_best_solution_so_far[p] = false;
	}
	
	//step 1.1 initialize the first row of state transition matrix
	for(int b = 0; b <= total_budget; b += 1)
	{
		g_status_transition_matrix[0][b]=99999;
	}

	if(g_knap_sack_debugging_flag)	fprintf(g_pFileDebugLog, "++++++++++++++++Knapsack progress-->\n");
	//step 1.2 Recurision
	for(int k = 1; k <= g_number_of_depot_node; k++)
	{ 
		//d = g_depot_node[k];//find depot index in node list.this varialbe leads a little confusion, will be modified later
		for(int b = 0;b <= g_total_budget; b ++)
		{
			if(g_knap_sack_debugging_flag)		fprintf(g_pFileDebugLog, "depot:%d, budget:%d \n",k,b);
			if(budget[k]<b && benefit[k]/* * g_depot_capacity[depot_set[k]] */+ g_status_transition_matrix[k-1][b-budget[k]] >= g_status_transition_matrix[k-1][b])
			{
				//find last solution
				if(g_knap_sack_debugging_flag)	fprintf(g_pFileDebugLog, "	depots in solution:");
				g_status_transition_matrix[k][b]= benefit[k]/* * g_depot_capacity[depot_set[k]] */+ g_status_transition_matrix[k-1][b-budget[k]];				
				keep[k][b] = true;
			}
			else
			{
				g_status_transition_matrix[k][b] = g_status_transition_matrix[k-1][b];
				keep[k][b] = false;
			}
		}		
	}
		fprintf(g_pFileDebugLog, "&&&&&&& knapsack final solution:\n");
		int bigK=g_total_budget;
		for(int i=g_number_of_depot_node;i>=1;i--)
		{
			if(keep[i][bigK]==true)
			{
				//depot i is in the optimal solution
				//use depot_set[i] to track the node id
				g_best_solution_so_far[depot_set[i]]=true;
				bigK = bigK-budget[i];
				fprintf(g_pFileDebugLog, "Knap Sack solution: keep[%d][%d]--->depot: %d \n",i,bigK,depot_set[i]);
			}
		}
	return 0;
}


float g_arc_cost_for_upper_bound[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_demand_link_path_indicator[_MAX_NUMBER_OF_LINKS];
int g_demand_link_path_number_of_nodes[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_NODES];
int g_demand_link_path_node_sequence[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_NODES];
int g_demand_link_path_time_sequence[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_NODES];

int g_vehicle_path_node_sequence_for_upper_bound[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_vehicle_path_time_sequence_for_upper_bound[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_vehicle_path_number_of_nodes_for_upper_bound[_MAX_NUMBER_OF_VEHICLES] = { 0 };

int	g_vehicle_path_link_sequence_UB[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS];
int	g_vehicle_path_time_sequence_UB[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS];
int	g_vehicle_path_resource_sequence_UB[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS];
float g_vehicle_path_cost_sequence_UB[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS];

float g_agent_demand_for_vehicle_profit_for_upper_bound[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];

int g_supply_to_agent_demand_for_upper_bound[_MAX_NUMBER_OF_AGENTS];
int g_agent_current_demand_for_upper_bound[_MAX_NUMBER_OF_AGENTS];

float g_link_resource_price[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];

//supply to demand, I want to use another variable name: g_supply_to_demand
int g_supply_to_agent_demand[_MAX_NUMBER_OF_AGENTS]={0};//supply to demand for lower bound
int g_supply_to_agent_demand_UB[_MAX_NUMBER_OF_AGENTS]={0};//supply to  for upper bound

float g_link_resource_agent_usage[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];
float g_link_resource_vehicle_usage[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];

//od demand multiplier, wanna modify its name
float g_agent_demand_for_vehicle_profit[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];

//multipliers combined cost 
float g_final_arc_cost[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS]={0};

float MVRP_optimal_value;

float g_depot_link_cost[_MAX_NUMBER_OF_LINKS]={0};
float g_depot_link_cost_UB[_MAX_NUMBER_OF_LINKS]={0};


float g_Optimization_Lagrangian_Method_Resource_Constrained_Location_Routing_Problem()
{
	cout << "Preparation......" << endl;

	//step 0: initialization 
	if(g_location_routing_debugging_flag)
	{
		fprintf(g_pFileDebugLog, "step 0: initialization \n");
	}

	for(int i=0;i<_MAX_NUMBER_OF_NODES;i++)
	{
		g_depot_capacity_price[i] = 0;
	}

	for (int link = 0; link < g_number_of_links; link++)
	{
		for (int t = 0; t < _MAX_NUMBER_OF_TIME_INTERVALS; t++)
		{
			//let all link price and all usage =0, 2D array, link, time
			g_link_resource_price[link][t] = 0;

			g_link_resource_vehicle_usage[link][t] = 0;
			g_link_resource_agent_usage[link][t] = 0;

			g_link_resource_price[link][t] = 0;

			g_arc_cost[link][t] = 0;

			//For location-routing
			g_link_resource_agent_demand[link][t] = 0;
			g_agent_demand_for_vehicle_profit[link][t] = 0;

		}
	}
	
	

	for (int v = 1; v <= g_number_of_vehicles; v++)
	{//let all vehicle price =0, passengers in vehicle =0, vehicle price =0, 3D array, vehicle, link, time
		for (int link = 0; link < g_number_of_links; link++)
		{
			for (int t = 0; t < _MAX_NUMBER_OF_TIME_INTERVALS; t++)
			{
				g_vehicle_capacity_resource_price[v][link][t] = 0.0;
				g_number_of_passengers_in_vehicle[v][link][t] = 0;
				g_agent_demand_for_vehicle_price[v][link][t] = 0;
				//For Location_Routing
			}
		}
	}
	
	for (int p = 1; p <= g_number_of_agents; p++) // no vehicle has been assigned to the agent yet.
	{//the record of passengers assignment to vehicle
		g_agent_assigned_vehicle_id[p] = -1;
	}

	//for upper bound, prepare the demand link path ( g_demand_link_path_node_sequence[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_NODES] and 
	//g_demand_link_path_time_sequence[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_NODES];

	prepare_demand_link_path();

	//cout << "Start scheduling agents by Lagrangian Relaxation method" << endl;
	//cout << "Running Time:" << g_GetAppRunningTime() << endl;
	CTime g_SolutionStartTime = CTime::GetCurrentTime();
	
	//link demand calculation
	for(int link = 0;link < g_number_of_links; link++)
	{
		g_link_demand[link]=0;
	}
	int link=0;
	for (int p = 0; p < g_number_of_agents; p++)
	{
			//find agent demand link ID
			link = g_get_link_no_based_on_from_node_to_node(g_agent_demand_from_node[p], g_agent_demand_to_node[p]);
			g_link_demand[link] = g_agent_demand_amount[p][0];
	}

	SYSTEMTIME sys; 
    GetLocalTime( &sys ); 
	fprintf(g_pFileDebugLog,"%4d/%02d/%02d %02d:%02d:%02d.%03d 星期%1d\n",sys.wYear,sys.wMonth,sys.wDay,sys.wHour,sys.wMinute, sys.wSecond,sys.wMilliseconds,sys.wDayOfWeek); 
	
	//loop for each LR iteration

	for (int LR_iteration = 1; LR_iteration <= g_number_of_LR_iterations; LR_iteration++)
	{
		g_lower_bound[LR_iteration]=0;
		g_upper_bound[LR_iteration]=0;

		cout << "Lagrangian Iteration " << LR_iteration << "/" << g_number_of_LR_iterations << endl;
		//if(g_location_routing_debugging_flag)
		{
			fprintf(g_pFileDebugLog, "----------Lagrangian Iteration: %d ----------------------------------------\n", LR_iteration+1);
		}
		float TotalTripPrice = 0;
		float TotalTravelTime = 0;
		double TotalWaitingTimeCost = 0;

		// step 1: update total demand profit 
		//scan all agent demand to update demand profit

	double DemandStepSize = 1.0f / (LR_iteration + 1.0f);
		if (DemandStepSize < g_minimum_subgradient_step_size)  //1.3.1 keep the minimum step size
		{
			DemandStepSize = g_minimum_subgradient_step_size;
		}

		DemandStepSize = DemandStepSize * 10;
		
		int link_no=-1;
		for (int p = 0; p < g_number_of_agents; p++)
		{
			//find agent demand link ID
			link_no = g_get_link_no_based_on_from_node_to_node(g_agent_demand_from_node[p], g_agent_demand_to_node[p]);
			g_link_demand[link_no] = g_agent_demand_amount[p][0];
			if(link_no==-1)
				continue;
			//update agent demand link profit
			for(int t = g_agent_departure_time_beginning[p]; t < g_agent_departure_time_ending[p]; t++)
			{
				g_agent_demand_for_vehicle_profit[link_no][t]//multiplier epsilon updating
						= max(0,g_agent_demand_for_vehicle_profit[link_no][t]+(DemandStepSize *(g_agent_demand_amount[p][0] - g_supply_to_agent_demand[p])));
						//the profit for system
						if(g_location_routing_debugging_flag)
						fprintf(g_pFileDebugLog, "&&& Agent Demand @ (link %d->%d) demand = %d, supply = %d(last iteration), current profit = %.2f @ time %d\n",
							g_link_from_node_number[link_no],
							g_link_to_node_number[link_no], 
							g_agent_demand_amount[p][0], 
							g_supply_to_agent_demand[p],
							g_agent_demand_for_vehicle_profit[link_no][t],
							t);
			}
		}

		float total_RRS_capacity_price=0;
		for(int i = 0; i < g_number_of_nodes; i++)
		{
			if(g_node_type[i]==0)
			{
				//multiplier theta updating
				g_depot_capacity_price[i] = max(0,(DemandStepSize/10) * (g_depot_capacity_usage[i]));// - g_depot_capacity[i]));
				g_depot_capacity_profit_for_Knapsack[i] = min(0,g_depot_capacity_profit_for_Knapsack[i] + (DemandStepSize/10) *( g_depot_capacity_usage[i] -   g_depot_capacity[i]));
				g_depot_capacity_price[i]=g_depot_capacity_profit_for_Knapsack[i];
				//g_depot_capacity_price[i] += DemandStepSize * (g_depot_capacity_usage[i] - g_depot_capacity[i]);
				//total_RRS_capacity_price+=g_depot_capacity_price[i]*g_depot_capacity[i];
				if(g_location_routing_debugging_flag)
						fprintf(g_pFileDebugLog, "&&& depot @ (node %d) capacity = %d, usage = %d(last iteration), current price = %.2f\n,",
							i,
							g_depot_capacity[i], 
							g_depot_capacity_usage[i], 
							g_depot_capacity_profit_for_Knapsack[i]);
		
			}
		}

		////step 1.1: max profit of current iteration， this for below seems not necessary now
		int link_ID = -1;
		float max_profit = 0;
		float current_profit=0;
		for(int p = 0; p < g_number_of_agents; p++)
		{
			link_ID = g_get_link_no_based_on_from_node_to_node(g_agent_demand_from_node[p], g_agent_demand_to_node[p]);
			if(link_ID!=-1)
			{
				max_profit += g_agent_demand_for_vehicle_profit[link_ID][g_agent_departure_time_beginning[p]]
						* (g_agent_demand_amount[p][0]);
			}
		}
		

		//step 2. solve Py-1
		//With the value of multiplier θ_k^τ in current iteration, 
		//solve the Knapsack problem Py-1 with Dynamic Programing algorithm
		//Knapsack problem solving with multiplier theta, the minimum total station(depot) capacity cost within construction budget
		float OFvalue = g_knapsack_problem(g_depot_capacity_profit_for_Knapsack, g_total_budget, g_number_of_depot_node);
		for(int d = 1;d <= g_number_of_nodes; d++)
		{
			g_depot_solution[LR_iteration][d] = g_best_solution_so_far[d];
		}

		//Use the solution w_k^τ of Py-1 as the recharging station solution of current iteration.
		//associate with knapsack problem, set the forbidden depot link
		//all link directed to the stations, which are not in the recharging station solution, must be forbid 
		for(int link = 0;link < g_number_of_links; link++)
		{
			g_depot_link_cost[link]=0;
			g_depot_link_cost_UB[link]=0;
		}
		for(int d = 1;d <= g_number_of_nodes; d++)
		{
			if(g_node_type[d] == 0)
			{
				if(!g_best_solution_so_far[d])//node d is not in the recharging station solution
				{
					
					for(int n = 1; n <= g_number_of_nodes;n++)
					{
						int link = g_get_link_no_based_on_from_node_to_node(d, n);// find all links that connect to forbidden recharging station
						if(link>=0)	
						{
							g_depot_link_cost[link] = _MAX_LABEL_COST/2;
							g_depot_link_cost_UB[link] = _MAX_LABEL_COST/2;
						}
					}

				}
				else
				{
					total_RRS_capacity_price+=g_depot_capacity_price[d] * g_depot_capacity[d];
				}
			}
		}

		//// step 3. Upper bound calculation, solve Px-2
		//step 3.1: Initialization of UB 
		////get the Initial profit matrix for upperbound
		float vehicle_max_transportation_cost=-1;//for punishing unsatisfied demand
		float vehicle_transportation_cost=-1;
		//if(g_location_routing_debugging_flag)	fprintf(g_pFileDebugLog, "------ UPPER BOUND ----->\n");
		for (int link = 0; link < g_number_of_links; link++)
		{
			for (int t = 0; t < _MAX_NUMBER_OF_TIME_INTERVALS; t++)
			{
				g_agent_demand_for_vehicle_profit_for_upper_bound[link][t] = g_agent_demand_for_vehicle_profit[link][t];
			}
		}

		////initial agent demand for upper bound calculation
		for(int p = 0; p < g_number_of_agents; p++)
		{
			g_agent_current_demand_for_upper_bound[p] = g_agent_demand_amount[p][0];
		}

		for(int p = 0; p < g_number_of_agents; p++)
		{
			g_supply_to_agent_demand_for_upper_bound[p] = 0;
		}

		////step 3.2: find paths for UB
		bool upper_bound_continue_flag=true;

		for(int link = 0; link < g_number_of_links; link++)
		{
			g_link_demand_UB[link] = g_link_demand[link];
		}

		for(int i = 0;i < _MAX_NUMBER_OF_NODES;i++)
		{
			g_depot_capacity_usage_UB[i] = 0;
			g_depot_capacity_price_UB[i] = g_depot_capacity_price[i];
		}

		for(int v =1;v<=g_number_of_vehicles;v++)
		{
			g_vehicle_capacity_UB[v] = g_vehicle_capacity[v];
		}

		for(int v = 1;v <= g_number_of_vehicles; v++)
		{	
			for (int link = 0; link < g_number_of_links; link++)
			{
				int from_node = g_link_from_node_number[link];
				for (int t = 0; t < _MAX_NUMBER_OF_TIME_INTERVALS; t++)
				{
					g_arc_cost_for_upper_bound[link][t] = 0.5 * g_link_travel_cost[link]
					/*+ g_depot_capacity_price_UB[from_node]*/ - g_link_demand_UB[link] * g_agent_demand_for_vehicle_profit[link][t]+ g_depot_link_cost_UB[link];
				}
			}

			int forBreak=0;

			float path_cost_by_vehicle_v =
			g_optimal_resource_space_time_dynamic_programming_shortest_path
			(v,
			g_arc_cost_for_upper_bound,
			g_to_node_cost,
			g_vertex_waiting_cost,
			g_vehicle_origin_node[v],
			g_vehicle_departure_time_beginning[v],
			g_vehicle_departure_time_ending[v],
			g_vehicle_destination_node[v],
			g_vehicle_arrival_time_beginning[v],
			g_vehicle_arrival_time_ending[v],
			g_vehicle_path_number_of_nodes_for_upper_bound[v],
			g_vehicle_path_node_sequence_for_upper_bound[v],
			g_vehicle_path_link_sequence_UB[v],
			g_vehicle_path_time_sequence_for_upper_bound[v],
			g_vehicle_path_resource_sequence_UB[v],
			g_vehicle_path_cost_sequence_UB[v],
			0,
			g_vehicle_capacity_UB[v],
			g_path_travel_time_f);

			//g_upper_bound[LR_iteration] +=path_cost_by_vehicle_v;
			
			//write log
			if(g_location_routing_debugging_flag)
			{
				fprintf(g_pFileDebugLog, "\Vehicle %d' path has %d nodes with a transportation cost of %f and travel time of %d: ",
				v,
				g_vehicle_path_number_of_nodes_for_upper_bound[v],
				path_cost_by_vehicle_v,
				g_path_travel_time);

			/*	for (int i = 0; i < g_vehicle_path_number_of_nodes_for_upper_bound[v]; i++)
				{
					if (i != g_vehicle_path_number_of_nodes_for_upper_bound[v] - 1)
						fprintf(g_pFileDebugLog, "%d -> ", g_vehicle_path_node_sequence_for_upper_bound[v][i]);
					else
						fprintf(g_pFileDebugLog, " %d ", g_vehicle_path_node_sequence_for_upper_bound[v][i]);
				}
				fprintf(g_pFileDebugLog, "\n");
				fprintf(g_pFileDebugLog, "  time sequence:  ");
				for (int i = 0; i < g_vehicle_path_number_of_nodes_for_upper_bound[v]; i++)
				{
					if (i != g_vehicle_path_number_of_nodes_for_upper_bound[v] - 1)
						fprintf(g_pFileDebugLog, "%d -> ", g_vehicle_path_time_sequence_for_upper_bound[v][i]);
					else
						fprintf(g_pFileDebugLog, " %d", g_vehicle_path_time_sequence_for_upper_bound[v][i]);
				}

				fprintf(g_pFileDebugLog, "\n");
				fprintf(g_pFileDebugLog, "  resource sequence:  ");
				for (int i = 0; i < g_vehicle_path_number_of_nodes_for_upper_bound[v]; i++)
				{
					if (i != g_vehicle_path_number_of_nodes_for_upper_bound[v] - 1)
						fprintf(g_pFileDebugLog, "%d -> ", g_vehicle_path_resource_sequence_UB[v][i]);
					else
						fprintf(g_pFileDebugLog, " %d", g_vehicle_path_resource_sequence_UB[v][i]);
				}
*/
				fprintf(g_pFileDebugLog, "\n");
			}
				//scan path to calculate the supply to demand of current vehicle 
				int link;
				if (path_cost_by_vehicle_v > 4000) continue;
				for(int p=0;p<g_number_of_agents;p++)
				{
					if(g_supply_to_agent_demand_for_upper_bound[p]==1) continue; //pass this demand link which has already been served before@@@@@
					link=-1;
					for(int i = 0; i < g_vehicle_path_number_of_nodes_for_upper_bound[v]-1; i++)
					{
						if(g_vehicle_path_node_sequence_for_upper_bound[v][i] == g_agent_demand_from_node[p] 
						&& g_vehicle_path_time_sequence_for_upper_bound[v][i] >= g_agent_departure_time_beginning[p]
						&& g_vehicle_path_time_sequence_for_upper_bound[v][i] <= g_agent_departure_time_ending[p]
						&&
						g_vehicle_path_node_sequence_for_upper_bound[v][i+1] == g_agent_demand_to_node[p]
						&& g_vehicle_path_time_sequence_for_upper_bound[v][i+1] >= g_agent_arrival_time_beginning[p]
						&& g_vehicle_path_time_sequence_for_upper_bound[v][i+1] <= g_agent_arrival_time_ending[p]
						)//the vehicle which traveled pass the agent demand link at appropriate time
						{
							//supply of current vehicle for agent demand 
							g_supply_to_agent_demand_for_upper_bound[p] = 1;
							//demand remaining after current vehicle operation
							g_agent_current_demand_for_upper_bound[p] = 0;
							//g_vehicle_capacity_UB[v]=g_vehicle_capacity_UB[v]-g_supply_to_agent_demand_for_upper_bound[p];//if a vehicle served a demand, the capacity of this vehicle should reduce
							link_no = g_get_link_no_based_on_from_node_to_node(g_agent_demand_from_node[p], g_agent_demand_to_node[p]);
		
							g_link_demand_UB[link_no]=g_agent_current_demand_for_upper_bound[p];

							//write log
							if(g_location_routing_debugging_flag)
							fprintf(g_pFileDebugLog, "&&& Agent demand %d from %d to %d is served by vehicle %d, depart node %d @ time %d, arrive %d @ time %d current demand %d\n",
							p,
							g_agent_demand_from_node[p],
							g_agent_demand_to_node[p],
							v,
							g_agent_demand_from_node[p],
							g_vehicle_path_time_sequence_for_upper_bound[v][i],
							g_agent_demand_to_node[p],
							g_vehicle_path_time_sequence_for_upper_bound[v][i+1],
							g_agent_current_demand_for_upper_bound[p]);	
							//write log end
						}
					}
				}
				
				
				//statistic depot usage
				bool find_capacity_usage_flag = false;//each vehicle will only use depot capacity once
				int TemporaryStationRepeatRecord=-1;
				//int current_depot=-1;
				//find_capacity_usage_flag=false;
				for(int i = 0; i < g_vehicle_path_number_of_nodes_for_upper_bound[v]-1; i++)
				{
					for(int j = 0;j< g_number_of_depot_node; j++)
					{
						if(g_vehicle_path_node_sequence_for_upper_bound[v][i] == g_depot_node[j])
						{
							if(g_vehicle_path_node_sequence_for_upper_bound[v][i]==TemporaryStationRepeatRecord) break;
							g_depot_capacity_usage_UB[g_depot_node[j]]++;
							TemporaryStationRepeatRecord = g_depot_node[j];
							if(g_depot_capacity_usage_UB[TemporaryStationRepeatRecord]>=g_depot_capacity[TemporaryStationRepeatRecord])
							{
								for(int n = 1; n <= g_number_of_nodes;n++)
								{
									int link = g_get_link_no_based_on_from_node_to_node(TemporaryStationRepeatRecord, n);// find all links that connect to forbidden recharging station
									if(link>=0)	
									{
										g_depot_link_cost_UB[link] = _MAX_LABEL_COST/2;
										if(g_location_routing_debugging_flag)
											fprintf(g_pFileDebugLog, "In UB: depot %d usage %d\n", TemporaryStationRepeatRecord, g_depot_capacity_usage_UB[TemporaryStationRepeatRecord]);
									}
								}
							}
							break;
						}
					}
					//if(find_capacity_usage_flag) break;
				}
		
				//update upper bound
				vehicle_transportation_cost=0;
				for (int i = 0; i < g_vehicle_path_number_of_nodes_for_upper_bound[v]-1; i++)
				{
					link =-1;
					link = g_get_link_no_based_on_from_node_to_node(g_vehicle_path_node_sequence_for_upper_bound[v][i], g_vehicle_path_node_sequence_for_upper_bound[v][i+1]);
					if(link != -1)
					{
						vehicle_transportation_cost += 0.5 * g_link_travel_cost[link];
					}
				}
				g_upper_bound[LR_iteration] += vehicle_transportation_cost;
				vehicle_max_transportation_cost=max(vehicle_max_transportation_cost,vehicle_transportation_cost);

				int just_for_Break=0;

				
				//write log end
		}
		//deal with unsatisfied demand in upper bound, upper bound punishment
		int number_of_satisfied_demand=0;
		int number_of_unsatisfied_demand=0;
		for(int p=0;p<g_number_of_agents;p++)
		{
			if(g_agent_current_demand_for_upper_bound[p]==0)
			{
				number_of_satisfied_demand++;
			}
			else
			{
				number_of_unsatisfied_demand++;
			}
		}
		float transportation_cost_supplement;
		if(number_of_satisfied_demand>0)
		{
			//transportation_cost_supplement=punishing_multiplier*(float)number_of_unsatisfied_demand*(g_upper_bound[LR_iteration]/(float)number_of_satisfied_demand);
			transportation_cost_supplement = punishing_multiplier * (float)number_of_unsatisfied_demand * vehicle_max_transportation_cost;
		}
		else
		{
			transportation_cost_supplement=9999;
		}
		g_upper_bound[LR_iteration] += transportation_cost_supplement;

		//if(g_location_routing_debugging_flag) 
		g_objective_function_value[LR_iteration]=g_upper_bound[LR_iteration];
		fprintf(g_pFileDebugLog, "The value of Objective function: %.4f\n",g_upper_bound[LR_iteration]);
		g_upper_bound[LR_iteration] += bound_adjust ;
		////ub end
		//if(g_location_routing_debugging_flag)
		fprintf(g_pFileDebugLog, "________current UB:%.2f ----->\n",g_upper_bound[LR_iteration] );
		//if(g_location_routing_debugging_flag)
		fprintf(g_pFileDebugLog, "<----- UPPER BOUND -----\n");
		cout << "Lagrangian Iteration " << LR_iteration << ", upper Bound:" << g_upper_bound[LR_iteration] << endl;

		

		//step 2: solve Py-2, multi vehicle routing problem, lower bound
		//step 2.1 prepare the total profit of links
		//compute the final profit of current iteration
		float vehicle_min_profit=1;
		//if(g_location_routing_debugging_flag) fprintf(g_pFileDebugLog, ">>>>>> LOWER BOUND >>>>>\n");
		for(int link = 0; link < g_number_of_links; link++)
		{
			int from_node = g_link_from_node_number[link];
			for(int t = 0; t < _MAX_NUMBER_OF_TIME_INTERVALS; t++)
			{
				g_final_arc_cost[link][t]= 0.5 * g_link_travel_cost[link]
				/*+ g_depot_capacity_price[from_node] */- g_link_demand[link] * g_agent_demand_for_vehicle_profit[link][t] + g_depot_link_cost[link];

	/*			if(g_location_routing_debugging_flag)
						fprintf(g_pFileDebugLog, "&&& final profit @ (link %d->%d)  = %.2f @ time %d\n",
							g_link_from_node_number[link],
							g_link_to_node_number[link],  
							g_final_arc_cost[link][t],
							t);*/
			}
		}




		//step 2.1: shortest path for each vehicle

		bool lower_bound_continue_flag = true;
		int g_number_of_required_vehicles = g_number_of_vehicles;


		//scan all vehicle to compute the supply
		//reset supply to demand
		for(int i = 0;i < g_number_of_agents; i++)
		{
			g_supply_to_agent_demand[i]=0;
			g_agent_assigned_vehicle_id[i] = -1;
		}


		//for (int v = 1; v <= _MAX_NUMBER_OF_VEHICLES; v++)//note that the scheduling sequence does not matter  here
		for (int v = 1; v <= g_number_of_vehicles; v++)//note that the scheduling sequence does not matter  here
		{
			
			if(!lower_bound_continue_flag) //if all demand are delivered
				break;


			if(g_location_routing_debugging_flag)
			fprintf(g_pFileDebugLog, "$$$$$ LINK COST FOR Vehicle $$$$$\n");

			// set arc cost for vehicles

			int gg=g_final_arc_cost[0][24];

			float path_cost_by_vehicle_v =
			g_optimal_resource_space_time_dynamic_programming_shortest_path
			(v,
			g_final_arc_cost,
			g_to_node_cost,
			g_vertex_waiting_cost,
			g_vehicle_origin_node[v],
			g_vehicle_departure_time_beginning[v],
			g_vehicle_departure_time_ending[v],
			g_vehicle_destination_node[v],
			g_vehicle_arrival_time_beginning[v],
			g_vehicle_arrival_time_ending[v],
			g_vehicle_path_number_of_nodes[v],
			g_vehicle_path_node_sequence[v],
			g_vehicle_path_link_sequence[v],
			g_vehicle_path_time_sequence[v],
			g_vehicle_path_resource_sequence[v],
			g_vehicle_path_cost_sequence[v],
			0,
			g_vehicle_capacity[v],
			g_path_travel_time_f);

			g_lower_bound[LR_iteration] += path_cost_by_vehicle_v;
			vehicle_min_profit = min(vehicle_min_profit,path_cost_by_vehicle_v);

			g_number_of_required_vehicles = v;

			//summarize the demand supplied by current vehicle
			for(int p=0;p<g_number_of_agents;p++)
			{
				int link=-1;
				//for(int v = 1;v <= g_number_of_required_vehicles; v++)
				{
					for(int i = 0; i < g_vehicle_path_number_of_nodes[v]-1; i++)
					{
						if(g_vehicle_path_node_sequence[v][i] == g_agent_demand_from_node[p] 
						&& g_vehicle_path_time_sequence[v][i] >= g_agent_departure_time_beginning[p]
						&& g_vehicle_path_time_sequence[v][i] <= g_agent_departure_time_ending[p]
						&&
						g_vehicle_path_node_sequence[v][i+1] == g_agent_demand_to_node[p]
						&& g_vehicle_path_time_sequence[v][i+1] >= g_agent_arrival_time_beginning[p]
						&& g_vehicle_path_time_sequence[v][i+1] <= g_agent_arrival_time_ending[p]
						)
						//the vehicle which traveled pass the agent demand link at appropriate time
						{	
							//supply for agent demand
							g_supply_to_agent_demand[p] += g_vehicle_capacity[v];
							g_agent_assigned_vehicle_id[p] = v;//not necessary

							link_no = g_get_link_no_based_on_from_node_to_node(g_agent_demand_from_node[p], g_agent_demand_to_node[p]);

							current_profit += min(g_agent_demand_amount[p][0],g_vehicle_capacity[v]) * g_agent_demand_for_vehicle_profit[link_no][g_agent_departure_time_beginning[p]];

							if(g_location_routing_debugging_flag)
							fprintf(g_pFileDebugLog, "&&& Agent demand %d from %d to %d is served by vehicle %d, depart node %d @ time %d, arrive %d @ time %d\n",
								p,
								g_agent_demand_from_node[p],
								g_agent_demand_to_node[p],
								v,
								g_agent_demand_from_node[p],
								g_vehicle_path_time_sequence[v][i],
								g_agent_demand_to_node[p],
								g_vehicle_path_time_sequence[v][i+1]);							
						}
					}
				}
			}

			

		/*	if(current_profit >= max_profit)
			{
				lower_bound_continue_flag = false;
				g_number_of_required_vehicles = v;
			}*/

			if(g_location_routing_debugging_flag)
			{
				fprintf(g_pFileDebugLog, "\Vehicle %d'  path has %d nodes with a transportation cost of %f and travel time of %d: ",
					v,
					g_vehicle_path_number_of_nodes[v],
					path_cost_by_vehicle_v,
					g_path_travel_time);

				for (int i = 0; i < g_vehicle_path_number_of_nodes[v]; i++)
				{
					if (i != g_vehicle_path_number_of_nodes[v] - 1)
						fprintf(g_pFileDebugLog, "%d -> ", g_vehicle_path_node_sequence[v][i]);
					else
						fprintf(g_pFileDebugLog, " %d ", g_vehicle_path_node_sequence[v][i]);
			
				}

				fprintf(g_pFileDebugLog, "  time sequence:  ");
				for (int i = 0; i < g_vehicle_path_number_of_nodes[v]; i++)
				{
					if (i != g_vehicle_path_number_of_nodes[v] - 1)
						fprintf(g_pFileDebugLog, "%d -> ", g_vehicle_path_time_sequence[v][i]);
					else
						fprintf(g_pFileDebugLog, " %d", g_vehicle_path_time_sequence[v][i]);
				}

				fprintf(g_pFileDebugLog, "\n");

				fprintf(g_pFileDebugLog, "  resource sequence:  ");
				for (int i = 0; i < g_vehicle_path_number_of_nodes[v]; i++)
				{
					if (i != g_vehicle_path_number_of_nodes[v] - 1)
						fprintf(g_pFileDebugLog, "%d -> ", g_vehicle_path_resource_sequence[v][i]);
					else
						fprintf(g_pFileDebugLog, " %d", g_vehicle_path_resource_sequence[v][i]);
				}

				fprintf(g_pFileDebugLog, "\n");
			}
		}

		//deal with unsatisfied demand in lower bound, lower bound punishment
			number_of_satisfied_demand=0;
			number_of_unsatisfied_demand=0;
			for(int p=0;p<g_number_of_agents;p++)
			{
				number_of_satisfied_demand += g_supply_to_agent_demand[p];
			}
			
			if(number_of_satisfied_demand < g_number_of_agents)
			{
				if(number_of_satisfied_demand!=0)
				{
					transportation_cost_supplement=punishing_multiplier *
						//(float)(g_number_of_agents - number_of_satisfied_demand)* (g_lower_bound[LR_iteration]/(float)number_of_satisfied_demand);
						(float)(g_number_of_agents - number_of_satisfied_demand)* vehicle_min_profit;
				}
				else
				{
					transportation_cost_supplement = -9999;
				}
			}
			g_lower_bound[LR_iteration] += transportation_cost_supplement;


		if(g_location_routing_debugging_flag)	fprintf(g_pFileDebugLog, "Max profit %.3f\n",max_profit);

		// step 2.2: scan agent demand for lower bound 
		
		if(g_location_routing_debugging_flag) fprintf(g_pFileDebugLog, "&&&&& SUPPLY¡¡FOR  DEMAND &&&&&\n");
	
		//summarize total profit of current iteration

		//scan all depot node in paths, compute the depot usage
		for(int i = 0;i < _MAX_NUMBER_OF_NODES;i++)
		{
			g_depot_capacity_usage[i] = 0;
		}

		bool find_capacity_usage_flag = false;
 		int TemporaryStationRepeatRecord=-1;
		for(int v = 1;v <= g_number_of_required_vehicles; v++)
		{
			//find_capacity_usage_flag=false;

			for(int i = 0; i < g_vehicle_path_number_of_nodes[v]-1; i++)
			{
				for(int j = 0;j< g_number_of_depot_node; j++)
				{
					if(g_vehicle_path_node_sequence[v][i] == g_depot_node[j])
					{
						if(g_vehicle_path_node_sequence_for_upper_bound[v][i]==TemporaryStationRepeatRecord) break;
						g_depot_capacity_usage[g_depot_node[j]]++;
						//find_capacity_usage_flag=true;
						TemporaryStationRepeatRecord= g_depot_node[j];
						break;
					}
				}
				//if(find_capacity_usage_flag) break;
			}
		}


		if(g_location_routing_debugging_flag)	fprintf(g_pFileDebugLog, "&&&&& SUPPLY¡¡FOR  DEMAND END &&&&&\n");   

		//step 2.3:lower bound calculation
		//lower bound. profit. sum of multiplier * (demand - capacity)   (demand constraints)

		g_lower_bound[LR_iteration] = g_lower_bound[LR_iteration] + max_profit + total_RRS_capacity_price;
		
		for(int i=0;i<g_number_of_nodes;i++)
		{
			if(g_depot_capacity_price[i]<_MAX_LABEL_COST-1)
				g_lower_bound[LR_iteration] = g_lower_bound[LR_iteration];// - g_depot_capacity_price[i] * g_depot_capacity[i];
		}

		//for making a negative lower bound to positive 

		g_lower_bound[LR_iteration] += bound_adjust;
		//g_lower_bound[LR_iteration] = min(g_lower_bound[LR_iteration], g_lower_bound[LR_iteration - 1]);

	
		//g_upper_bound[LR_iteration] += bound_adjust;
		//g_upper_bound[LR_iteration] = min(g_upper_bound[LR_iteration], g_upper_bound[LR_iteration - 1]);

		//g_lower_bound[LR_iteration] = g_lower_bound[LR_iteration] + g_number_of_vehicles * g_vehicle_price;

		//g_lower_bound[LR_iteration]=max(g_lower_bound[LR_iteration],g_lower_bound[LR_iteration-1]);
		
		cout << "Lagrangian Iteration " << LR_iteration << ", Lower Bound:" << g_lower_bound[LR_iteration] << endl;
		//if(g_location_routing_debugging_flag)	
			fprintf(g_pFileDebugLog, "________current LB:%.2f ----->\n",g_lower_bound[LR_iteration]);
		//if(g_location_routing_debugging_flag)
			fprintf(g_pFileDebugLog, "<<<<<< LOWER BOUND <<<<<<<<\n");

		
		//step 4: gap calculation
		/*if(LR_iteration>6)
		{
			if(g_upper_bound[LR_iteration] >= g_upper_bound[LR_iteration-1])
				g_upper_bound[LR_iteration] = g_upper_bound[LR_iteration-1];
			if(g_lower_bound[LR_iteration] <= g_lower_bound[LR_iteration-1])
				g_lower_bound[LR_iteration] = g_lower_bound[LR_iteration-1];
		}*/


		g_gap[LR_iteration]=(g_upper_bound[LR_iteration] - g_lower_bound[LR_iteration])/g_upper_bound[LR_iteration];
		if(LR_iteration>6)
		{
			if(g_gap[LR_iteration]>g_gap[LR_iteration-1])
			{
				g_gap[LR_iteration]=g_gap[LR_iteration-1];
				g_lower_bound[LR_iteration]=g_lower_bound[LR_iteration-1];
				g_upper_bound[LR_iteration]=g_upper_bound[LR_iteration-1];
				g_objective_function_value[LR_iteration]=g_objective_function_value[LR_iteration-1];
				for(int d = 1;d <= g_number_of_nodes; d++)
				{
					g_depot_solution[LR_iteration][d] = g_depot_solution[LR_iteration-1][d];
				}
				
			}
		}

		MVRP_optimal_value = g_upper_bound[LR_iteration];
		cout << "Lagrangian Iteration " << LR_iteration << ", gap:" << g_gap[LR_iteration] << endl;
		
		/*if(LR_iteration>10 && (g_lower_bound[LR_iteration] - g_lower_bound[LR_iteration-1])<0.001)
			break;*/

		//		g_LogFile << "Computational time:," << ctime.GetTotalSeconds() << ",Iteration:, " << LR_iteration + 1 << ",Lower bound in Minute:," << globallowerbound / g_MinuteDivision << ",Upper bound in Minute:," << globalupperbound / g_MinuteDivision << ",Lower bound:," << globallowerbound << ",Upper bound:," << globalupperbound << ",Total Trip Price:," << TotalTripPrice << ",Total Resource Price (Resource Cost):," << TotalResourcePrice << ",Total Travel Time:," << TotalTravelTime << ",Optimality gap:," << (globalupperbound - globallowerbound) / globalupperbound << endl;
	}



	

		//if(g_location_routing_debugging_flag)
		{
			fprintf(g_pFileDebugLog,"Upper bound sequence:\n");
			for(int i =6;i<_MAX_NUMBER_OF_ITERATIONS;i++)
			{
				fprintf(g_pFileDebugLog,"%.3f	", g_upper_bound[i]);
			}

			fprintf(g_pFileDebugLog,"\n");

			fprintf(g_pFileDebugLog,"Lower bound sequence:\n");
			for(int i =6;i<_MAX_NUMBER_OF_ITERATIONS;i++)
			{
				fprintf(g_pFileDebugLog,"%.3f	",g_lower_bound[i]);
			}
			fprintf(g_pFileDebugLog,"\n");

			fprintf(g_pFileDebugLog,"Relative Gap (%) sequence:\n");
			for(int i =6;i<_MAX_NUMBER_OF_ITERATIONS;i++)
			{
				fprintf(g_pFileDebugLog,"%.1f	",g_gap[i]*100);
			}
			fprintf(g_pFileDebugLog,"\n\n");

			fprintf(g_pFileDebugLog,"Objective function value sequence:\n");
			for(int i =6;i<_MAX_NUMBER_OF_ITERATIONS;i++)
			{
				fprintf(g_pFileDebugLog,"%.1f	",g_objective_function_value[i]);
			}
			fprintf(g_pFileDebugLog,"\n\n");

			fprintf(g_pFileDebugLog,"RRS solution sequence:\n");
			for(int i =6;i<_MAX_NUMBER_OF_ITERATIONS;i++)
			{
				for(int d = 1;d <= g_number_of_nodes; d++)
				{
					if(g_depot_solution[i][d])
					{
						fprintf(g_pFileDebugLog,"[%d]",d);
					}
				}
				fprintf(g_pFileDebugLog,"; ");
			}
			fprintf(g_pFileDebugLog,"\n\n");
			
		}
	float ig=1;
	for(int i =6;i<_MAX_NUMBER_OF_ITERATIONS;i++)
	{
		if(g_gap[i]<ig && g_gap[i]>0)
		{
			ig=g_gap[i];
		}
	}
	if(g_location_routing_debugging_flag)
	{
		fprintf(g_pFileDebugLog,"# of demand links\t # of vehicles\t relative gap\n");
		fprintf(g_pFileDebugLog,"%d\t %d\t %.3f\n",g_number_of_agents,g_number_of_vehicles,ig);
        GetLocalTime( &sys ); 
		fprintf(g_pFileDebugLog,"%4d/%02d/%02d %02d:%02d:%02d.%03d 星期%1d\n",sys.wYear,sys.wMonth,sys.wDay,sys.wHour,sys.wMinute, sys.wSecond,sys.wMilliseconds,sys.wDayOfWeek); 
	}
	cout << "End of Lagrangian Iteration Process " << endl;
	//cout << "Running Time:" << g_GetAppRunningTime() << endl;

	return MVRP_optimal_value;
	
}


//for upper bound link path preparation
void prepare_demand_link_path()
{
	//get the ture paths of agent demand links
	int demand_link_count=0;

	for (int link = 0; link < g_number_of_links; link++)
	{
		for (int t = 0; t < _MAX_NUMBER_OF_TIME_INTERVALS; t++)
		{
			if(g_link_type[link]==4)
			{
				g_arc_cost[link][t] = _MAX_LABEL_COST;//forbid the agent demand link
			}
		}
		g_demand_link_path_indicator[link]= -1;//indicator initiation
	}

	for(int link=0;link<g_number_of_links;link++)
	{
		if(g_link_type[link]==4)
		{
			g_demand_link_path_indicator[link]=demand_link_count;

			float path_cost_by_vehicle_v =
			g_optimal_time_dependenet_label_correcting(
				g_arc_cost,
				g_link_from_node_number[link],
				0,
				0,
				g_link_to_node_number[link],
				0,
				_MAX_NUMBER_OF_TIME_INTERVALS,
				g_vehicle_path_number_of_nodes[demand_link_count],
				g_vehicle_path_node_sequence[demand_link_count],
				g_vehicle_path_time_sequence[demand_link_count],
				0,
				g_path_travel_time,false);

			demand_link_count++;
		}
	}

	for (int link = 0; link < g_number_of_links; link++)
	{
		for (int t = 0; t < _MAX_NUMBER_OF_TIME_INTERVALS; t++)
		{
			if(g_link_type[link]==4)
			{
				g_arc_cost[link][t] = 0;
			}
		}

	}

}
	



void g_allocate_memory()
{
	
	l_state_node_label_cost = Allocate3DDynamicArray<float>(_MAX_NUMBER_OF_NODES, _MAX_NUMBER_OF_TIME_INTERVALS, _MAX_NUMBER_OF_RESOURCES);
	
	l_state_node_predecessor= Allocate3DDynamicArray<int>(_MAX_NUMBER_OF_NODES, _MAX_NUMBER_OF_TIME_INTERVALS, _MAX_NUMBER_OF_RESOURCES);

	l_state_time_predecessor= Allocate3DDynamicArray<int>(_MAX_NUMBER_OF_NODES, _MAX_NUMBER_OF_TIME_INTERVALS, _MAX_NUMBER_OF_RESOURCES);

	l_state_resource_predecessor= Allocate3DDynamicArray<int>(_MAX_NUMBER_OF_NODES, _MAX_NUMBER_OF_TIME_INTERVALS, _MAX_NUMBER_OF_RESOURCES);
//float l_state_node_label_cost[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS][_MAX_NUMBER_OF_RESOURCES] = {0};

}


void g_free_memory()
{
	
	Deallocate3DDynamicArray<float>(l_state_node_label_cost,  _MAX_NUMBER_OF_NODES, _MAX_NUMBER_OF_TIME_INTERVALS);
	Deallocate3DDynamicArray<int>(l_state_node_predecessor,  _MAX_NUMBER_OF_NODES, _MAX_NUMBER_OF_TIME_INTERVALS);
	Deallocate3DDynamicArray<int>(l_state_time_predecessor,  _MAX_NUMBER_OF_NODES, _MAX_NUMBER_OF_TIME_INTERVALS);
	Deallocate3DDynamicArray<int>(l_state_resource_predecessor,  _MAX_NUMBER_OF_NODES, _MAX_NUMBER_OF_TIME_INTERVALS);
		
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





	g_GenerateGAMSInputData();

	g_pFileDebugLog = fopen("C:\\AgentPlusRRS-LRP\\Debug.txt", "w");
	g_ReadInputData("C:\\AgentPlusRRS-LRP\\VRPDP.csv");
	
	g_allocate_memory();

	//DP testing
	/*for (int link = 0; link < g_number_of_links; link++)
	{
		for (int t = 0; t < _MAX_NUMBER_OF_TIME_INTERVALS; t++)
		{
			g_arc_cost[link][t] =   g_link_free_flow_travel_time[link];
		}
	}

	for (int link = 0; link < g_number_of_links; link++)
	{
		for (int t = 0; t < _MAX_NUMBER_OF_TIME_INTERVALS; t++)
		{
			l_arc_cost[link][t] = 0.5 *  g_link_travel_cost[link];
		}
	}
*/
	

	/*float ff=g_optimal_resource_space_time_dynamic_programming_shortest_path(1,
		l_arc_cost,
		g_to_node_cost,
		g_vertex_waiting_cost,
		g_vehicle_origin_node[1],
		g_vehicle_departure_time_beginning[1],
		g_vehicle_departure_time_ending[1],
		g_vehicle_destination_node[1],
		g_vehicle_arrival_time_beginning[1],
		g_vehicle_arrival_time_ending[1],
		g_vehicle_path_number_of_nodes[1],
		g_vehicle_path_node_sequence[1],
		g_vehicle_path_link_sequence[1],
		g_vehicle_path_time_sequence[1],
		g_vehicle_path_resource_sequence[1],
		g_vehicle_path_cost_sequence[1],
		0,
		g_vehicle_capacity[1],
		g_path_travel_time_f);*/
	//	DP test end

		/*float path_cost_by_vehicle_v =
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
				g_vehicle_path_time_sequence[v],
				0,
				g_path_travel_time,true);*/



	float value = g_Optimization_Lagrangian_Method_Resource_Constrained_Location_Routing_Problem();
	//g_knapsack_problem();
	//g_Optimization_Lagrangian_Method_Link_Capacity_Problem();
	fprintf(g_pFileDebugLog, "Program Ends.\n");
	fclose(g_pFileDebugLog);


	g_free_memory();

	return nRetCode;
}
