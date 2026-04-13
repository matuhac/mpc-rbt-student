#include "Planning.hpp"
#include <chrono>
#include "mpc_rbt_simulator/RobotConfig.hpp"

#include <vector>
#include <queue>
#include <tuple>
#include <cmath>
#include <limits>
#include <algorithm>
#include <functional>

//using namespace std::placeholders;

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {
    
    	dilated = 0;

        // Client for map
	map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

        // Service for path
        plan_service_ = create_service<nav_msgs::srv::GetPlan>(
        	"/plan_path", std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2));
        
        // Publisher for path
        path_pub_ = create_publisher<nav_msgs::msg::Path>("planned_path", 10);

        RCLCPP_INFO(get_logger(), "Planning node started.");

        // Connect to map server
        while(!map_client_->wait_for_service(std::chrono::seconds(5))) 
        {
        	RCLCPP_WARN(this->get_logger(), "Waiting for map server...");
        }     

        // Request map
        auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
        //asynchronni odeslani pozadavku na mapu 
        map_client_->async_send_request(request, std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "Trying to fetch map...");
        
    }

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    // add code here
    
    RCLCPP_INFO(get_logger(), "Callback called!");

    // ********
    // * Help *
    // ********
    
    auto response = future.get();
    if (response) 
    {
        RCLCPP_INFO(get_logger(), "Mapa ziskana!");  
        map_ = response->map;
        	
        //info o mape
        
        int width 	= map_.info.width;
        int height 	= map_.info.height;
        double res 	= map_.info.resolution;
        double ox 	= map_.info.origin.position.x;
        double oy 	= map_.info.origin.position.y;
 	/*
        RCLCPP_INFO(get_logger(), "--MAP INFO--");
        RCLCPP_INFO(get_logger(), "Size:	%d x %d cells", width, height);
        RCLCPP_INFO(get_logger(), "Resolution:	%.4f m/cell", res);
        RCLCPP_INFO(get_logger(), "Origin:	(%.2f, %.2f) metres", ox, oy);
        RCLCPP_INFO(get_logger(), "WorldSize:	%.2f x %.2f metres", width * res, height * res);
        RCLCPP_INFO(get_logger(), "================================");
        
        for(int row1 = height - 1; row1 >= 0; row1--){
		std::string line = "";
		for(int col1 = 0; col1 < width; col1++){
			int8_t val = map_.data[row1*width+col1];
			if	(val <  0) line+="?";
			else if	(val == 0) line+=".";
			else		   line+="#";
		}
	RCLCPP_INFO(get_logger(), "%s", line.c_str());
        }   
        */
        
        //spocitej cestu, nahradime dilataci az pote bude aStar
        dilateMap();
        /*
       for(int row1 = height - 1; row1 >= 0; row1--){
		for(int col1 = width - 1; col1 >= 0; col1--){
			
			float wx = ox + (col1) * res;
			float wy = oy + (row1) * res;
		
			int8_t val = map_.data[row1*width+col1];
			if(val == 100 && wx > 0.0){
				RCLCPP_INFO(get_logger(), "x: %f y: %f Occ: %d", wx, wy, val);
			}
			
		}
	}
        */
        RCLCPP_INFO(get_logger(), "Dilatace konec");
        
    }
    
    
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {
    
	RCLCPP_INFO(get_logger(), "Volam aStar");
	//zavolej aStar
	aStar(request->start, request->goal);
	RCLCPP_INFO(get_logger(), "aStar hotov");
	//Path smooting
	smoothPath();
	//publikace
	path_pub_->publish(path_);
	response->plan = path_;
	
	RCLCPP_INFO(get_logger(), "Path size: %d", path_.poses.size());
	RCLCPP_INFO(get_logger(), "Path Published");
}

void PlanningNode::dilateMap() {    
    	RCLCPP_INFO(get_logger(), "Dilatace Start");
    
    	//parametry mapy
	int width 	= map_.info.width;
	int height 	= map_.info.height;
	double res 	= map_.info.resolution;

	auto dilatedCheck = map_;
	
	double radiusSet = 0.60; //radius pro dilataci, v metrech
	int radius = (int)std::ceil(radiusSet/res); //prevod do integeru, budeme potrebovat indexy
	
	RCLCPP_INFO(get_logger(), "Parametry dilatace: %.4f, %d", radiusSet, radius);
	
	RCLCPP_INFO(get_logger(), "map_.data.size()=%zu, width*height=%d",
        map_.data.size(), map_.info.width * map_.info.height);
        
        size_t dilatedSize = width * height;
    
    //pruchod vsemi souradnicemi
    for(int row = 0; row < height; row++){
    	//RCLCPP_INFO(get_logger(), "New Row: %d", row);
    	for(int col = 0; col < width; col++){
	    	
	    	int cur_idx = row * width + col; //index soucasneho prvku
	    	
	    	if(dilatedCheck.data[cur_idx] != 100){continue;} //pokud je prvek volny nic nedelame
	    	
	    	//spocitat polomer a vse v polomeru oznacit za sousedy
	    	for(int r1 = -radius; r1 <= radius; r1++){
	    		for(int r2 = -radius; r2 <= radius; r2++){
	    		
	    			int new_row = row + r1;
	    			int new_col = col + r2;
	    			//jsme li mimo mapu tak nedelame nic
				if (new_col < 0 || new_col >= width) {continue;}
				if (new_row < 0 || new_row >= height) {continue;}
	    			if(new_row == row && new_col == col) {continue;} //nebudeme dilatovat sami sebe
	    			
	    			double distance = sqrt((new_row-row)*(new_row-row) + (new_col-col)*(new_col-col));
	    			//RCLCPP_INFO(get_logger(), "Distance: %f", distance);
	    			
	    			if(distance <= radius){
	    				int index = new_row * width + new_col;
	    			        //Kontrola indexu
					if (index < 0 || index >= (int)dilatedSize) { 
						//RCLCPP_ERROR(get_logger(), "zapis mimo pole: idx=%d, size=%zu, " "new_col=%d, new_row=%d", index, dilatedSize, new_col, new_row);
						continue;
        				}
	    			
	    				//RCLCPP_INFO(get_logger(), "Point Dilated!");
	    				map_.data[index] = 100; //oznacime bod za obsazeny
	    			}
	    		}
	    	}
    	}
    }
    RCLCPP_INFO(get_logger(), "Dilatace Spocitana!");
}


void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {

	RCLCPP_INFO(get_logger(), "A star called!");

	//inicializace dat o mape
	int width 	= map_.info.width;
	int height 	= map_.info.height;
	double res 	= map_.info.resolution;
	double ox 	= map_.info.origin.position.x;
	double oy 	= map_.info.origin.position.y;
	
	size_t total = width * height;
	
	RCLCPP_INFO(get_logger(), "map info");
	RCLCPP_INFO(get_logger(), "total: %d", total);
	
	if (map_.data.size() != total) {
		RCLCPP_ERROR(get_logger(),
		"Map data size mismatch! data.size()=%zu but width*height=%zu",
		map_.data.size(), total);
		return;
	}

	//souradnice startu a cile prevedeny na grid
	int start_col = (int)((start.pose.position.x - ox) / res);
	int start_row = (int)((start.pose.position.y - oy) / res);
	int goal_col  = (int)((goal.pose.position.x - ox) / res);
	int goal_row  = (int)((goal.pose.position.y - oy) / res);
	
	//pomocne vektory atd.
	std::vector<bool> closed(total, false); //sdruzuje uz testovane nody
	std::vector<int> g(total, std::numeric_limits<int>::max()); //cost pro dosazeni soucasneho nodu ze startu
	
	//parent pozice, pro zpetne vykresleni		
	std::vector<int> parent_col(total, -1);
	std::vector<int> parent_row(total, -1);
	
	
	//heuristicka funkce, pocita manhattan vzdalenost od soucasneho nodu k cili
	auto heuristic = [&](int col, int row) -> int { return std::abs(col-goal_col) + std::abs(row - goal_row);};
	
	
	//Kontrola zda je bod volny nebo ne
	auto isFree = [&](int col, int row) -> bool {
		//kontrola zda bod nelezi mimo mapu
		if (col < 0 || col >= width) return false;
		if (row < 0 || row >= height) return false;
		//kontrola validniho bodu zda je obsazen, 0 true, -1 nebo 100 false
		int8_t val = map_.data[row*width+col];
		return (val == 0);
	};
	
	
	//			    f   col  row
	using prvek = std::tuple<int, int, int>;
	//Drzi pozice,ktere byly zpracovany, bude je radit dle f
	std::priority_queue<prvek, 
		std::vector<prvek>, 
		std::greater<prvek>> open_set;
	
	
	int start_idx = start_row * width +start_col;
	g[start_idx] = 0; //startovni pozice ma g cost nula
	//             spocitej heuristiku zacatku,        souradnice zacatku
	open_set.push({heuristic(start_col, start_row), start_col, start_row}); //push start nodu do open setu
	
	//RCLCPP_INFO(get_logger(), "prior queue");
	
	
	
	//4-okoli, temito smery budeme zkoumat okoli
	//const std::vector<std::pair<int,int>> directions = {{1,0},{-1,0},{0,1},{0,-1}, {1,1},{-1,-1,},{1,-1},{-1,1}}; //vpravo, vlevo, nahoru, dolu, diagonaly
	const std::vector<std::pair<int,int>> directions = {{1,0},{-1,0},{0,1},{0,-1}}; //vpravo, vlevo, nahoru, dolu
	
	bool found = false; //promenna co indikuje nalezeni cile
	
	//int counter = 0;
	
	RCLCPP_INFO(get_logger(), "aStar algoritmus zacina");
	
	//HLAVNI CYKLUS
	while(!open_set.empty()) {
		//counter++;
		//RCLCPP_WARN(get_logger(), "%d", counter);
	
		auto [f_cur, cur_col, cur_row] = open_set.top(); //z open setu vyberem node s nejvyssi prioritou (nejmensi f)
		open_set.pop(); //tento prvek odstranime z fronty
		
		int cur_idx = cur_row * width + cur_col; //index soucasneho prvku
		
		if(closed[cur_idx]) continue; //je li prvek uz closed tak ho preskocime
		closed[cur_idx] = true; //pridame prvek do uz testovanych
		
		//testovani nalezeni cile
		if(cur_col == goal_col && cur_row == goal_row){
			RCLCPP_INFO(get_logger(), "Cil Nalezen");
			found = true;
			break;
		}
		
		for(auto[dir_col, dir_row] : directions) {
			//souradnice sousedu
			int neighbor_col = cur_col + dir_col;
			int neighbor_row = cur_row + dir_row;
			
			//if(neighbor_row > width  || neighbor_row < 0) {continue;}
	    		//if(neighbor_col > height || neighbor_col < 0) {continue;}
			
			//testovani zda je soused volna pozice
			if(!isFree(neighbor_col, neighbor_row)) continue; //preskocit
			
			int neighbor_idx = neighbor_row * width + neighbor_col; //index souseda
			if(closed[neighbor_idx]) continue; //byl soused uz testovany?
			
			int g_new = g[cur_idx] + 1; //g cost se proste pricte jednicka
			//int g_new = g[cur_idx] + sqrt(dir_row*dir_row + dir_col*dir_col); // 
				
			//je nova g cost mensi nez u sousedu?
			if(g_new < g[neighbor_idx]) {

				//souradnice ze kterych jsme vstopili do souseda, toto budou waypointy
				g[neighbor_idx]		= g_new; //celkova g_cost od zacatku po tento node
				parent_col[neighbor_idx]=cur_col;
				parent_row[neighbor_idx]=cur_row;

				//cost heuristika noveho bodu 
				int f_new = g_new + heuristic(neighbor_col, neighbor_row);
				//zaradime souseda do open setu
				open_set.push({f_new, neighbor_col, neighbor_row});
			}			
		}
		
	}
	
	//Hlaska cil nenaleze
	if(!found){
		RCLCPP_WARN(get_logger(), "Goal not found");
		return;
	}
	//vytvoreni zpravy
	path_.poses.clear();
	path_.header.stamp = now();
	path_.header.frame_id = map_.header.frame_id;
	
	int cur_col = goal_col;
	int cur_row = goal_row;
	
	int count_waypoints = 0;
	//rekonstrukce cesty
	while(!(cur_col == start_col && cur_row == start_row)){
		geometry_msgs::msg::PoseStamped ps;
		ps.header = path_.header;
		
		ps.pose.position.x = ox + (cur_col + 0.5) * res;
		ps.pose.position.y = oy + (cur_row + 0.5) * res;
		ps.pose.orientation.w = 1.0;
		
		path_.poses.push_back(ps);
		
		int idx = cur_row * width + cur_col;
		int p_col = parent_col[idx];
		int p_row = parent_row[idx];
		cur_col = p_col;
		cur_row = p_row;
		
		count_waypoints++;
	}
	RCLCPP_WARN(get_logger(), "Num. of waypoints: %d", count_waypoints);
	
	//zprava
	geometry_msgs::msg::PoseStamped ps;
	ps.header = path_.header;
	ps.pose.position.x = ox + (start_col + 0.5) * res;
	ps.pose.position.y = oy + (start_row + 0.5) * res;
	ps.pose.orientation.w = 1.0;
	path_.poses.push_back(ps);
	//otoceni cesty
	std::reverse(path_.poses.begin(), path_.poses.end());
	
	//path_pub_->publish(path_);
	//RCLCPP_INFO(get_logger(), "Path Published");
	
	
    // add code here
    // ********
    // * Help *
    // ********
    /*
    Cell cStart(...x-map..., ...y-map...);
    Cell cGoal(...x-map..., ...y-map...);

    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(map_.info.height * map_.info.width, false);

    openList.push_back(std::make_shared<Cell>(cStart));

    while(!openList.empty() && rclcpp::ok()) {
        ...
    }

    RCLCPP_ERROR(get_logger(), "Unable to plan path.");
    */
}

void PlanningNode::smoothPath() {


	//smoothing potrebuje alespon 3 body
	if (path_.poses.size() < 3) {
	RCLCPP_WARN(get_logger(), "Path too short to smooth, skipping.");
	return;
	}


	//velikost cesty
	int N = path_.poses.size();

	std::vector<double> x(N), y(N);           //smoothed positions
	std::vector<double> orig_x(N), orig_y(N); //original positions

	//inicializace smoothed pozici pocatecnimi pozicemi
	for (int i = 0; i < N; i++) {
	x[i] = orig_x[i] = path_.poses[i].pose.position.x;
	y[i] = orig_y[i] = path_.poses[i].pose.position.y;
	}
	
	//parametry pro smoothing
	//alpha  - jak moc je bod pritahovan k puvodni pozici, vetsi alpha mensi rozdil od puvodni cesty
	//beta	 - jak moc je bod pritahovan k sousedum
	//tolerance -> pokud zmena klesne pod tuto hodnotu, pak algoritmus konci

	double alpha     = 0.1;
	double beta      = 0.6;
	double tolerance = 1e-6;
	int    max_iter  = 500;

	//ALGORITMUS

	for (int iter = 0; iter < max_iter; iter++) {

	double total_change = 0.0;

		for (int i = 1; i < N - 1; i++) {

		    //nova x souradnice
		    double new_x = x[i]
			+ alpha * (orig_x[i] - x[i])
			+ beta  * (x[i-1] + x[i+1] - 2.0 * x[i]);

		    //nova y souradnice
		    double new_y = y[i]
			+ alpha * (orig_y[i] - y[i])
			+ beta  * (y[i-1] + y[i+1] - 2.0 * y[i]);

		    //vypocitej zmenu, manhattan distance
		    total_change += std::abs(new_x - x[i]) + std::abs(new_y - y[i]);

		    x[i] = new_x;
		    y[i] = new_y;
		}

	//kontrola velikosty zmeny
	if (total_change < tolerance) {
	    RCLCPP_INFO(get_logger(), "Path smoother converged after %d iterations.", iter + 1);
	    break;
	}
	}


    //zapiseme novou cestu do path_
    for (int i = 0; i < N; i++) {
        path_.poses[i].pose.position.x = x[i];
        path_.poses[i].pose.position.y = y[i];
    }

    RCLCPP_INFO(get_logger(), "Path smoothing complete, %d waypoints.", N);
}

Cell::Cell(int c, int r) {
    // add code here
    x = c;
    y = r;
    f = 0;
    g = 0;
    h = 0;
    parent = NULL;
}

