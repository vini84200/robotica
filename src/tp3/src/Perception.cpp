#include "Perception.h"
#include "Utils.h"

#include <queue>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/////////////////////////////////////
/// CONSTRUTOR e FUNCOES PUBLICAS ///
/////////////////////////////////////

Perception::Perception()
{
    started_ = false;
    directionOfNavigation_ = 0.0;
    validDirection_=false;
}

bool Perception::hasValidDirection()
{
    return validDirection_;
}

bool Perception::hasValidMap()
{
    return started_;
}

double Perception::getDirectionOfNavigation()
{
    return directionOfNavigation_;
}

geometry_msgs::msg::PoseStamped& Perception::getDirectionOfNavigationMsg()
{
    return msg_directionOfNavigation_;
}

nav_msgs::msg::OccupancyGrid& Perception::getOccTypesMapMsg()
{
    return msg_occTypes_;
}

nav_msgs::msg::OccupancyGrid& Perception::getPlanTypesMapMsg()
{
    return msg_planTypes_;
}

Pose2D Perception::getLatestPoseFromOdometry()
{
    Pose2D robotPose;

    // Update robotPose from robot transformation
    robotPose.x = odomROS_.pose.pose.position.x;
    robotPose.y = odomROS_.pose.pose.position.y;

    // Convert quaternion to euler angles
    tf2::Quaternion q4(odomROS_.pose.pose.orientation.x,
                       odomROS_.pose.pose.orientation.y, 
                       odomROS_.pose.pose.orientation.z, 
                       odomROS_.pose.pose.orientation.w);
    tf2::Matrix3x3 m4(q4);
    double roll, pitch, yaw;
    m4.getRPY(roll,pitch,yaw);

    // Update orientation with yaw
    robotPose.theta = RAD2DEG(yaw);

    return robotPose;
}

/////////////////////////////////////////////
/// Callbacks dos topicos de MAPA e ODOM  ///
/////////////////////////////////////////////

void Perception::receiveGridmap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &value)
{
    // STRUCTURE OF nav_msgs::msg::OccupancyGrid
    // # This represents a 2-D grid map, in which each cell represents the probability of occupancy.
    // Header header 
    // 
    // # MetaData for the map
    //   # The time at which the map was loaded
    //   time map_load_time
    //   # The map resolution [m/cell]
    //   float32 resolution
    //   # Map width [cells]
    //   uint32 width
    //   # Map height [cells]
    //   uint32 height
    //   # The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.
    //   geometry_msgs/Pose origin
    // MapMetaData info
    //
    // # The map data, in row-major order, starting with (0,0).  
    // # Occupancy probabilities are in the range [0,100].  Unknown is -1.
    // # OBS: implemented in c++ with std::vector<u_int8>
    // int8[] data

    if(started_==false){

        // At the first time, initialize all variables and maps
        numCellsX_ = value->info.width;
        numCellsY_ = value->info.height;

        float cellSize = value->info.resolution;

        mapWidth_ = numCellsX_*cellSize;
        mapHeight_ = numCellsY_*cellSize;
        scale_ = 1.0/cellSize;

        occupancyTypeGrid_.resize(numCellsX_*numCellsY_,OCC_UNEXPLORED);

        planningTypeGrid_.resize(numCellsX_*numCellsY_,PLAN_INVALID);
        fValueGrid_.resize(numCellsX_*numCellsY_,DBL_MAX);
        gValueGrid_.resize(numCellsX_*numCellsY_,DBL_MAX);
        hValueGrid_.resize(numCellsX_*numCellsY_,DBL_MAX);
        parentGrid_.resize(numCellsX_*numCellsY_,-1);

        minKnownX_ = numCellsX_-1;
        minKnownY_ = numCellsY_-1;
        maxKnownX_ = maxKnownY_ = 0;

        started_=true;
    }

    // Copy the occupancy grid map to the occupancyTypeGrid_ (which will be modified next)
    for(unsigned int i=0; i<numCellsX_*numCellsY_; i++)
        occupancyTypeGrid_[i] = value->data[i];

    // Classify cells
    updateGridKnownLimits();
    updateCellsClassification();

    // Get cell in free space closest to the robot position
    // This is required because the robot may be near obstacles, 
    // in regions where the planning is not performed
    Pose2D robotPose = getLatestPoseFromOdometry();
    int robotIndexInFreeSpace = getNearestFreeCell(robotPose);

    // Select center of nearest viable frontier
    int nearestFrontierIndex = clusterFrontiersAndReturnIndexOfClosestOne(robotIndexInFreeSpace);

    // Compute A*
    // first - compute heuristic in all cells (euclidian distance to the goal)
    precomputeHeuristic();
    // second - compute the A* algorithm
    int goal = computeShortestPathToFrontier(robotIndexInFreeSpace);

    // Printing the index of the goal cell, must be the same as 'nearestFrontierIndex'
    std::cout << "goal " << goal << std::endl; //

    if (goal == -1) {
        validDirection_ = false;
    } else {
        validDirection_ = true;
    }

    // Mark path cells for vizualization
    markPathCells(goal);

    // Compute direction of navigation based on the path
    double yaw = computeDirectionOfNavigation(robotIndexInFreeSpace, goal);
    directionOfNavigation_ = normalizeAngleDEG(RAD2DEG(yaw)-robotPose.theta);

    // Update and publish direction of navigation
    msg_directionOfNavigation_.header = value->header;
    msg_directionOfNavigation_.pose.position.x = robotPose.x;
    msg_directionOfNavigation_.pose.position.y = robotPose.y;
    msg_directionOfNavigation_.pose.position.z = 0;
    tf2::Quaternion quat_tf;
    quat_tf.setRPY( 0, 0, yaw );
    msg_directionOfNavigation_.pose.orientation=tf2::toMsg(quat_tf);

    // Update messages
    msg_occTypes_.header = value->header;
    msg_occTypes_.info = value->info;
    msg_occTypes_.data = occupancyTypeGrid_;

    msg_planTypes_.header = value->header;
    msg_planTypes_.info = value->info;
    msg_planTypes_.data = planningTypeGrid_;

    started_=true;
}

void Perception::receiveOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr &value)
{
    //  STRUCTURE OF nav_msgs::msg::Odometry

    // This represents an estimate of a position and velocity in free space.  
    // The pose in this message should be specified in the coordinate frame given by header.frame_id.
    // The twist in this message should be specified in the coordinate frame given by the child_frame_id
    
    // Header header
    //     # Standard metadata for higher-level stamped data types.
    //     # This is generally used to communicate timestamped data
    //     # in a particular coordinate frame.
    //     #
    //     # sequence ID: consecutively increasing ID
    //     uint32 seq
    //     #Two-integer timestamp that is expressed as:
    //     # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    //     # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    //     # time-handling sugar is provided by the client library
    //     time stamp
    //     #Frame this data is associated with
    //     # 0: no frame
    //     # 1: global frame
    //     string frame_id
    //              # timestamp in the header is the acquisition time of
    //              # the first ray in the scan.
    //              #
    //              # in frame frame_id, angles are measured around
    //              # the positive Z axis (counterclockwise, if Z is up)
    //              # with zero angle being forward along the x axis
    odomROS_.header = value->header;
    
    odomROS_.child_frame_id = value->child_frame_id;
    
    odomROS_.pose = value->pose;

    odomROS_.twist = value->twist;
}


//////////////////////////////////////////////////////////////////
/// FUNCOES DE PLANEJAMENTO DE CAMINHOS - A SEREM PREENCHIDAS  ///
//////////////////////////////////////////////////////////////////

void Perception::updateCellsClassification()
{
    /// TODO:
    /// varra as celulas dentro dos limites conhecidos do mapa 'occupancyTypeGrid_'
    /// e atualize os valores, marcando como condição de contorno se for o caso

    /// coordenada x das celulas vai de minKnownX_ a maxKnownX_
    /// coordenada y das celulas vai de minKnownY_ a maxKnownY_
    /// compute o indice da celula no grid usando:
    ///    int i = x + y*numCellsX_;
    /// ex: atualizando uma celula como ocupada
    ///    occupancyTypeGrid_[i] = OCC_OCCUPIED;

    /// grid na entrada (valor inteiro): 
    /// - celulas desconhecidas = -1 
    /// - celulas conhecidas com ocupação variando de 0 a 100

    /// grid na saida (valor inteiro):
    /// - celulas desconhecidas                = OCC_UNEXPLORED   (definido como -1)
    /// - celulas de obstaculo                 = OCC_OCCUPIED     (definido como 100)
    /// - celulas livres vizinhas a obstaculos = OCC_NEAROBSTACLE (definido como 90)
    /// - celulas de fronteira                 = OCC_FRONTIER     (definido como 30)
    /// - demais celulas livres                = OCC_FREE         (definido como 50)

    // Dica
    // 1) Marcar obstaculos
    // 2) Marcar celulas vizinhas a obstaculos considerando 'dangerZoneWidth' celulas
    int dangerZoneWidth = 5;
    // 3) Marcar fronteiras (ignorando OCC_OCCUPIED e OCC_NEAROBSTACLE)
    // 4) Marcar restantes, que nao sao inexploradas, como livre

    std::vector<int8_t> occupancyGridValues = std::vector<int8_t>(occupancyTypeGrid_.begin(), occupancyTypeGrid_.end());

    int FREE_THRESHOLD = 50;
    int OCCUPIED_THRESHOLD = 50;

    for (int x = minKnownX_; x <= maxKnownX_; x++)
    {
        for (int y = minKnownY_; y <= maxKnownY_; y++)
        {
            int i = x + y * numCellsX_;
            if (occupancyGridValues[i] == OCC_UNEXPLORED)
            {
                occupancyTypeGrid_[i] = OCC_UNEXPLORED;
            }
            else if (occupancyGridValues[i] >= OCCUPIED_THRESHOLD)
            {
                occupancyTypeGrid_[i] = OCC_OCCUPIED;
            }
            else if (occupancyGridValues[i] <= FREE_THRESHOLD)
            {
                // Check if its near an obstacle
                bool nearObstacle = false;
                for (int dx = -dangerZoneWidth; dx <= dangerZoneWidth; dx++)
                {
                    for (int dy = -dangerZoneWidth; dy <= dangerZoneWidth; dy++)
                    {
                        if (dx == 0 && dy == 0)
                            continue;

                        int nx = x + dx;
                        int ny = y + dy;

                        if (nx >= minKnownX_ && nx <= maxKnownX_ && ny >= minKnownY_ && ny <= maxKnownY_)
                        {
                            int ni = nx + ny * numCellsX_;
                            if (occupancyGridValues[ni] >= OCCUPIED_THRESHOLD)
                            {
                                nearObstacle = true;
                                break;
                            }
                        }
                    }
                    if (nearObstacle)
                        break;
                }
                if (nearObstacle)
                {
                    occupancyTypeGrid_[i] = OCC_NEAROBSTACLE;
                }
                else
                {
                    // Check if its a frontier cell, i.e., a free cell adjacent to an unexplored cell
                    bool isFrontier = false;
                    for (int dx = -1; dx <= 1; dx++) 
                    {
                        for (int dy = -1; dy <= 1; dy++) 
                        {
                            if (dx == 0 && dy == 0)
                                continue;
                            int nx = x + dx;
                            int ny = y + dy;

                            if (nx >= minKnownX_ && nx <= maxKnownX_ && ny >= minKnownY_ && ny <= maxKnownY_)
                            {
                                int ni = nx + ny * numCellsX_;
                                if (occupancyGridValues[ni] == OCC_UNEXPLORED)
                                {
                                    isFrontier = true;
                                    break;
                                }
                            }
                            else 
                            {
                                // If the neighbor is out of bounds, consider it as unexplored
                                isFrontier = true;
                                break;
                            }

                        }
                    }

                    if (isFrontier)
                    {
                        occupancyTypeGrid_[i] = OCC_FRONTIER;
                    }
                    else
                    {
                        occupancyTypeGrid_[i] = OCC_FREE;
                    }
                }
            }
            
        }
    }

}

void Perception::precomputeHeuristic()
{

    /// TODO:
    /// varra as celulas dentro dos limites conhecidos do mapa 'planningTypeGrid_'
    /// e atualize os valores das medidas f, g, h e pi das celulas validas
    
    /// coordenada x das celulas vai de minKnownX_ a maxKnownX_
    /// coordenada y das celulas vai de minKnownY_ a maxKnownY_
    /// compute o indice da celula no grid usando:
    ///    int i = x + y*numCellsX_;

    /// Dica: uma celula 'i' eh valida se (planningTypeGrid_[i] != PLAN_INVALID)

    /// A atualizacao deve seguir as seguintes regras:
    ///   fValueGrid_[i] e gValueGrid_[i] - valores f e g: recebem DBL_MAX (equivalente ao infinito)
    ///   parentGrid_[i] - valor pi (indicando o pai da celula): recebe -1, pois a priori nao aponta para ninguem. 
    ///   hValueGrid_[i] - valor h - distancia para o goal

    for (int x = minKnownX_; x <= maxKnownX_; x++)
    {
        for (int y = minKnownY_; y <= maxKnownY_; y++)
        {
            int i = x + y * numCellsX_;
            gValueGrid_[i] = DBL_MAX;
            fValueGrid_[i] = DBL_MAX;
            parentGrid_[i] = -1;
            
            float minDistance = FLT_MAX;
            for (auto indices : frontierCentersIndices)
            {
                int fx = indices % numCellsX_;
                int fy = indices / numCellsX_;

                float distance = sqrt(pow(fx - x, 2.0) + pow(fy - y, 2.0));
                if (distance < minDistance)
                {
                    minDistance = distance;
                }
            }
            hValueGrid_[i] = minDistance;
        }
    }

}

// offset para os 8 vizinhos
// uso, i-esimo vizinho (nx,ny) da posicao (x,y):
//      int nx = x+offset[i][0];
//      int ny = y+offset[i][1];
int offset[8][2] = {{-1,  1}, { 0,  1}, { 1,  1}, { 1,  0}, { 1, -1}, { 0, -1}, {-1, -1}, {-1,  0}};

// custo de distancia para os 8 vizinhos
// uso, atualizando custo do i-esimo vizinho
//      int id_celula  =  x +  y*numCellsX_;
//      int id_vizinho = nx + ny*numCellsX_;
//      gValueGrid_[id_vizinho] = gValueGrid_[id_celula] + cost[i];
double cost[8] = {sqrt(2), 1, sqrt(2), 1, sqrt(2), 1, sqrt(2), 1};

int Perception::computeShortestPathToFrontier(int robotCellIndex)
{
    int rx = robotCellIndex % numCellsX_;
    int ry = robotCellIndex / numCellsX_;

    /// TODO:
    /// Computar o algoritmo A Star usando os valores em hValueGrid_ 
    /// e atualizando os valores em fValueGrid_, gValueGrid_ e parentGrid_

    /// Ao fim deve retornar o indice da celula de goal, encontrada na busca. Ou -1 se nao encontrar
    int goal = -1;

    /// Sugestao: usar a fila de prioridades abaixo
    /// onde o primeiro elemento do par eh o f-value e o segundo elemento eh o indice da celula
    std::priority_queue< std::pair<double, int> , std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>> > pq;

    /// Exemplo: insercao na fila
    //      std::pair<double, int> vizinho;
    //      vizinho.first = fValueGrid_[id_vizinho];
    //      vizinho.second = id_vizinho;
    //      pq.push(vizinho);

    /// Exemplo: remocao da fila
    //      std::pair<double, int> celula = pq.top();
    //      pq.pop();
    //      int id_celula = celula.second;

    /// O algoritmo comeca da posicao do robo
    gValueGrid_[robotCellIndex] = 0;

    std::pair<double, int> inicio;
    inicio.first = hValueGrid_[robotCellIndex];
    inicio.second = robotCellIndex;

    pq.push(inicio);

    /// Completar algoritmo A Star, consultando a fila enquanto ela nao estiver vazia
    while(!pq.empty()) {
        auto node = pq.top();
        pq.pop();
        int id = node.second;
        double gScore = gValueGrid_[id];
        int x = id % numCellsX_;
        int y = id / numCellsX_;

        if (this->isGoal(id)) {
            goal = id;
            return id;
        }

        // Expand neighbours
        for (int i = 0; i < 8; i++)
        {
            auto os = offset[i];

            int dx = os[0];
            int dy = os[1];
            int nx = x + dx;
            int ny = y + dy;
            // Check if is valid
            if (
                nx < minKnownX_ || nx > maxKnownX_ || ny < minKnownY_ || ny > maxKnownY_)
                continue;
            int ni = nx + ny * numCellsX_;
            if (
                occupancyTypeGrid_[ni] == OCC_OCCUPIED || occupancyTypeGrid_[ni] == OCC_UNEXPLORED)
            {
                continue;
            }
            double c = cost[i];
            if (
                occupancyTypeGrid_[ni] == OCC_NEAROBSTACLE)
            {
                c += 20.;
            }

            double tentative_gscore = c + gScore;
            if (gValueGrid_[ni] > tentative_gscore)
            {
                parentGrid_[ni] = id;
                gValueGrid_[ni] = tentative_gscore;
                double f = tentative_gscore + hValueGrid_[ni];
                fValueGrid_[ni] = f;
                std::pair<double, int> vizinho;
                vizinho.first = f;
                vizinho.second = ni;
                pq.push(vizinho);
            }
        }
    }

    std::cout << "No goal found" << std::endl;

    return goal;
}

bool Perception::isGoal(int cellIndex) {
    for (auto indices : frontierCentersIndices) {
        if (indices == cellIndex) { 
            return true;
        }
    }
    return false;
}

////////////////////////////////////////////////////////
/// FUNCOES AUXILIARES PARA PLANEJAMENTO DE CAMINHOS ///
////////////////////////////////////////////////////////

// Given the explored area, update the following variables: minKnownX_, maxKnownX_, minKnownY_, maxKnownY_
void Perception::updateGridKnownLimits()
{
    for(int x=0; x<numCellsX_; x++){
        for(int y=0; y<numCellsY_; y++){
            int i = x + y*numCellsX_;
            if(occupancyTypeGrid_[i]!=-1)
            {
                if(x<minKnownX_) minKnownX_ = x;
                if(y<minKnownY_) minKnownY_ = y;
                if(x>maxKnownX_) maxKnownX_ = x;
                if(y>maxKnownY_) maxKnownY_ = y;
            }
        }
    }
}

// Groups all frontier cells in clusters, and keep the ones with size greater than 'minFrontierSize' 
// Then selects the centers of each of the frontiers
// Next, choose the one closest to the robot's position as the goal
// Returns the index of the cell associated with the center of the selected frontier
int Perception::clusterFrontiersAndReturnIndexOfClosestOne(int robotCellIndex)
{
    frontierCentersIndices.clear();

    int width=1;
    int minFrontierSize = 3;

    // Check occupancyTypeGrid_ and set PLAN_GOALS in planningTypeGrid_
    for(int x=minKnownX_; x<=maxKnownX_; x++){
        for(int y=minKnownY_; y<=maxKnownY_; y++){
            int i = x + y*numCellsX_;

            if(occupancyTypeGrid_[i] == OCC_FRONTIER) 
                planningTypeGrid_[i] = PLAN_GOALS;      // Frontier cells are goals
            else if(occupancyTypeGrid_[i] == OCC_FREE) 
                planningTypeGrid_[i] = PLAN_REGULAR;    // Free cells are regular cells (where path can be computed)
            else
                planningTypeGrid_[i] = PLAN_INVALID;    // Remaining cells are invalid for planning
        }
    }

    // Group all neighboring goal cells
    for(int x=minKnownX_; x<=maxKnownX_; x++){
        for(int y=minKnownY_; y<=maxKnownY_; y++){
            int i = x + y*numCellsX_;

            // detect a goal cell that is not MARKED yet
            if(planningTypeGrid_[i] == PLAN_GOALS){
                planningTypeGrid_[i] = PLAN_MARKEDGOALS;

                std::vector<unsigned int> frontier;

                float centerx = 0, centery = 0;
                float count = 0;

                // mark all neighbor goal cells
                // breadth-first search using a queue
                std::queue<int> q;
                q.push(i);
                while(!q.empty())
                {
                    int c = q.front();
                    q.pop();
                    frontier.push_back(c);

                    int cx = c % numCellsX_;
                    int cy = c / numCellsX_;
                    centerx += cx;
                    centery += cy;
                    count++;

                    for(int nx=cx-width;nx<=cx+width;nx++){
                        for(int ny=cy-width;ny<=cy+width;ny++){
                            int ni = nx + ny*numCellsX_;
                            if(planningTypeGrid_[ni] == PLAN_GOALS){
                                planningTypeGrid_[ni] = PLAN_MARKEDGOALS;
                                q.push(ni);
                            }
                        }
                    }
                }

                // keep frontiers that are larger than minFrontierSize
                if(count > minFrontierSize){
                    centerx /= count;
                    centery /= count;
 
                    // find cell closest to frontier center
                    float minDist=FLT_MAX;
                    int closest=-1;

                    for(unsigned int k=0;k<frontier.size();k++){
                        int fx = frontier[k] % numCellsX_;
                        int fy = frontier[k] / numCellsX_;

                        float dist = sqrt(pow(fx-centerx,2.0)+pow(fy-centery,2.0));
                        if(dist < minDist){
                            minDist = dist;
                            closest = frontier[k];
                        }
                    }

                    // add center of frontier to list of Goals
                    frontierCentersIndices.push_back(closest);

                }else{

                    // ignore small frontiers
                    for(unsigned int k=0;k<frontier.size();k++){
                        planningTypeGrid_[frontier[k]] = PLAN_REGULAR;
                    }
                }
            }
        }
    }

    // These are the filtered frontiers (that are not too small)
    std::cout << "Number of frontiers: " << frontierCentersIndices.size() << std::endl;
    for(unsigned int k=0;k<frontierCentersIndices.size();k++){
        planningTypeGrid_[frontierCentersIndices[k]] = PLAN_GOALS;
    }

    if(frontierCentersIndices.empty())
        return -1;
    else{

        // // Select nearest frontier among the filtered frontiers
        // int nearestFrontierIndex=-1;
        // float distance = DBL_MAX;

        // int rx = robotCellIndex % numCellsX_;
        // int ry = robotCellIndex / numCellsX_;
        // for(int k=0;k<frontierCentersIndices.size();k++){
        //     int nFx = frontierCentersIndices[k] % numCellsX_;
        //     int nFy = frontierCentersIndices[k] / numCellsX_;
        //     float d = sqrt(pow(rx-nFx,2.0)+pow(ry-nFy,2.0));
        //     if(d < distance)
        //     {
        //         distance = d;
        //         nearestFrontierIndex = frontierCentersIndices[k];
        //     }
        // }

        // // Clear frontiers that were not selected
        // for(int k=0;k<frontierCentersIndices.size();k++){
        //     if(frontierCentersIndices[k] != nearestFrontierIndex)
        //         planningTypeGrid_[frontierCentersIndices[k]] = PLAN_MARKEDGOALS;
        // }

        // return nearestFrontierIndex;
        return -1;
    }
}

// Mark all path cells in the 'planningTypeGrid_' after the A* Star algorithm is computed
// by checking the index of the parent of each cell starting from the goal
void Perception::markPathCells(int goal)
{
    if(goal != -1){

        int c = parentGrid_[goal];

        while(c != -1){
            planningTypeGrid_[c] = PLAN_PATH;
            c = parentGrid_[c];
        }
    }
}

// Select a path cell in a distance given by 'localGoalRadius' from the robot
// and compute the angle difference from the robot orientation to this cell
double Perception::computeDirectionOfNavigation(int robotCellIndex, int goalIndex)
{
    int rx = robotCellIndex % numCellsX_;
    int ry = robotCellIndex / numCellsX_;

    int c = goalIndex;

    double localGoalRadius = 3;

    int cx, cy;

    while(parentGrid_[c] != -1){
        cx = c % numCellsX_;
        cy = c / numCellsX_;

        double dist = sqrt(pow(cx-rx,2.0)+pow(cy-ry,2.0));
        
        if(dist < localGoalRadius){
            break;
        }
        c = parentGrid_[c];
    }

    double yaw = atan2(cy-ry,cx-rx);

    return yaw;
}

// Return index of the free cell closest to the robot position
int Perception::getNearestFreeCell(Pose2D robot)
{
    int rx = robot.x*scale_ + numCellsX_/2;
    int ry = robot.y*scale_ + numCellsY_/2; 

    int u;

    for(int l=1; l<20; l++){

        for(int cx=rx-l; cx<=rx+l; cx++){
            u = cx + (ry+l)*numCellsX_;
            if(occupancyTypeGrid_[u] == OCC_FREE)
                return u;
            u = cx + (ry-l)*numCellsX_;
            if(occupancyTypeGrid_[u] == OCC_FREE)
                return u;
        }

        for(int cy=ry-l; cy<=ry+l; cy++){
            u = rx+l + cy*numCellsX_;
            if(occupancyTypeGrid_[u] == OCC_FREE)
                return u;
            u = rx-l + cy*numCellsX_;
            if(occupancyTypeGrid_[u] == OCC_FREE)
                return u;
        }

    }

    return -1;
}
