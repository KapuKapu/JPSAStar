/*----------------------------------------------------------------------------#
#    Copyright 2013 Julian Weitz                                              #
#                                                                             #
#    This program is free software: you can redistribute it and/or modify     #
#    it under the terms of the GNU General Public License as published by     #
#    the Free Software Foundation, either version 3 of the License, or        #
#    any later version.                                                       #
#                                                                             #
#    This program is distributed in the hope that it will be useful,          #
#    but WITHOUT ANY WARRANTY; without even the implied warranty of           #
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            #
#    GNU General Public License for more details.                             #
#                                                                             #
#    You should have received a copy of the GNU General Public License        #
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.    #
# ---------------------------------------------------------------------------*/

#include "JPSAStar.hpp"
using namespace jpsastar;


/**
 *  Generates list of waypoints using node.parent
 *
 *  Target node is included in the resulting list.
 *
 *  \param target Last waypoint node
 *
 *  \return       List of waypoints from start node to target node
**/
std::list<cv::Vec2i> JPSAStar::buildPath(const Node &target) const{
    std::list<cv::Vec2i> path;
    path.push_front(target.vector);
    Node *parent = target.parent;
    while(parent != NULL){
        path.push_front(parent->vector);
        parent = parent->parent;
        }
    return path;
    }


/**
 *  Returns 8-connected unoccupied pixel coordintaes to a given node
 *
 *  \param current Center node
 *
 *  \return        List of 8-connected unoccupied neighbors
**/
std::list<cv::Vec2i> JPSAStar::connected(Node &current) const{
    std::list<cv::Vec2i> neighbors;
    bool x_p_one = false;
    bool x_m_one = false;
    std::vector<int> connected_idxs;
    // Range and occupancy checks
    if(current.vector[0] + 1 < this->map_.cols){
        x_p_one = true;
        if(0 < this->map_.at<uchar>(current.vector[1], current.vector[0] + 1))
            neighbors.push_back( cv::Vec2i(current.vector[0] + 1, current.vector[1]) );
        }
    if(0 <= current.vector[0] - 1){
        x_m_one = true;
        if(0 < this->map_.at<uchar>(current.vector[1], current.vector[0] - 1))
            neighbors.push_back( cv::Vec2i(current.vector[0] - 1, current.vector[1]) );
        }
    if(current.vector[1] + 1 < this->map_.rows){
        if( 0 < this->map_.at<uchar>(current.vector[1] + 1, current.vector[0]) )
            neighbors.push_back( cv::Vec2i(current.vector[0], current.vector[1] + 1) );
        if( x_p_one && 0 < this->map_.at<uchar>(current.vector[1] + 1, current.vector[0] + 1) )
            neighbors.push_back( cv::Vec2i(current.vector[0] + 1, current.vector[1] + 1) );
        if( x_m_one && 0 < this->map_.at<uchar>(current.vector[1] + 1, current.vector[0] - 1) )
            neighbors.push_back( cv::Vec2i(current.vector[0] - 1, current.vector[1] + 1) );
        }
    if(0 <= current.vector[1] - 1){
        if( 0 < this->map_.at<uchar>(current.vector[1] - 1, current.vector[0]) )
            neighbors.push_back( cv::Vec2i(current.vector[0], current.vector[1] - 1) );
        if( x_p_one && 0 < this->map_.at<uchar>(current.vector[1] - 1, current.vector[0] + 1) )
            neighbors.push_back( cv::Vec2i(current.vector[0] + 1, current.vector[1] - 1) );
        if( x_m_one && 0 < this->map_.at<uchar>(current.vector[1] - 1, current.vector[0] - 1) )
            neighbors.push_back( cv::Vec2i(current.vector[0] - 1, current.vector[1] - 1) );
        }
    return neighbors;
    }


/**
 *  Computes forced neighbors of diagonally expanded node
 *
 *  \param current   Parent of computed neighbors
 *  \param direction Vector pointing in diagonal direction: (1,1), (-1,1), (-1,-1) or (1,-1)
 *
 *  \return          Forced neighbors of current
**/
std::list<cv::Vec2i> JPSAStar::diagonalForced(const cv::Vec2i &current, const cv::Vec2i &direction) const{
    std::list<cv::Vec2i> forced;
    int x_forced = current[0] + direction[0];
    int y_forced = current[1] + direction[1];
    if(   0 <= x_forced && x_forced < this->map_.cols
       && this->map_.at<uchar>(current[1] - direction[1], current[0]) == 0
       && 0 < this->map_.at<uchar>(current[1] - direction[1], x_forced) ){
        forced.push_back( cv::Vec2i(x_forced, current[1] - direction[1]) );
        }
    if(   0 <= y_forced && y_forced < this->map_.rows
       && this->map_.at<uchar>(current[1], current[0] - direction[0]) == 0
       && 0 < this->map_.at<uchar>(y_forced, current[0] - direction[0]) ){
        forced.push_back( cv::Vec2i(current[0] - direction[0], y_forced) );
        }
    return forced;
    }


/**
 *  Computes diagonal jump point in direction of given direction
 *
 *  \param current   Origin of computed jump point
 *  \param target    Target coordinates, because the target is a special jump point
 *  \param direction Vector pointing in diagonal direction: (1,1), (-1,1), (-1,-1) or (1,-1)
 *
 *  \return          Jump point of current
**/
cv::Vec2i* JPSAStar::diagonalJPS(cv::Vec2i current, const cv::Vec2i &target, const cv::Vec2i &direction) const{
    cv::Vec2i *jps_vec;
    // While in range and not occupied
    while(   0 < current[0] && current[0] < this->map_.cols
          && 0 < current[1] && current[1] < this->map_.rows
          && 0 < this->map_.at<uchar>(current[1], current[0])){
        // Check if target reached
        if(current[0] == target[0] && current[1] == target[1])
            return new cv::Vec2i(current);
        // Check for diagonal forced neighbors
        if( !this->diagonalForced(current, direction).empty() )
            return new cv::Vec2i(current);
        // Check for straight x and y jump points
        jps_vec = this->straightJPS(current, target, cv::Vec2i(direction[0], 0));
        if(jps_vec){
            delete jps_vec;
            return new cv::Vec2i(current);
            }
        jps_vec = this->straightJPS(current, target, cv::Vec2i(0, direction[1]));
        if(jps_vec){
            delete jps_vec;
            return new cv::Vec2i(current);
            }
        current += direction;
        }
    return NULL;
    }


/**
 *  Gernerates path from start to target using a grid map
 *
 *  This function uses the jump point search A* algorithm described in
 *  "Online Graph Pruning for Pathfinding On Grid Maps" by Daniel
 *  Harabor and Alban Grastien.
 *  The grid map has to be an 8 bit grey scale image. Values below 255
 *  are considered to be occupied. And cells/pixels with a value of
 *  255 is considered to be free space and therefore usable for
 *  navigation.
 *
 *  \param start  (x,y) of the start point in this->map_ coordinates
 *  \param target (x,y) of the target point in this->map_ coordinates
 *
 *  \return       Waypoints from start (excluded) to target (included). Empty if no path was found.
 *
 *  \throws       NotOnMap is thrown if start or target isn't on the map.
**/
std::list<cv::Vec2i> JPSAStar::findPath(cv::Vec2i start, cv::Vec2i target) const{
    // Throw exception if start or target is out of map range
    if(   start[0] < 0 || this->map_.size().width < start[0]
       || start[1] < 0 || this->map_.size().height < start[1] )
        throw NotOnMap( std::string("[JPSAStar] Start vector (")
                      + std::to_string(start[0]) + "," + std::to_string(start[1])
                      + ") out of map range ("
                      + std::to_string(this->map_.size().width) + "," + std::to_string(this->map_.size().height)
                      + ")" );
    if(   target[0] < 0 || this->map_.size().width < target[0]
       || target[1] < 0 || this->map_.size().height < target[1] )
        throw NotOnMap( std::string("[JPSAStar] Target vector (")
                      + std::to_string(target[0]) + "," + std::to_string(target[1])
                      + ") out of map range ("
                      + std::to_string(this->map_.size().width) + "," + std::to_string(this->map_.size().height)
                      + ")" );

    std::list<cv::Vec2i> ret;
    MinHeap open_queue;
    VecSet closed_set;

    Node *current = new Node(start, NULL, 0.0, this->distance(start, target));
    open_queue.insert( std::pair<float, Node*>(current->g_value, current) );
    while(!open_queue.empty()){
        current = open_queue.begin()->second;
        open_queue.erase( open_queue.begin() );
        // Check if target was reached
        if( current->vector[0] == target[0] && current->vector[1] == target[1] ){
            ret = this->buildPath(*current);
            break;
            }
        closed_set.insert(current);

        // Get successors via pruning and jump point search
        cv::Vec2i *jump_point;
        MinHeap::iterator in_open;
        VecSet::iterator in_closed;
        std::list<cv::Vec2i> pruned = this->prunedNeighbors(*current);
        std::list<cv::Vec2i>::iterator it, end;
        Node *jp_node;
        for(it=pruned.begin(),end=pruned.end(); it != end ;++it){
            // Do Jump Point Search for neighbor
            jump_point = this->jumpPoint(current->vector, *it, target);
            if(jump_point != NULL){
                jp_node = new Node(*jump_point, NULL);
                // Do regular A* stuff for neighbors
                float g_neighbor = current->g_value + this->distance(current->vector, *jump_point);
                in_closed = closed_set.find(jp_node);
                // Search vor jump point in closed_set
                in_open = open_queue.end();
                // Search vor jump point in open_queue
                for(MinHeap::iterator ith=open_queue.begin(); ith != open_queue.end() ;++ith){
                    if(ith->second->vector[0] == jp_node->vector[0] && ith->second->vector[1] == jp_node->vector[1]){
                        in_open = ith;
                        break;
                        }
                    }
                if( in_closed != closed_set.end() && (*in_closed)->g_value <= g_neighbor){
                    if( (*in_closed)->g_value <= g_neighbor )
                        continue;
                    else{
                        delete *in_closed;
                        closed_set.erase(in_closed);
                        }
                    }
                if( in_open != open_queue.end() && g_neighbor < in_open->second->g_value){
                    delete in_open->second;
                    open_queue.erase(in_open);
                    in_open = open_queue.end();
                    }
                if( in_open == open_queue.end() ){
                    jp_node->g_value = g_neighbor;
                    jp_node->f_value = g_neighbor + this->distance(jp_node->vector, target);
                    jp_node->parent = current;
                    open_queue.insert( std::pair<float, Node*>(jp_node->f_value, jp_node) );
                    }
                delete jump_point;
                }
            }
        }
    // Clean up
    MinHeap::iterator mh_it = open_queue.begin(), mh_end = open_queue.end();
    for(; mh_it != mh_end;++mh_it){
        delete mh_it->second;
        }
    VecSet::iterator vs_it = closed_set.begin(), vs_end = closed_set.end();
    for(; vs_it != vs_end ;++vs_it){
        delete *vs_it;
        }
    // No path found
    return ret;
    }


/**
 *  Constructor
 *
 *  The grid map has to be an 8 bit grey scale image. Values below 255
 *  are considered to be occupied. And a cells/pixels with a value of
 *  255 is considered to be free space and therefore usable for
 *  navigation.
 *
 *  \param map 8 bit grey scale image
**/
JPSAStar::JPSAStar(cv::Mat map) : map_(map){
    }

/**
 *  Computes jump point of given coodinates
 *
 *  \param parent  Parent of current
 *  \param current Origin of computed jump point
 *  \param target  Target coordinates, because the target is a special jump point
 *
 *  \return        Jump point of current
**/
cv::Vec2i* JPSAStar::jumpPoint(const cv::Vec2i &parent, const cv::Vec2i &current, const cv::Vec2i &target) const{
    cv::Vec2i direction = current - parent;
    if(direction[0] != 0) direction[0] = direction[0] / abs(direction[0]);
    if(direction[1] != 0) direction[1] = direction[1] / abs(direction[1]);

    if(direction[0] != 0 && direction[1] != 0)
        return this->diagonalJPS(current, target, direction);
    else
        return this->straightJPS(current, target, direction);
    }


/**
 *  Returns a clone of the map
 *
 *  \return Clone of internally used map
**/
cv::Mat JPSAStar::map() const{
    return this->map_.clone();
    }


/**
 *  Prunes 8-connected neighbors according to the direction of expansion of a give node
 *
 *  \param current Center node for which neighbors will be generated
 *
 *  \return        Pruned neighbors of current
**/
std::list<cv::Vec2i> JPSAStar::prunedNeighbors(Node &current) const{
    // Check for start node
    if(current.parent == NULL){
        return this->connected(current);
        }
    std::list<cv::Vec2i> pruned;
    cv::Vec2i diff_vec = current.vector - current.parent->vector;
    if(diff_vec[0] != 0) diff_vec[0] = diff_vec[0] / abs(diff_vec[0]);
    if(diff_vec[1] != 0) diff_vec[1] = diff_vec[1] / abs(diff_vec[1]);
    int x_nat = current.vector[0] + diff_vec[0];
    int y_nat = current.vector[1] + diff_vec[1];
    // Diagonal prune case
    if(diff_vec[0] != 0 && diff_vec[1] != 0){
        // Natural neighbors
        if( 0 <= x_nat && x_nat < this->map_.cols && 0 < this->map_.at<uchar>(current.vector[1], x_nat) ){
            pruned.push_back( cv::Vec2i(x_nat, current.vector[1]) );
            }
        if( 0 <= y_nat && y_nat < this->map_.rows && 0 < this->map_.at<uchar>(y_nat, current.vector[0]) ){
            pruned.push_back( cv::Vec2i(current.vector[0], y_nat) );
            }
        if(   0 <= x_nat && x_nat < this->map_.cols
           && 0 <= y_nat && y_nat < this->map_.rows
           && 0 < this->map_.at<uchar>(y_nat, x_nat) ){
            pruned.push_back( cv::Vec2i(x_nat, y_nat) );
            }
        // Forced neighbors
        pruned.splice( pruned.end(), this->diagonalForced(current.vector, diff_vec) );
        }
    // Straight x prune case
    else if(diff_vec[0] != 0){
        // Natural neighbor
        if( 0 <= x_nat && x_nat < this->map_.cols && 0 < this->map_.at<uchar>(current.vector[1], x_nat) ){
            pruned.push_back( cv::Vec2i(x_nat, current.vector[1]) );
            }
        // Forced neighbors
        pruned.splice( pruned.end(), this->straightForced(current.vector, diff_vec) );
        }
    // Straight y prune case
    else{
        // Natural neighbor
        if( 0 <= y_nat && y_nat < this->map_.rows && 0 < this->map_.at<uchar>(y_nat, current.vector[0]) ){
            pruned.push_back( cv::Vec2i(current.vector[0], y_nat) );
            }
        // Forced neighbors
        pruned.splice( pruned.end(), this->straightForced(current.vector, diff_vec) );
        }
    return pruned;
    }


/**
 *  Sets map used for path planning
 *
 *  \param new_map Map used for path planning. Underlying cv::Mat data will not be dublicated.
**/
void JPSAStar::setMap(cv::Mat new_map){
    this->map_ = new_map;
    }


/**
 *  Computes forced neighbors of straight expanded node
 *
 *  \param current   Parent of computed neighbors
 *  \param direction Vector pointing in straight direction: (1,0), (0,1), (-1,0) or (0,-1)
 *
 *  \return          Forced neighbors of current
**/
std::list<cv::Vec2i> JPSAStar::straightForced(const cv::Vec2i &current, const cv::Vec2i &direction) const{
    std::list<cv::Vec2i> forced;
    int x_forced, y_forced;
    // Straight x forced search
    if(direction[0] != 0){
        x_forced = current[0] + direction[0];
        if(0 <= x_forced && x_forced < this->map_.cols){
            y_forced = current[1] - 1;
            if(   -1 < y_forced
               && this->map_.at<uchar>(y_forced, current[0]) == 0
               && 0 < this->map_.at<uchar>(y_forced, x_forced) ){
                forced.push_back( cv::Vec2i(x_forced, y_forced) );
                }
            y_forced = current[1] + 1;
            if(   y_forced < this->map_.rows
               && this->map_.at<uchar>(y_forced, current[0]) == 0
               && 0 < this->map_.at<uchar>(y_forced, x_forced) ){
                forced.push_back( cv::Vec2i(x_forced, y_forced) );
                }
            }
        }
    // Straight y forced search
    else{
        y_forced = current[1] + direction[1];
        if(0 <= y_forced && y_forced < this->map_.rows){
            x_forced = current[0] - 1;
            if(   -1 < x_forced
               && this->map_.at<uchar>(current[1], x_forced) == 0
               && 0 < this->map_.at<uchar>(y_forced, x_forced) ){
                forced.push_back( cv::Vec2i(x_forced, y_forced) );
                }
            x_forced = current[0] + 1;
            if(   x_forced < this->map_.cols
               && this->map_.at<uchar>(current[1], x_forced) == 0
               && 0 < this->map_.at<uchar>(y_forced, x_forced) ){
                forced.push_back( cv::Vec2i(x_forced, y_forced) );
                }
            }
        }
    return forced;
    }


/**
 *  Computes straight jump point in direction of given direction
 *
 *  \param current   Origin of computed jump point
 *  \param target    Target coordinates, because the target is a special jump point
 *  \param direction Vector pointing in straight direction: (1,0), (0,1), (-1,0) or (0,-1)
 *
 *  \return          Jump point of current
**/
cv::Vec2i* JPSAStar::straightJPS(cv::Vec2i current, const cv::Vec2i &target, const cv::Vec2i &direction) const{
    if(direction[0] != 0){
        // While in range and not occupied
        while( 0 < current[0] && current[0] < this->map_.cols && 0 < this->map_.at<uchar>(current[1], current[0]) ){
            // Check if target reached
            if(current[0] == target[0] && current[1] == target[1])
                return new cv::Vec2i(current);
            // Check for straight forced neighbors
            if( !this->straightForced(current, direction).empty() )
                return new cv::Vec2i(current);
            current[0] += direction[0];
            }
        }
    else{
        // While in range and not occupied
        while( 0 < current[1] && current[1] < this->map_.rows && 0 < this->map_.at<uchar>(current[1], current[0]) ){
            // Check if target reached
            if(current[0] == target[0] && current[1] == target[1])
                return new cv::Vec2i(current);
            // Check for straight forced neighbors
            if( !this->straightForced(current, direction).empty() )
                return new cv::Vec2i(current);
            current[1] += direction[1];
            }
        }
    return NULL;
    }
