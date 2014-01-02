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

#ifndef JPSASTAR_HPP_NBO2KO09
#define JPSASTAR_HPP_NBO2KO09

#include <cmath>
#include <stdexcept>
#include <functional>
#include <set>
#include <list>
#include <map>
#include <utility>
#include <algorithm>
#include <opencv2/opencv.hpp>

namespace jpsastar{
    /** \mainpage JPSAStar-Project Documentation
     *  \section sec_over Overview
     *      There was no simple to use implementation of jump point search A*
     *      for c++ and OpenCV available, so i did it myself. This
     *      implementation is based on the paper "Online Graph Pruning for
     *      Pathfinding On Grid Maps" written by Daniel Harabor and Alban
     *      Grastien.
     *
     *  \section sec_useage Usage
     *      Copy and integrate the files JPSAStar.cpp and JPSAStar.hpp into
     *      your build setup and you are ready to go.
    **/

    /**
     *  A* node containing parent, g value, f value and pixel position
    **/
    struct Node{
        Node(cv::Vec2i vec, Node *parent, float g=0.0, float f=0.0)
             : vector(vec),
               g_value(g),
               f_value(f),
               parent(parent){};
        bool operator<(const Node &rhs) const{ return this->g_value < rhs.g_value; };
        bool operator>(const Node &rhs) const{ return this->g_value > rhs.g_value; };
        bool operator==(const cv::Vec2i &rhs) const{
            return this->vector[0] == rhs[0] && this->vector[1] == rhs[1];
            };

        float g_value;    ///< G score representing the cost from the start point to the Node
        float f_value;    ///< F score representing the heuristic enhanced costs from start to target
        Node *parent;     ///< Node from which this Node can be reached
        cv::Vec2i vector; ///< Image position of the pixel this Node is representing
        };


    /**
     *  Interface class that uses jump point search A* for path finding
     *
     *  This class uses the jump point search A* algorithm described in
     *  "Online Graph Pruning for Pathfinding On Grid Maps" by Daniel
     *  Harabor and Alban Grastien.
    **/
    class JPSAStar{
        public:
        JPSAStar(cv::Mat map);
        std::list<cv::Vec2i> findPath(cv::Vec2i start, cv::Vec2i target) const;
        cv::Mat map() const;
        void setMap(cv::Mat new_map);

        private:
        std::list<cv::Vec2i> buildPath(const Node &target) const;
        std::list<cv::Vec2i> connected(Node &current) const;
        std::list<cv::Vec2i> diagonalForced(const cv::Vec2i &current,
                                            const cv::Vec2i &direction) const;
        cv::Vec2i* diagonalJPS(cv::Vec2i current,
                               const cv::Vec2i &target,
                               const cv::Vec2i &direction) const;
        float distance(const cv::Vec2i &a, const cv::Vec2i &b) const{
            return sqrt( pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2) ); };
        cv::Vec2i* jumpPoint(const cv::Vec2i &parent,
                             const cv::Vec2i &current,
                             const cv::Vec2i &target) const;
        std::list<cv::Vec2i> prunedNeighbors(Node &current) const;
        std::list<cv::Vec2i> straightForced(const cv::Vec2i &current,
                                            const cv::Vec2i &direction) const;
        cv::Vec2i* straightJPS(cv::Vec2i current,
                               const cv::Vec2i &target,
                               const cv::Vec2i &direction) const;

        cv::Mat map_; ///< Image used to calcutale the path, must be 8-Bit grey scale
        };

    /**
     *  Exception class thrown if a vector isn't on the map
    **/
    class NotOnMap : public virtual std::out_of_range{
        public:
        NotOnMap(const std::string &what) : std::out_of_range(what){};
        };

    typedef std::multimap< float, Node* > MinHeap;
    struct VecCmp{
        bool operator()(const Node *lhs, const Node *rhs) const{
            if(lhs->vector[0] < rhs->vector[0])
                return true;
            else if(lhs->vector[0] == rhs->vector[0])
                return lhs->vector[1] < rhs->vector[1];
            return false;
            }
        };
    typedef std::set<Node*, VecCmp> VecSet;
    }

#endif /* end of include guard: JPSASTAR_HPP_NBO2KO09 */
