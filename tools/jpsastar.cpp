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

#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "jpsastar/JPSAStar.hpp"

namespace po = boost::program_options;

cv::Mat map_color;
cv::Mat map_draw;
cv::Vec2i start(-1, -1);
cv::Vec2i destination(-1, -1);
jpsastar::JPSAStar algo( (cv::Mat()) );

static void mouseClick(int event, int x, int y, int flags, void *param){
    if(event != cv::EVENT_LBUTTONDOWN)
        return;

    if(start[0] == -1 && destination[0] == -1)
        map_draw = map_color.clone();
    if(start[0] == -1){
        start << x,y;
        cv::circle(map_draw, cv::Point(x, y), 2, cv::Scalar(0,255,0), -1);
        }
    else{
        destination << x,y;
        std::list<cv::Vec2i> path = algo.findPath(start, destination);
        if( path.empty() ){
            // No path found, draw X
            cv::line( map_draw,
                      cv::Point(destination[0]-2, destination[1]-2),
                      cv::Point(destination[0]+2, destination[1]+2),
                      cv::Scalar(0,0,255) );
            cv::line( map_draw,
                      cv::Point(destination[0]+2, destination[1]-2),
                      cv::Point(destination[0]-2, destination[1]+2),
                      cv::Scalar(0,0,255) );
            }
        else{
            cv::Vec2i prev = path.front();
            path.pop_front();
            for(auto &vec : path){
                cv::line( map_draw, cv::Point(prev[0], prev[1]), cv::Point(vec[0], vec[1]), cv::Scalar(0,255,0) );
                prev = vec;
                }
            cv::circle( map_draw, cv::Point(x, y), 2, cv::Scalar(0,0,255), -1);
            }
        start << -1,-1;
        destination << -1,-1;
        }
    cv::imshow("JPSAStar", map_draw);
    }


int main(int argc, char *argv[]){
    // Parse commandline options
    po::options_description options("Options");
    options.add_options()("help,h", "Show this help output.")
                         ("map,m", po::value< std::string >(), "Path to the image of the map");
    po::positional_options_description operands;
    operands.add("map", 1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(options).positional(operands).run(), vm);
    po::notify(vm);

    auto printHelp = [&options](){ std::cout << "Usage: jpsastar IMAGE_FILE" << std::endl
                                             << "       jpsastar [Options]" << std::endl << std::endl
                                             << options << std::endl; };
    if(vm.count("help")){
        printHelp();
        return 1;
        }
    if(vm.count("map")){
        map_color = cv::imread(vm["map"].as<std::string>(), CV_LOAD_IMAGE_COLOR);
        }
    else{
        printHelp();
        std::cout << "[ERROR] Map file-path missing!" << std::endl;
        return 1;
        }

    cv::Mat map_thres;
    cv::cvtColor(map_color, map_thres, CV_BGR2GRAY);
    cv::threshold(map_thres, map_thres, 230, 255, cv::THRESH_BINARY);
    algo.setMap(map_thres);
    cv::namedWindow("JPSAStar");
    cv::imshow("JPSAStar", map_color);
    cv::namedWindow("threshold");
    cv::imshow("threshold", map_thres);

    cv::setMouseCallback("JPSAStar", mouseClick, 0);
    while(true){
        char c = cv::waitKey(0);
        if(c == 'q')
            break;
        }
    return 0;
    }

