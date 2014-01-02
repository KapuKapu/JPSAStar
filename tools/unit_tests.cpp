#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <algorithm>
#include <string>
#include <opencv2/opencv.hpp>

#define private public
#include "jpsastar/JPSAStar.hpp"
#undef private


std::string to_string(const std::list<cv::Vec2i> &vec_list){
                       std::string r = "[ ";
                       for(auto &vec : vec_list){
                           r += "(" + std::to_string(vec[0]) + "," + std::to_string(vec[1]) + ") ";
                           }
                       r += "]";
                       return r;
                       };

std::string to_string(const cv::Vec2i &vec){
                       return "(" + std::to_string(vec[0]) + "," + std::to_string(vec[1]) + ")";
                       };


TEST(PruneNeighbors, PartentNULL){
    std::list<cv::Vec2i> expected, pruned;
    cv::Mat map5x5 = (cv::Mat_<char>(5,5) << 255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255);

    expected.push_back( cv::Vec2i(0,0) );
    expected.push_back( cv::Vec2i(1,0) );
    expected.push_back( cv::Vec2i(2,0) );
    expected.push_back( cv::Vec2i(0,1) );
    expected.push_back( cv::Vec2i(2,1) );
    expected.push_back( cv::Vec2i(0,2) );
    expected.push_back( cv::Vec2i(1,2) );
    expected.push_back( cv::Vec2i(2,2) );

    jpsastar::JPSAStar jpsastar(map5x5);
    jpsastar::Node current(cv::Vec2i(1,1), NULL);
    pruned = jpsastar.prunedNeighbors(current);

    ASSERT_THAT( expected, testing::UnorderedElementsAreArray(pruned.begin(), pruned.end()) )
        << "Expected: " << to_string(expected) << "\n"
        << "  Actual: " << to_string(pruned);
    }


TEST(PruneNeighbors, StraightForcedRight){
    std::list<cv::Vec2i> expected, pruned;
    cv::Mat map5x5 = (cv::Mat_<char>(5,5) << 255, 255, 255, 255, 255,
                                             255, 255,   0, 255, 255,
                                             255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255);

    expected.push_back( cv::Vec2i(3,1) );
    expected.push_back( cv::Vec2i(3,2) );

    jpsastar::JPSAStar jpsastar(map5x5);
    jpsastar::Node parent(cv::Vec2i(1,2), NULL);
    jpsastar::Node current(cv::Vec2i(2,2), &parent);
    pruned = jpsastar.prunedNeighbors(current);

    ASSERT_THAT( expected, testing::UnorderedElementsAreArray(pruned.begin(), pruned.end()) )
        << "Expected: " << to_string(expected) << "\n"
        << "  Actual: " << to_string(pruned);
    }


TEST(PruneNeighbors, StraightForcedLeft){
    std::list<cv::Vec2i> expected, pruned;
    cv::Mat map5x5 = (cv::Mat_<char>(5,5) << 255, 255, 255, 255, 255,
                                             255, 255,   0, 255, 255,
                                             255, 255, 255, 255, 255,
                                             255, 255,   0, 255, 255,
                                             255, 255, 255, 255, 255);

    expected.push_back( cv::Vec2i(1,1) );
    expected.push_back( cv::Vec2i(1,2) );
    expected.push_back( cv::Vec2i(1,3) );

    jpsastar::JPSAStar jpsastar(map5x5);
    jpsastar::Node parent(cv::Vec2i(3,2), NULL);
    jpsastar::Node current(cv::Vec2i(2,2), &parent);
    pruned = jpsastar.prunedNeighbors(current);

    ASSERT_THAT( expected, testing::UnorderedElementsAreArray(pruned.begin(), pruned.end()) )
        << "Expected: " << to_string(expected) << "\n"
        << "  Actual: " << to_string(pruned);
    }


TEST(PruneNeighbors, StraightForcedUp){
    std::list<cv::Vec2i> expected, pruned;
    cv::Mat map5x5 = (cv::Mat_<char>(5,5) << 255, 255, 255, 255, 255,
                                             255,   0,   0, 255, 255,
                                             255,   0, 255,   0, 255,
                                             255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255);

    expected.push_back( cv::Vec2i(3,1) );

    jpsastar::JPSAStar jpsastar(map5x5);
    jpsastar::Node parent(cv::Vec2i(2,3), NULL);
    jpsastar::Node current(cv::Vec2i(2,2), &parent);
    pruned = jpsastar.prunedNeighbors(current);

    ASSERT_THAT( expected, testing::UnorderedElementsAreArray(pruned.begin(), pruned.end()) )
        << "Expected: " << to_string(expected) << "\n"
        << "  Actual: " << to_string(pruned);
    }


TEST(PruneNeighbors, StraightWallDown){
    std::list<cv::Vec2i> expected, pruned;
    cv::Mat map5x5 = (cv::Mat_<char>(5,5) << 255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255,
                                               0,   0,   0,   0,   0);

    jpsastar::JPSAStar jpsastar(map5x5);
    jpsastar::Node parent(cv::Vec2i(2,2), NULL);
    jpsastar::Node current(cv::Vec2i(2,3), &parent);
    pruned = jpsastar.prunedNeighbors(current);

    ASSERT_THAT( expected, testing::UnorderedElementsAreArray(pruned.begin(), pruned.end()) )
        << "Expected: " << to_string(expected) << "\n"
        << "  Actual: " << to_string(pruned);
    }


TEST(PruneNeighbors, DiagonalNatural){
    std::list<cv::Vec2i> expected, pruned;
    cv::Mat map5x5 = (cv::Mat_<char>(5,5) << 255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255);
    expected.push_back( cv::Vec2i(2,3) );
    expected.push_back( cv::Vec2i(3,3) );
    expected.push_back( cv::Vec2i(3,2) );

    jpsastar::JPSAStar jpsastar(map5x5);
    jpsastar::Node parent(cv::Vec2i(1,1), NULL);
    jpsastar::Node current(cv::Vec2i(2,2), &parent);
    pruned = jpsastar.prunedNeighbors(current);

    ASSERT_THAT( expected, testing::UnorderedElementsAreArray(pruned.begin(), pruned.end()) )
        << "Expected: " << to_string(expected) << "\n"
        << "  Actual: " << to_string(pruned);
    }


TEST(PruneNeighbors, DiagonalForced){
    std::list<cv::Vec2i> expected, pruned;
    cv::Mat map5x5 = (cv::Mat_<char>(5,5) << 255, 255, 255, 255, 255,
                                             255, 255,   0, 255, 255,
                                             255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255);
    expected.push_back( cv::Vec2i(1,1) );
    expected.push_back( cv::Vec2i(1,2) );
    expected.push_back( cv::Vec2i(1,3) );
    expected.push_back( cv::Vec2i(2,3) );

    jpsastar::JPSAStar jpsastar(map5x5);
    jpsastar::Node parent(cv::Vec2i(3,1), NULL);
    jpsastar::Node current(cv::Vec2i(2,2), &parent);
    pruned = jpsastar.prunedNeighbors(current);

    ASSERT_THAT( expected, testing::UnorderedElementsAreArray(pruned.begin(), pruned.end()) )
        << "Expected: " << to_string(expected) << "\n"
        << "  Actual: " << to_string(pruned);
    }


TEST(PruneNeighbors, DiagonalWall){
    std::list<cv::Vec2i> expected, pruned;
    cv::Mat map5x5 = (cv::Mat_<char>(5,5) << 255, 255, 255, 255, 255,
                                             255,   0,   0, 255, 255,
                                             255,   0, 255, 255, 255,
                                             255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255);
    jpsastar::JPSAStar jpsastar(map5x5);
    jpsastar::Node parent(cv::Vec2i(3,3), NULL);
    jpsastar::Node current(cv::Vec2i(2,2), &parent);
    pruned = jpsastar.prunedNeighbors(current);

    ASSERT_THAT( expected, testing::UnorderedElementsAreArray(pruned.begin(), pruned.end()) )
        << "Expected: " << to_string(expected) << "\n"
        << "  Actual: " << to_string(pruned);
    }


TEST(JumpPointSearch, StraightWall){
    cv::Mat map5x5 = (cv::Mat_<char>(5,5) << 0, 255, 255, 255, 255,
                                             0, 255, 255, 255, 255,
                                             0, 255, 255, 255, 255,
                                             0, 255, 255, 255, 255,
                                             0, 255, 255, 255, 255);
    jpsastar::JPSAStar jpsastar(map5x5);
    cv::Vec2i current(4,2);
    cv::Vec2i neighbor(3,2);
    cv::Vec2i *jp = jpsastar.jumpPoint( current, neighbor, cv::Vec2i(0,0) );

    ASSERT_THAT(jp, testing::IsNull());
    }


TEST(JumpPointSearch, StraightJP){
    cv::Mat map5x5 = (cv::Mat_<char>(5,5) << 255, 255,   0, 255, 255,
                                             255, 255,   0, 255, 255,
                                             255, 255,   0, 255, 255,
                                             255, 255,   0, 255, 255,
                                             255, 255, 255, 255, 255);
    jpsastar::JPSAStar jpsastar(map5x5);
    cv::Vec2i current(1,0);
    cv::Vec2i neighbor(1,1);
    cv::Vec2i *jp = jpsastar.jumpPoint( current, neighbor, cv::Vec2i(0,0) );

    ASSERT_EQ( *jp, cv::Vec2i(1,3) );
    }


TEST(JumpPointSearch, DiagonalWall){
    cv::Mat map5x5 = (cv::Mat_<char>(5,5) << 255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255,
                                             255, 255, 255, 255,   0,
                                             255, 255, 255, 255,   0);
    jpsastar::JPSAStar jpsastar(map5x5);
    cv::Vec2i current(0,0);
    cv::Vec2i neighbor(1,1);
    cv::Vec2i *jp = jpsastar.jumpPoint( current, neighbor, cv::Vec2i(0,0) );

    ASSERT_THAT(jp, testing::IsNull())
        << "Expected: NULL\n"
        << "  Actual: " << to_string(*jp);
    }


TEST(JumpPointSearch, DiagonalJPNeighbor){
    cv::Mat map5x5 = (cv::Mat_<char>(5,5) << 255, 255, 255, 255, 255,
                                             255, 255, 255,   0, 255,
                                             255, 255, 255,   0, 255,
                                             255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255);
    cv::Vec2i expected(1,3);

    jpsastar::JPSAStar jpsastar(map5x5);
    cv::Vec2i current(0,4);
    cv::Vec2i neighbor(1,3);
    cv::Vec2i *jp = jpsastar.jumpPoint( current, neighbor, cv::Vec2i(0,0) );

    ASSERT_EQ(*jp, expected)
        << "Expected: " << to_string(expected) << "\n"
        << "  Actual: " << to_string(*jp);
    }


TEST(JumpPointSearch, DiagonalJPDiag){
    cv::Mat map5x5 = (cv::Mat_<char>(6,8) << 255, 255, 255, 255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255, 255, 255, 255,
                                             255, 255, 255, 255, 255, 255,   0, 255,
                                             255, 255, 255, 255, 255, 255,   0, 255,
                                             255, 255, 255, 255, 255, 255,   0, 255);
    cv::Vec2i expected(3,2);

    jpsastar::JPSAStar jpsastar(map5x5);
    cv::Vec2i current(0,5);
    cv::Vec2i neighbor(1,4);
    cv::Vec2i *jp = jpsastar.jumpPoint( current, neighbor, cv::Vec2i(0,0) );

    ASSERT_EQ(*jp, expected)
        << "Expected: " << to_string(expected) << "\n"
        << "  Actual: " << to_string(*jp);
    }


TEST(JPSAStar, EmptyMap){
    jpsastar::JPSAStar jpsastar( (cv::Mat()) );
    cv::Vec2i start(0,5);
    cv::Vec2i target(19,40);

    ASSERT_THROW(jpsastar.findPath(start, target), jpsastar::NotOnMap)
        << "Expected: Throws exception\n"
        << "  Actual: ???";
    }


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    }
