
/*
 * STUDENT CODE - EDIT THIS FILE
 */

#include "search.h"

#include <algorithm>
#include <queue>
#include <vector>

Search::Search(Mothership &mothership)
    : mothership(mothership)
{
}

SearchResult Search::findShipwreck()
{
    const int robotCount = mothership.getRobotCount();

    if (robotCount == 0)
    {
        return SearchResult{false, Location{}};
    }

    // Store the last safe segment for each robot.
    std::vector<int> limits(robotCount, TETHER_LENGTH);

    for (const Location &abyss : mothership.getAbysses())
    {
        if (abyss.robot >= 0 && abyss.robot < robotCount)
        {
            limits[abyss.robot] = std::min(
                limits[abyss.robot], abyss.segment - 1);
        }
    }

    // Track each robot's position and debris signal.
    std::vector<int> positions(robotCount, 0);
    std::vector<int> signals(robotCount, 0);

    // A priority queue chooses the most promising robot.
    struct RobotPriority
    {
        int score;
        int robot;

        bool operator<(const RobotPriority &other) const
        {
            if (score != other.score)
            {
                return score < other.score;
            }

            return robot > other.robot;
        }
    };

    std::priority_queue<RobotPriority> pq;

    for (int robot = 0; robot < robotCount; ++robot)
    {
        positions[robot] = mothership.getRobotPosition(robot);

        if (positions[robot] < limits[robot])
        {
            pq.push({0, robot});
        }
    }

    int activeRobot = mothership.getActiveRobot();

    while (!pq.empty())
    {
        RobotPriority current = pq.top();
        pq.pop();

        const int robot = current.robot;

        // Ignore outdated priority entries.
        if (current.score != signals[robot])
        {
            continue;
        }

        if (positions[robot] >= limits[robot])
        {
            continue;
        }

        if (activeRobot != robot)
        {
            mothership.switchToRobot(robot);
            activeRobot = robot;
        }

        
        // Explore up to 10 drives before choosing another robot.
        // Each drive still respects the 100-segment limit.
        const int drivesPerTurn = 10;

        for (int drive = 0; drive < drivesPerTurn; ++drive)
        {
            if (positions[robot] >= limits[robot])
            {
                break;
            }

            const int safeDistance =
                limits[robot] - positions[robot];

            const int distance = std::min(
                MAX_SEGMENTS_PER_DRIVE, safeDistance);

            DriveResult result =
                mothership.driveActiveRobot(distance);

            positions[robot] = mothership.getRobotPosition(robot);

            if (result.reason == DriveStopReason::ShipwreckFound)
            {
                return SearchResult{
                    true,
                    Location{robot, positions[robot]}
                };
            }

            if (result.reason == DriveStopReason::DebrisFound)
            {
                seafloorSurvey.recordDebris(
                    Location{robot, positions[robot]});

                
              for (int offset = -1; offset <= 1; ++offset)
              {
                  int neighbor =
                    (robot + offset + robotCount) % robotCount;

                  if (positions[neighbor] < limits[neighbor])
                  {
                      ++signals[neighbor];
                      pq.push({signals[neighbor], neighbor});
                  }
              }
            }
        }


        if (positions[robot] < limits[robot])
        {
            pq.push({signals[robot], robot});
        }
    }

    return SearchResult{false, Location{}};
}
