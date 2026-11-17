
/*
 * STUDENT CODE - EDIT THIS FILE
 */

#include "search.h"

#include <algorithm>
#include <cstdlib>
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

    // Track each robot's position and debris signal.
    std::vector<int> positions(robotCount);
    std::vector<int> signals(robotCount, 0);

    // Track the most recent debris location for each robot.
    std::vector<int> lastDebris(robotCount, -1);

    // Determine the safe travel limit for each robot.
    for (const Location &abyss : mothership.getAbysses())
    {
        if (abyss.robot >= 0 && abyss.robot < robotCount)
        {
            limits[abyss.robot] = std::min(
                limits[abyss.robot], abyss.segment - 1);
        }
    }

    // Get the current position of every robot.
    for (int robot = 0; robot < robotCount; ++robot)
    {
        positions[robot] = mothership.getRobotPosition(robot);
    }

    int activeRobot = mothership.getActiveRobot();

    // Explore in layers of 1000 segments.
    const int layerSize = 1000;

    for (int layerStart = 0;
         layerStart < TETHER_LENGTH;
         layerStart += layerSize)
    {
        bool anyReachable = false;

        // Store robots that can still explore.
        std::vector<int> order;

        for (int robot = 0; robot < robotCount; ++robot)
        {
            if (positions[robot] < limits[robot])
            {
                order.push_back(robot);
                anyReachable = true;
            }
        }

        // Stop if no robots can travel farther.
        if (!anyReachable)
        {
            break;
        }

        // Prioritize robots with recent nearby debris.
        std::sort(order.begin(), order.end(),
            [&](int a, int b)
            {
                int scoreA = 0;
                int scoreB = 0;

                if (lastDebris[a] >= 0 &&
                    std::abs(positions[a] - lastDebris[a])
                        <= SIGNAL_RANGE)
                {
                    scoreA = signals[a];
                }

                if (lastDebris[b] >= 0 &&
                    std::abs(positions[b] - lastDebris[b])
                        <= SIGNAL_RANGE)
                {
                    scoreB = signals[b];
                }

                if (scoreA != scoreB)
                {
                    return scoreA > scoreB;
                }

                return a < b;
            });

        // Explore robots in priority order.
        for (int robot : order)
        {
            int target = std::min(
                limits[robot], layerStart + layerSize);

            if (positions[robot] >= target)
            {
                continue;
            }

            // Switch to the selected robot if necessary.
            if (activeRobot != robot)
            {
                mothership.switchToRobot(robot);
                activeRobot = robot;
            }

            // Drive until the robot reaches this layer's target.
            while (positions[robot] < target)
            {
                int distance = std::min(
                    MAX_SEGMENTS_PER_DRIVE,
                    target - positions[robot]);

                DriveResult result =
                    mothership.driveActiveRobot(distance);

                positions[robot] =
                    mothership.getRobotPosition(robot);

                // Return immediately if the shipwreck is found.
                if (result.reason ==
                    DriveStopReason::ShipwreckFound)
                {
                    return SearchResult{
                        true,
                        Location{robot, positions[robot]}
                    };
                }

                // Record debris and update nearby robot signals.
                if (result.reason ==
                    DriveStopReason::DebrisFound)
                {
                    seafloorSurvey.recordDebris(
                        Location{robot, positions[robot]});

                    for (int offset = -1;
                         offset <= 1; ++offset)
                    {
                        int neighbor =
                            (robot + offset + robotCount)
                            % robotCount;

                        ++signals[neighbor];
                        lastDebris[neighbor] = positions[robot];
                    }
                }
            }
        }
    }

    // No reachable shipwreck was found.
    return SearchResult{false, Location{}};
}
