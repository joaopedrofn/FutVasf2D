#ifndef QLEARNING
#define QLEARNING

#include <vector>
#include <iostream>
#include "worldModel.h"

//current, max from next, reward
static double learn(int u, int v, int r)
{
    //std::cout << "learn" << std::endl;
    return (u + alpha * (r + (qGamma * v) - u));
    // return ((1-alpha) * u + (alpha * (r + qGamma * v)));
}

static int greedySelection(std::vector<double> actions)
{

    double biggest = -1000000000.0;
    int bestAction = -1;
    int numTies = 0;

    for (uint i = 0; i < actions.size(); i++)
    {

        if (biggest < actions[i])
        {

            biggest = actions[i];
            bestAction = i;
        }
        else if (biggest == actions[i])
        {

            numTies++;
            if (rand() % (numTies + 1) == 0)
            {
                biggest = actions[i];
                bestAction = i;
            }
        }
    }
    return bestAction;
}

static int greedyEpSelection(std::vector<double> actions, double epsilon)
{

    double x = drand48();
    if (x < epsilon)
    {
        int v = int(rand() % 9);
        //std::cout << v << std::endl;
        return v;
    }
    else
    {
        return greedySelection(actions);
    }
}

#endif
