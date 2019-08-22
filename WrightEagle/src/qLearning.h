#ifndef QLEARNING
#define QLEARNING

#include <vector>
#include <iostream>
#include "worldModel.h"

#include "qLearning.h"
//current, max from next, reward
static double learn(int u, int v, int r)
{
    return (u + alpha * (r + (qGamma * v) - u));
    // return ((1-alpha) * u + (alpha * (r + qGamma * v)));
}

static int greedySelection(vector<double> actions)
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

static int greedyEpSelection(vector<double> actions, double epsilon)
{

    double x = drand48();
    if (x < epsilon)
    {
        int v = int(rand() % 4);
        return v;
    }
    else
    {
        return greedySelection(actions);
    }
}

static int future(int currentState, int action)
{

    int FutureState;

    switch (action)
    {

    case 0:

        FutureState = currentState - 4;
        break;
    case 1:

        FutureState = currentState + 4;
        break;
    case 2:

        if (currentState == 3 || currentState == 7 || currentState == 11 || currentState == 15)
        {
            FutureState = -1;
        }
        else
        {

            FutureState = currentState + 1;
        }
        break;
    case 3:

        if (currentState == 0 || currentState == 4 || currentState == 8 || currentState == 12)
        {
            FutureState = -1;
        }
        else
        {

            FutureState = currentState - 1;
        }
        break;
    default:
        cout << "DEU RUIM NO FUTURE" << endl;
        exit(1);
        break;
    }

    return FutureState;
}

#endif
