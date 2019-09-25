/************************************************************************************
 * WrightEagle (Soccer Simulation League 2D)                                        *
 * BASE SOURCE CODE RELEASE 2016                                                    *
 * Copyright (c) 1998-2016 WrightEagle 2D Soccer Simulation Team,                   *
 *                         Multi-Agent Systems Lab.,                                *
 *                         School of Computer Science and Technology,               *
 *                         University of Science and Technology of China            *
 * All rights reserved.                                                             *
 *                                                                                  *
 * Redistribution and use in source and binary forms, with or without               *
 * modification, are permitted provided that the following conditions are met:      *
 *     * Redistributions of source code must retain the above copyright             *
 *       notice, this list of conditions and the following disclaimer.              *
 *     * Redistributions in binary form must reproduce the above copyright          *
 *       notice, this list of conditions and the following disclaimer in the        *
 *       documentation and/or other materials provided with the distribution.       *
 *     * Neither the name of the WrightEagle 2D Soccer Simulation Team nor the      *
 *       names of its contributors may be used to endorse or promote products       *
 *       derived from this software without specific prior written permission.      *
 *                                                                                  *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND  *
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED    *
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
 * DISCLAIMED. IN NO EVENT SHALL WrightEagle 2D Soccer Simulation Team BE LIABLE    *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL       *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR       *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER       *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,    *
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF *
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                *
 ************************************************************************************/

#include "BehaviorDefense.h"
#include "BehaviorFormation.h"
#include "BehaviorBlock.h"
#include "BehaviorMark.h"
#include "WorldState.h"
#include "Agent.h"
#include "Formation.h"
#include "Dasher.h"
#include "Logger.h"
#include "BehaviorIntercept.h"
#include "Utilities.h"
#include <fstream>
#include "qLearning.h"
#include "PossibleActions.h"
#include "PossibleStates.h"

BehaviorDefensePlanner::BehaviorDefensePlanner(Agent &agent) : BehaviorPlannerBase<BehaviorDefenseData>(agent)
{
}

BehaviorDefensePlanner::~BehaviorDefensePlanner()
{
}

void BehaviorDefensePlanner::Plan(std::list<ActiveBehavior> &behavior_list)
{
	std::stringstream qTableString;
	qTableString << "qTable" << mSelfState.GetUnum();
	std::ifstream qTableFileIn(qTableString.str(), std::ios::binary);
	double qTable[648][10];
	qTableFileIn.read((char *)&qTable, sizeof(qTable));
	qTableFileIn.close();
	mAgent.lastPosition = mSelfState.GetPos();
	mAgent.lastBallPosition = mWorldState.GetBall().GetPos();
	//GETTING VARIABLES
	double distToBall = mPositionInfo.GetBallDistToTeammate(mSelfState.GetUnum());
	Unum closestTeammate = mPositionInfo.GetCloseTeammateToBall()[0];
	double amITheClosest = closestTeammate == mSelfState.GetUnum();
	double teammateDistToBall = amITheClosest ? distToBall : mPositionInfo.GetBallDistToTeammate(closestTeammate);
	Unum opponent = mPositionInfo.GetOpponentWithBall();
	Vector oppPosition = mWorldState.GetOpponent(opponent).GetPos();
	Vector goaliePosition = mWorldState.GetTeammate(mWorldState.GetTeammateGoalieUnum()).GetPos();
	Vector goalPosition(goaliePosition.X() > 0 ? 51.162 : -51.162, 0);
	double oppDistToGoal = oppPosition.Dist(goalPosition);
	double teammatesDistToOpp[10];
	int index = 0;
	double avgDist = 0;
	for (int i = 1; i <= 11; i++)
	{
		if (i != mWorldState.GetTeammateGoalieUnum())
		{
			double aux = mWorldState.GetTeammate(i).GetPos().Dist(oppPosition);
			teammatesDistToOpp[index++] = aux;
		}
	}
	int n = sizeof(teammatesDistToOpp) / sizeof(teammatesDistToOpp[0]);
	std::sort(teammatesDistToOpp, teammatesDistToOpp + (n));
	avgDist = teammatesDistToOpp[0] + teammatesDistToOpp[1] + teammatesDistToOpp[2] + teammatesDistToOpp[3];
	avgDist /= 4;
	double sum = 0;
	for (int i = 0; i < 4; i++)
	{
		sum += (teammatesDistToOpp[i] - avgDist) * (teammatesDistToOpp[i] - avgDist);
	}
	double density = sqrt(sum / 4);
	Vector ballPosition = mWorldState.GetBall().GetPos();
	Vector position = mSelfState.GetPos();
	bool north = ballPosition.Y() > position.Y();
	bool west = ballPosition.X() < position.X();

	int curState = GetState(distToBall, teammateDistToBall, amITheClosest, oppDistToGoal, density, north, west);
	
	mAgent.lastStateOccurred = curState;

	std::vector<double> actionSpace{qTable[curState][0], qTable[curState][1], qTable[curState][2], qTable[curState][3], qTable[curState][4], qTable[curState][5], qTable[curState][6], qTable[curState][7], qTable[curState][8]};
	int actionToTake = greedyEpSelection(actionSpace, (1 - (qTable[curState][9] / 100000))); //valor diferente.

	//Uncomment to Trainning

	//mAgent.lastActionTaken = actionToTake;

	double power = mSelfState.CorrectDashPowerForStamina(ServerParam::instance().maxDashPower());
	int step = 10;

	switch (actionToTake)
	{
	case MoveNorth:
		if (mSelfState.GetPos().Y() + 2 <= 34)
			Dasher::instance().GoToPoint(mAgent, Vector(mSelfState.GetPos().X(), mSelfState.GetPos().Y() + step), 1.0, power, false, true);
		break;
	case MoveSouth:
		if (mSelfState.GetPos().Y() - 2 >= -34)
			Dasher::instance().GoToPoint(mAgent, Vector(mSelfState.GetPos().X(), mSelfState.GetPos().Y() - step), 1.0, power, false, true);
		break;
	case MoveWest:
		if (mSelfState.GetPos().X() - 2 >= -52.5)
			Dasher::instance().GoToPoint(mAgent, Vector(mSelfState.GetPos().X() - step, mSelfState.GetPos().Y()), 1.0, power, false, true);
		break;
	case MoveEast:
		if (mSelfState.GetPos().X() + 2 <= 52.5)
			Dasher::instance().GoToPoint(mAgent, Vector(mSelfState.GetPos().X() + step, mSelfState.GetPos().Y()), 1.0, power, false, true);
		break;
	case MoveToBall:
		Dasher::instance().GoToPoint(mAgent, ballPosition, 1.0, power, false, true);
		break;
	case InterceptAction:
		BehaviorInterceptPlanner(mAgent).Plan(behavior_list);
		break;
	case BlockAction:
		BehaviorBlockPlanner(mAgent).Plan(behavior_list);
		break;
	case MarkAction:
		BehaviorMarkPlanner(mAgent).Plan(behavior_list);
		break;
	case StayStill:
	default:
		break;
	}

	// Uncomment to Trainning

	/*
	
	mAgent.lastActions.push_back(actionToTake);
	mAgent.lastActionsState.push_back(curState);
	PlayMode pm = mWorldState.GetPlayMode();
	ServerPlayMode spm = SPM_Null;
	switch (pm)
	{
	case PM_Goal_Opps:
		spm = SPM_Goal_Train;
		break;
	case PM_Captured:
		spm = SPM_Captured;
		break;
	case PM_OutOfBounds:
		spm = SPM_OutOfBounds;
		break;
	case PM_Play_On_11:
		spm = SPM_PlayOn_11;
		break;
	case PM_Play_On_10:
		spm = SPM_PlayOn_1;
		break;
	case PM_Play_On_9:
		spm = SPM_PlayOn_9;
		break;
	case PM_Play_On_8:
		spm = SPM_PlayOn_8;
		break;
	case PM_Play_On_7:
		spm = SPM_PlayOn_7;
		break;
	case PM_Play_On_6:
		spm = SPM_PlayOn_6;
		break;
	case PM_Play_On_5:
		spm = SPM_PlayOn_5;
		break;
	case PM_Play_On_4:
		spm = SPM_PlayOn_4;
		break;
	case PM_Play_On_3:
		spm = SPM_PlayOn_3;
		break;
	case PM_Play_On_2:
		spm = SPM_PlayOn_2;
		break;
	case PM_Play_On_1:
		spm = SPM_PlayOn_1;
		break;
	default:
		break;
	}

	mAgent.lastActionsPM.push_back(spm);
	mAgent.cycleCounter++;

	*/

	if (!mActiveBehaviorList.empty())
	{
		mActiveBehaviorList.sort(std::greater<ActiveBehavior>());
		behavior_list.push_back(mActiveBehaviorList.front());

		if (mActiveBehaviorList.size() > 1)
		{ //允许非最优行为提交视觉请求
			double plus = 1.0;
			ActiveBehaviorPtr it = mActiveBehaviorList.begin();
			for (++it; it != mActiveBehaviorList.end(); ++it)
			{
				it->SubmitVisualRequest(plus);
				plus *= 2.0;
			}
		}
	}
}
