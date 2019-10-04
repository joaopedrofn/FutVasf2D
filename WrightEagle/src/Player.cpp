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

#include "Player.h"
#include "DecisionTree.h"
#include "DynamicDebug.h"
#include "Formation.h"
#include "CommandSender.h"
#include "Parser.h"
#include "Thread.h"
#include "UDPSocket.h"
#include "WorldModel.h"
#include "Agent.h"
#include "VisualSystem.h"
#include "Logger.h"
#include "CommunicateSystem.h"
#include "TimeTest.h"
#include "Dasher.h"
#include <fstream>
#include "qLearning.h"
#include "PossibleActions.h"
//#include "PossibleStates.h"
#include <vector>

Player::Player() : mpDecisionTree(new DecisionTree)
{
}

Player::~Player()
{
	delete mpDecisionTree;
}

void Player::SendOptionToServer()
{
	while (!mpParser->IsClangOk())
	{
		mpAgent->CheckCommands(mpObserver);
		mpAgent->Clang(7, 8);
		mpObserver->SetCommandSend();
		WaitFor(200);
	}

	while (!mpParser->IsSyncOk())
	{
		mpAgent->CheckCommands(mpObserver);
		mpAgent->SynchSee();
		mpObserver->SetCommandSend();
		WaitFor(200);
	}

	mpAgent->CheckCommands(mpObserver);
	mpAgent->EarOff(false);
	mpObserver->SetCommandSend();
	WaitFor(200);
}

void Player::Run()
{
	//TIMETEST("Run");

	static Time last_time = Time(-100, 0);

	mpObserver->Lock();

	/** 下面几个更新顺序不能变 */
	Formation::instance.SetTeammateFormations();
	CommunicateSystem::instance().Update(); //在这里解析hear信息，必须首先更新
	mpAgent->CheckCommands(mpObserver);
	mpWorldModel->Update(mpObserver);

	mpObserver->UnLock();

	const Time &time = mpAgent->GetWorldState().CurrentTime();

	if (last_time.T() >= 0)
	{
		if (time != Time(last_time.T() + 1, 0) && time != Time(last_time.T(), last_time.S() + 1))
		{
			if (time == last_time)
			{
				mpAgent->World().SetCurrentTime(Time(last_time.T(), last_time.S() + 1)); //否则决策数据更新会出问题
			}
		}
	}

	last_time = time;

	Formation::instance.UpdateOpponentRole(); //TODO: 暂时放在这里，教练未发来对手阵型信息时自己先计算

	VisualSystem::instance().ResetVisualRequest();
	mpDecisionTree->Decision(*mpAgent);

	VisualSystem::instance().Decision();
	CommunicateSystem::instance().Decision();

	if (ServerParam::instance().synchMode())
	{
		mpAgent->Done();
	}

	mpAgent->SetHistoryActiveBehaviors();

	Logger::instance().LogSight();
	//QTable AREA
	/*==============================================//
	*
	*	Uncomment to trainning
	*
	==================================================*/
	///*
		if (++mpAgent->generalCycleCounter >= 3000)
		{
			mpAgent->generalCycleCounter = 0;

			std::cout << "Block of cycles " << ++mpAgent->countDone << ": " << mpAgent->goalCounter << " Goals" << std::endl;
			mpAgent->goalCounter = 0;
		}

		std::stringstream qTableString;
		qTableString << "qTable" << mpAgent->GetSelfUnum();
		std::ifstream qTableFileIn(qTableString.str(), std::ios::binary);
		double qTable[648][10];
		qTableFileIn.read((char *)&qTable, sizeof(qTable));
		qTableFileIn.close();

		int curState = mpAgent->lastStateOccurred;
		ServerPlayMode curSPM = mpObserver->GetServerPlayMode();
		
		int actionToTake = mpAgent->lastActionTaken;
		if (curState != -1 && (curSPM == SPM_Captured || curSPM == SPM_OutOfBounds || curSPM == SPM_Goal_Train || curSPM == SPM_PlayOn_1 || curSPM == SPM_PlayOn_2 || curSPM == SPM_PlayOn_3 || curSPM == SPM_PlayOn_4 || curSPM == SPM_PlayOn_5 || curSPM == SPM_PlayOn_6 || curSPM == SPM_PlayOn_7 || curSPM == SPM_PlayOn_8 || curSPM == SPM_PlayOn_9 || curSPM == SPM_PlayOn_10 || curSPM == SPM_PlayOn_11))
		{
			double reward = 0;
			int thatState = mpAgent->lastActionsState.size() ? mpAgent->lastActionsState[0] : -1;
			int thatAction = mpAgent->lastActions.size() ? mpAgent->lastActions[0] : -1;
			ServerPlayMode thatPM = mpAgent->lastActionsPM.size() ? mpAgent->lastActionsPM[0] : SPM_Null;
			std::vector<double> actionSpace{qTable[curState][0], qTable[curState][1], qTable[curState][2], qTable[curState][3], qTable[curState][4], qTable[curState][5], qTable[curState][6], qTable[curState][7], qTable[curState][8]};
			int maxFromCurrent = greedySelection(actionSpace);
			
			qTable[curState][9]++;
			
			PositionInfo  mPositionInfo = mpAgent->Info().GetPositionInfo();
			Unum closest_tm  = mPositionInfo.GetClosestTeammateToBall();

			if(mPositionInfo.GetTeammateWithBall() == mpAgent->GetSelfUnum() && mPositionInfo.GetLastWasOpp() == true){
				mPositionInfo.SetLastWasOpp(false);
				std::cout << "reward ->" << mpAgent->GetSelfUnum() << std::endl;
			}

			switch (mpObserver->GetServerPlayMode())
			{
			case SPM_Captured:
				if(mpAgent->GetSelfUnum() == closest_tm){
					mpAgent->cycleCounter = 0;
					mpAgent->lastActions.clear();
					mpAgent->lastActionsState.clear();
					mpAgent->lastActionsPM.clear();
					std::cout << "Capture -> " << closest_tm << std::endl;
					reward = 20;
					qTable[thatState][thatAction] = learn(qTable[thatState][thatAction], maxFromCurrent, reward);
				}
				break;
			case SPM_OutOfBounds:
				mpAgent->cycleCounter = 0;
				mpAgent->lastActions.clear();
				mpAgent->lastActionsState.clear();
				mpAgent->lastActionsPM.clear();

				// std::cout << "OUT OF BOUNDS\n";
				reward = 10;
				qTable[thatState][thatAction] = learn(qTable[thatState][thatAction], maxFromCurrent, reward);
				break;
			case SPM_Goal_Train:
				mpAgent->goalCounter++;
				mpAgent->cycleCounter = 0;
				mpAgent->lastActions.clear();
				mpAgent->lastActionsState.clear();
				mpAgent->lastActionsPM.clear();
				// std::cout << "GOAL :(\n";
				reward = -20;
				qTable[thatState][thatAction] = learn(qTable[thatState][thatAction], maxFromCurrent, reward);
				break;
			case SPM_TimeOut:
				mpAgent->cycleCounter = 0;
				mpAgent->lastActions.clear();
				mpAgent->lastActionsState.clear();
				mpAgent->lastActionsPM.clear();
				break;
			default:
				break;
			}
			char side = mpObserver->OurInitSide();
			Vector goal(side == 'l' ? 51.162 : -51.162, 0);
			// std::cout << goal << std::endl;
			if (mpAgent->cycleCounter == 1)
			{
				mpAgent->cycleCounter = 0;
				mpAgent->lastActions.erase(mpAgent->lastActions.begin());
				mpAgent->lastActionsState.erase(mpAgent->lastActionsState.begin());
				mpAgent->lastActionsPM.erase(mpAgent->lastActionsPM.begin());
				switch (mpObserver->GetServerPlayMode())
				{
				case SPM_PlayOn_11:
				case SPM_PlayOn_10:
				case SPM_PlayOn_9:
				case SPM_PlayOn_8:
				case SPM_PlayOn_7:
				case SPM_PlayOn_6:
				case SPM_PlayOn_5:
				case SPM_PlayOn_4:
				case SPM_PlayOn_3:
				case SPM_PlayOn_2:
				case SPM_PlayOn_1:
					if (mpAgent->lastBallPosition.Dist(goal) > mpAgent->lastGoalDist)
					{
						if (mpAgent->lastBallPosition.Dist(mpAgent->lastPosition) < mpAgent->lastPlayerDist)
							reward = 5;
						else
							reward = 2;

						qTable[thatState][thatAction] = learn(qTable[thatState][thatAction], maxFromCurrent, reward);
					}
					break;
				default:
					break;
				}

			}

			mpAgent->lastGoalDist = mpAgent->lastBallPosition.Dist(goal);
			mpAgent->lastPlayerDist = mpAgent->lastBallPosition.Dist(mpAgent->lastPosition);

			std::ofstream qTableFileOut(qTableString.str(), std::ios::binary);
			qTableFileOut.write((char *)&qTable, sizeof(qTable));
			qTableFileOut.close();
		}
	//*/
	//END OF QTable AREA

	
		
}
