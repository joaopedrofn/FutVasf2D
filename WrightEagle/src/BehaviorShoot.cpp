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

#include "BehaviorShoot.h"
#include "ServerParam.h"
#include "PlayerParam.h"
#include "VisualSystem.h"
#include "Utilities.h"
#include "Dasher.h"
#include "Tackler.h"
#include "Kicker.h"
#include "InterceptModel.h"
#include "ActionEffector.h"
#include "TimeTest.h"
#include "Evaluation.h"
#include <list>
#include <cmath>
#include "PositionInfo.h"
#include "Geometry.h"
#include "BehaviorBase.h"

using namespace std;

namespace {
bool ret = BehaviorExecutable::AutoRegister<BehaviorShootExecuter>();
}

const BehaviorType BehaviorShootExecuter::BEHAVIOR_TYPE = BT_Shoot;

/**
 * BehaviorShootExecuter's constructor
 */
BehaviorShootExecuter::BehaviorShootExecuter(Agent & agent): BehaviorExecuterBase<BehaviorAttackData>(agent)
{
	Assert(ret);
}

/**
 * BehaviorShootExecuter's constructor
 */
BehaviorShootExecuter::~BehaviorShootExecuter()
{
}

/**
 * Execute an active behavior as shoot. May raise ball or player.
 * \param act_bhv
 * \return true  if success;
 *         false otherwise.
 */
bool BehaviorShootExecuter::Execute(const ActiveBehavior & shoot)
{
	Logger::instance().LogShoot(mBallState.GetPos(), shoot.mTarget, "@Shoot");
	if (shoot.mDetailType == BDT_Shoot_Tackle)
		return Tackler::instance().TackleToDir(mAgent,shoot.mAngle);
	else{
		
		ofstream myfile;
		myfile.open ("shootsLog.txt", ios::app);
		myfile << mWorldState.CurrentTime() << '\n';
		myfile.close();

		return Kicker::instance().KickBall(mAgent, shoot.mTarget, ServerParam::instance().ballSpeedMax(), KM_Quick, 0, true);
	}
}

/**
 * BehaviorShootPlanner's constructor
 */
BehaviorShootPlanner::BehaviorShootPlanner(Agent & agent):
	BehaviorPlannerBase<BehaviorAttackData>(agent)
{
}

/**
 * BehaviorShootPlanner's destructor
 */
BehaviorShootPlanner::~BehaviorShootPlanner()
{
}

template < typename T > std::string to_string( const T& n ) {
    std::ostringstream stm ;
    stm << n ;
    return stm.str() ;
}
double getDistance(Vector player, Vector ball){
	double x = ball.X()-player.X();
	double y = ball.Y()-player.Y();
	return sqrt(x*x+y*y);
}

/**
 * Plan.
 * None or one ActiveBehavior will be push back to behavior_list.
 */
void BehaviorShootPlanner::Plan(list<ActiveBehavior> & behavior_list)
{
	if (!mSelfState.IsKickable()) return;


	PlayerState goalie = mWorldState.GetTeammate(mWorldState.GetTeammateGoalieUnum());
	if((goalie.GetPos().X() > 0 && mBallState.GetPos().X() >= 0)
		|(goalie.GetPos().X() < 0 && mBallState.GetPos().X() <= 0)) return;

	//AI
		// double power = 10;
		AngleDeg left= (ServerParam::instance().oppLeftGoalPost()- mSelfState.GetPos()).Dir()  ;
		AngleDeg right = (ServerParam::instance().oppRightGoalPost()- mSelfState.GetPos()).Dir();
		Vector target ;
		AngleDeg interval;
		Line c(ServerParam::instance().oppLeftGoalPost(),ServerParam::instance().oppRightGoalPost());
		AngleDeg shootDir = mPositionInfo.GetShootAngle(left,right,mSelfState ,interval);
		

		Ray f(mSelfState.GetPos(),shootDir);
		c.Intersection(f,target);
		ActiveBehavior shoot(mAgent, BT_Shoot);
				// //shoot.mTarget = target;
				//shoot.mAngle = 0;
				shoot.mEvaluation = 2.0 + FLOAT_EPS;
				shoot.mTarget = target;
		double power = Kicker::instance().GetOneKickPower(mWorldState.GetBall().GetVel(), shoot.mTarget, mSelfState.GetKickRate());	string args;
		double direction = (atan2 (shoot.mTarget.Y(),shoot.mTarget.X()) * 180 / 3.14159265) - mSelfState.GetBodyDir();
		args = "./RF.exe ";
		args += to_string(mWorldState.GetBall().GetPos().X()) + ' ' + to_string(mWorldState.GetBall().GetPos().Y());
		args += ' ' + to_string(mWorldState.GetBall().GetVel().X());
		args += ' '+to_string(power)+' '+to_string(mSelfState.GetPos().Y());
		double mates [11];
		for(int i = 1; i<= 11; i++){
			mates[i-1] = getDistance(mWorldState.GetTeammate(i).GetPos(), mWorldState.GetBall().GetPos());
		}
		sort(mates, mates+11);
		Vector ballPos = mWorldState.GetBall().GetPos();
		
		for(int j = 1; j <= 11; j++){
			PlayerState pl = mWorldState.GetTeammate(j);
			if(getDistance(pl.GetPos(), ballPos) == mates[1])
				args += ' '+to_string(pl.GetFocusOnUnum());
			if(getDistance(pl.GetPos(), ballPos) == mates[2])
				args += ' '+to_string(pl.GetFocusOnUnum());
			if(getDistance(pl.GetPos(), ballPos) == mates[3])
				args += ' '+to_string(pl.GetFocusOnUnum());
			if(getDistance(pl.GetPos(), ballPos) == mates[4])
				args += ' '+to_string(pl.GetFocusOnUnum());
			if(getDistance(pl.GetPos(), ballPos) == mates[6])
				args += ' '+to_string(pl.GetFocusOnUnum());
		}
		double goalx;
		if(mWorldState.GetTeammate(mWorldState.GetTeammateGoalieUnum()).GetPos().X() > 0)
			goalx = -54;
		else goalx = 54;
		double goaly1 = 7.02;
		double goaly2 = -7.02;
		Vector vec1(goalx-mSelfState.GetPos().X(), goaly1-mSelfState.GetPos().Y());
		Vector vec2(goalx-mSelfState.GetPos().X(), goaly2-mSelfState.GetPos().Y());
		double abs1 = sqrt(vec1.X()*vec1.X()+vec1.Y()*vec1.Y());
		double abs2 = sqrt(vec2.X()*vec2.X()+vec2.Y()*vec2.Y());
		double viewAngle = acos((vec1.X()*vec2.X()+vec1.Y()*vec2.Y())/(abs1*abs2));
		args += ' '+to_string(viewAngle);
		double goaliex = mWorldState.GetOpponent(mWorldState.GetOpponentGoalieUnum()).GetPos().X();
		double y1 = (goalx-mSelfState.GetPos().X()*tan((3.14159265/180)*(mSelfState.GetBodyDir()+direction)));
		double goaliey = mWorldState.GetOpponent(mWorldState.GetOpponentGoalieUnum()).GetPos().Y();
		Vector vect1(goalx-mSelfState.GetPos().X(), y1-mSelfState.GetPos().Y());
	    Vector vect2(goaliex-mSelfState.GetPos().X(), goaliey-mSelfState.GetPos().Y());
	    abs1 = sqrt(vect1.X()*vect1.X()+vect1.Y()*vect1.Y());
	    abs2 = sqrt(vect2.X()*vect2.X()+vect2.Y()*vect2.Y());
	    double kickAngle = acos((vect1.X()*vect2.X()+vect1.Y()*vect2.Y())/(abs1*abs2));
		args += ' '+to_string(kickAngle);
	    char res[100];
	    FILE *fp = popen(args.c_str(), "r");
	    fgets(res, 100, fp);
	    if(atof(res) == 1){
			behavior_list.push_back(shoot);
			// ofstream myfile;
			//   myfile.open ("shootsLog.txt", ios::app);
			 // cout << args << "Shoot player "<<mSelfState.GetUnum()<<"\n";
			//   myfile.close();

				return;
	    }
	//AI

	// if (mWorldState.GetPlayMode() == PM_Our_Foul_Charge_Kick ||
	// 		mWorldState.GetPlayMode() == PM_Our_Back_Pass_Kick ||
	// 		mWorldState.GetPlayMode() == PM_Our_Indirect_Free_Kick||
	// 		(mWorldState.GetLastPlayMode()==PM_Our_Indirect_Free_Kick && mAgent.IsLastActiveBehaviorInActOf(BT_Pass)))   //Indircet后传球保持不射门，至少跑位后上个动作会改
	// {
	// 	return;
	// }

	// if (mSelfState.GetPos().X() > ServerParam::instance().pitchRectanglar().Right() - PlayerParam::instance().shootMaxDistance()) {
	// 	AngleDeg left= (ServerParam::instance().oppLeftGoalPost()- mSelfState.GetPos()).Dir()  ;
	// 	AngleDeg right = (ServerParam::instance().oppRightGoalPost()- mSelfState.GetPos()).Dir();
	// 	Vector target ;
	// 	AngleDeg interval;
	// 	Line c(ServerParam::instance().oppLeftGoalPost(),ServerParam::instance().oppRightGoalPost());
	// 	AngleDeg shootDir = mPositionInfo.GetShootAngle(left,right,mSelfState ,interval);
	// 	if(interval < mSelfState.GetRandAngle(ServerParam::instance().maxPower(),ServerParam::instance().ballSpeedMax(),mBallState)*3){
	// 		return;
	// 	}

	// 	Ray f(mSelfState.GetPos(),shootDir);
	// 	c.Intersection(f,target);
	// 	// if(Tackler::instance().CanTackleToDir(mAgent, shootDir)
	// 	// 		&& Tackler::instance().GetBallVelAfterTackle(mAgent,shootDir).Mod() > ServerParam::instance().ballSpeedMax() -  0.05){
	// 		ActiveBehavior shoot(mAgent, BT_Shoot,BDT_Shoot_Tackle);
	// 		shoot.mTarget = target;
	// 		shoot.mAngle = shootDir;
	// 		shoot.mEvaluation = 2.0 + FLOAT_EPS;
	// 		behavior_list.push_back(shoot);
	// 	// }

	// 	// else {
	// 	// 	ActiveBehavior shoot(mAgent, BT_Shoot);
	// 	// 	shoot.mTarget = target;
	// 	// 	shoot.mEvaluation = 2.0 + FLOAT_EPS;

	// 	// 	behavior_list.push_back(shoot);
	// 	// }
	// }
}
