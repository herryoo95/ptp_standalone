#include "PTPSystem.hpp"


namespace soildynamics {

//PUBLIC
PTPSystem::PTPSystem(){
}
PTPSystem::~PTPSystem()
{
}
void PTPSystem::test_class(const vecD i){
printf("start...\n");
	sysTest = 0.1111;
printf("sysTest ... = %f\n", sysTest);
}

void PTPSystem::endPTP(){
	//	free(CELLP);
	//	free(PARTICLEP);
}
char* PTPSystem::printPTP(){
	static char buf[500];
	//sprintf_s(buf,sizeof(buf),"PTP: s(%.2f,%.2f,%.2f)mm e(%.2f,%.2f,%.2f)mm n(%d,%d,%d)\n", PTP_GLOBAL.start.x/MAG,PTP_GLOBAL.start.y/MAG,PTP_GLOBAL.start.z/MAG,PTP_GLOBAL.end.x/MAG,PTP_GLOBAL.end.y/MAG,PTP_GLOBAL.end.z/MAG,PTP_GLOBAL.n.x,PTP_GLOBAL.n.y,PTP_GLOBAL.n.z);
	snprintf(buf,sizeof(buf),"PTP: s(%.2f,%.2f,%.2f)mm e(%.2f,%.2f,%.2f)mm n(%d,%d,%d)\n", PTP_GLOBAL.start.x/MAG,PTP_GLOBAL.start.y/MAG,PTP_GLOBAL.start.z/MAG,PTP_GLOBAL.end.x/MAG,PTP_GLOBAL.end.y/MAG,PTP_GLOBAL.end.z/MAG,PTP_GLOBAL.n.x,PTP_GLOBAL.n.y,PTP_GLOBAL.n.z);

	return buf; 
}
char* PTPSystem::printCELL(const vecN n){
	static char buf[500];
	//sprintf_s(buf,sizeof(buf),"CELL_GLOBAL[%d][%d][%d] s(%.3f,%.3f,%.3f)mm e(%.3f,%.3f,%.3f)mm\n",n.x,n.y,n.z,CELL_GLOBAL[n.x][n.y][n.z].start.x/MAG,CELL_GLOBAL[n.x][n.y][n.z].start.y/MAG,CELL_GLOBAL[n.x][n.y][n.z].start.z/MAG,CELL_GLOBAL[n.x][n.y][n.z].end.x/MAG,CELL_GLOBAL[n.x][n.y][n.z].end.y/MAG,CELL_GLOBAL[n.x][n.y][n.z].end.z/MAG);
	snprintf(buf,sizeof(buf),"CELL_GLOBAL[%d][%d][%d] s(%.3f,%.3f,%.3f)mm e(%.3f,%.3f,%.3f)mm\n",n.x,n.y,n.z,CELL_GLOBAL[n.x][n.y][n.z].start.x/MAG,CELL_GLOBAL[n.x][n.y][n.z].start.y/MAG,CELL_GLOBAL[n.x][n.y][n.z].start.z/MAG,CELL_GLOBAL[n.x][n.y][n.z].end.x/MAG,CELL_GLOBAL[n.x][n.y][n.z].end.y/MAG,CELL_GLOBAL[n.x][n.y][n.z].end.z/MAG);

	return buf; 
}
char* PTPSystem::printPARTICLE(const vecN n){
	static char buf[500];
	//sprintf_s(buf,sizeof(buf),"PARTICLE_GLOBAL[%d][%d][%d]:(%.3f,%.3f,%.3f)mm\n",n.x,n.y,n.z,PARTICLE_GLOBAL[n.x][n.y][n.z].loc.x/MAG,PARTICLE_GLOBAL[n.x][n.y][n.z].loc.y/MAG,PARTICLE_GLOBAL[n.x][n.y][n.z].loc.z/MAG);
	snprintf(buf,sizeof(buf),"PARTICLE_GLOBAL[%d][%d][%d]:(%.3f,%.3f,%.3f)mm\n",n.x,n.y,n.z,PARTICLE_GLOBAL[n.x][n.y][n.z].loc.x/MAG,PARTICLE_GLOBAL[n.x][n.y][n.z].loc.y/MAG,PARTICLE_GLOBAL[n.x][n.y][n.z].loc.z/MAG);
	return buf; 
}

bool PTPSystem::getPTPForce(FOOT foot, vecD* force, vecD* forceT){
	static FOOT foot_last;
	static int last_flag=0;
	double big_rSQR;
	static vecD fT;
	static vecD f;

	//local_debug("(%f,%f,%f),(%f,%f,%f) \n",foot_last.loc.x,foot_last.loc.y,foot_last.loc.z,foot_last.vel.x,foot_last.vel.y,foot_last.vel.z);


	///ADAPT MAG
	foot.l = foot.l*MAG;
	foot.loc.x = foot.loc.x*MAG;
	foot.loc.y = foot.loc.y*MAG;
	foot.loc.z = foot.loc.z*MAG;

	foot.r = foot.r*MAG;
	foot.vel.x = foot.vel.x*MAG;
	foot.vel.y = foot.vel.y*MAG;
	foot.vel.z = foot.vel.z*MAG;


	if(last_flag && (!FLOAT_EQ(foot.r,foot_last.r) || !FLOAT_EQ(foot.type,foot_last.type) || !FLOAT_EQ(foot.l,foot_last.l))){
		local_debug("INPUT ERROR\n");		
		foot_last = foot;
		return false;
	}else if(last_flag && foot.loc.x==foot_last.loc.x && 
		foot.loc.y==foot_last.loc.y &&
		foot.loc.z==foot_last.loc.z &&
		foot.vel.x==foot_last.vel.x &&
		foot.vel.y==foot_last.vel.y &&
		foot.vel.z==foot_last.vel.z){

			local_debug("SAME FOOT INPUT\n");
			*forceT=fT;
			*force=f;
			return true; 

	}else{
		ptpWorks(&foot,&foot_last);
//	int ptpWorks(const FOOT* foot,const FOOT* lastFoot, const PTP* ptp, PTPUtil* ut);
		if(foot.type==SPHERE){
			big_rSQR = foot.r*foot.r;
		}else if(foot.type==CYLINDER){
			big_rSQR = foot.r*foot.r+foot.l*foot.l;
		}

		if(foot.type==SPHERE){

			if(last_flag){
				if(PTP_UTIL.getLengthSQR(&foot.loc,&foot_last.loc,ALL)>big_rSQR){
					calcForce(&foot, FINE,&f,&fT);	
				}else{			
#ifdef FAST_CALC
					calcForce(&foot, FAST,&f,&fT);	
#endif
#ifndef FAST_CALC
					calcForce(&foot, FINE,&f,&fT);	
#endif
				}
			}else{	
				calcForce(&foot, FINE,&f,&fT);	
			}	


		}else if(foot.type==CYLINDER){

			if(last_flag){
				if(PTP_UTIL.getLengthSQR(&foot.loc,&foot_last.loc,ALL)>big_rSQR){
					calcForce(&foot,FINE,&f,&fT);	
				}else{			
#ifdef FAST_CALC
					calcForce(&foot, FAST,&f,&fT);		
#else
					calcForce(&foot, FINE,&f,&fT);
#endif
				}
			}else{	
				calcForce(&foot, FINE,&f,&fT);		
			}	

		}

		foot_last = foot;
		last_flag=1;
		forceT->x = fT.x;
		forceT->y = fT.y;
		forceT->z = fT.z;
		force->x = f.x;
		force->y = f.y;
		force->z = f.z;
		return true;
	}

}
 
} //end of namespace soildynamics
