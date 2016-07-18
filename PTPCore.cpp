/**************************************************************************
DFKI
PTPCore.cpp
Auther: Dr. -Ing. Yong-Ho Yoo
***************************************************************************/

#include "PTPCore.hpp"

namespace soildynamics {

PTPCore::PTPCore(){
	isPTPInitialized = false;
//	PARTICLE_GLOBAL = new PARTICLE[MAX_X][MAX_Y][MAX_Z];
//	CELL_GLOBAL = new CELL[MAX_X][MAX_Y][MAX_Z];
//	filenameCALC = new	char[150];
	
}
PTPCore::~PTPCore()
{
	/*
	delete[] PROJ_Y_M;
	delete[] PROJ_Y_P;
	delete[] PROJ_X_M;
	delete[] PROJ_X_P;
	delete[] PROJ_Z_M;
	delete[] PROJ_Z_P;
	
	delete[] PARTICLE_GLOBAL;
	delete[] CELL_GLOBAL;
	delete[] filenameCALC;
	*/
}

int PTPCore::getSumOfParticlesNum(){
	return	PTP_GLOBAL.n.x*PTP_GLOBAL.n.y*PTP_GLOBAL.n.z;
}



vecD PTPCore::getOriginalParticlePosition(const vecN n){
	vecD r;
	vecD d=PTP_GLOBAL.cellScale;

	r.x = PTP_GLOBAL.start.x + (n.x)*d.x;
	r.y = PTP_GLOBAL.end.y - (n.y)*d.y;
	r.z = PTP_GLOBAL.start.z + (n.z)*d.z;
	return r;
}

vecD PTPCore::getFootAABBBound(const FOOT* foot, const int type){
	vecD start;
	vecD end;


	switch(foot->type){
	case CYLINDER:
		start.x = foot->loc.x - foot->r;
		start.z = foot->loc.z - foot->r;
		start.y = foot->loc.y - foot->l*0.5f;

		end.x = foot->loc.x + foot->r;
		end.z = foot->loc.z + foot->r;
		end.y = foot->loc.y + foot->l*0.5f;

		break;
	case SPHERE:
		start.x = foot->loc.x - foot->r;
		start.z = foot->loc.z - foot->r;
		start.y = foot->loc.y - foot->r;

		end.x = foot->loc.x + foot->r;
		end.z = foot->loc.z + foot->r;
		end.y = foot->loc.y + foot->r;


		break;
	}
#ifdef USE_COLLISION_DEBUG
	DEBUG_GLOBAL.boundPointEnd = start;
	DEBUG_GLOBAL.boundPointStart = end;
#endif
	switch(type){
	case END:
		return end;
		break;
	case START:
		return start;
		break;
	default:
		return start;
		break;

	}
}

vecN PTPCore::judgeWhichCell(const vecD* loc){
		vecD d=PTP_GLOBAL.cellScale;
		vecN r;
		vecD temp;
		temp.x = (loc->x-CELL_GLOBAL[0][PTP_GLOBAL.n.y-1][0].start.x);
		temp.y = (loc->y-CELL_GLOBAL[0][PTP_GLOBAL.n.y-1][0].start.y);
		temp.z = (loc->z-CELL_GLOBAL[0][PTP_GLOBAL.n.y-1][0].start.z);
		//	local_debug("Point(%f,%f,%f)\n",temp.x,temp.y,temp.z);


		temp.x = (temp.x/d.x);
		temp.y = PTP_GLOBAL.n.y - (temp.y/d.y);
		temp.z = (temp.z/d.z);

		r.x = (int)temp.x;
		r.y = (int)temp.y;
		r.z = (int)temp.z;

		//	if(temp.x>0) r.x = (int)(temp.x-EPSILON); else r.x = (int)(temp.x+EPSILON);
		//	if(temp.y>0) r.y = (int)(temp.y-EPSILON); else r.y = (int)(temp.y+EPSILON);
		//	if(temp.z>0) r.z = (int)(temp.z-EPSILON); else r.z = (int)(temp.z+EPSILON);

		//	local_debug("Point(%f,%f,%f)\n",temp.x,temp.y,temp.z);
		//	local_debug("JUDGED %f %f %f \n",temp.x,temp.y,temp.z);

		return r;
}


vecD PTPCore::getCellSize(){
		vecD discreteScale;

		vecD maxVec=PTP_UTIL.getMaxVec(&PTP_GLOBAL.end,&PTP_GLOBAL.start);
		vecD minVec=PTP_UTIL.getMinVec(&PTP_GLOBAL.end,&PTP_GLOBAL.start);

		if(PTP_GLOBAL.n.x-1==0){
			discreteScale.x=(maxVec.x-minVec.x);
		}else{
			discreteScale.x = (maxVec.x-minVec.x)/(PTP_GLOBAL.n.x-1);
		}
		if(PTP_GLOBAL.n.y-1==0){

			discreteScale.y = (maxVec.y-minVec.y);
			//	printf("...discreteScale.y = %f",discreteScale.y);
						
		}else{
			discreteScale.y = (maxVec.y-minVec.y)/(PTP_GLOBAL.n.y-1);
		}

		if(PTP_GLOBAL.n.z-1==0){

			discreteScale.z = (maxVec.z-minVec.z);
		}else{
			discreteScale.z = (maxVec.z-minVec.z)/(PTP_GLOBAL.n.z-1);

		}
	//printf("...discreteScale [%f, %f, %f]\n",discreteScale.x, discreteScale.y, discreteScale.z);
		return discreteScale;									///RETURN VAL[mm*MAG]
}


double PTPCore::getArea(const CELL* cell,const int direction){
		vecD cellSize = PTP_UTIL.minusVecD(&cell->end,&cell->start);	/// CELLSIZE[mm*MAG]
		double r=0;

		///cellSize[mm*MAG]
		switch (direction){
		case Z_AXIS: //x-y
			r=cellSize.x*cellSize.y;
			break;
		case Y_AXIS: //x-z
			r=cellSize.x*cellSize.z;
			break;
		case X_AXIS: //y-z				
			r=cellSize.y*cellSize.z;
			break;
		default:
			break;
		}
		//r *= MM2M*MM2M;
		///RETURN VAL[mm^2*MAG^2]
		return r;
}



double PTPCore::getVolume(const CELL* cell){
		vecD cellSize = PTP_UTIL.minusVecD(&cell->end,&cell->start);	/// CELLSIZE[mm*MAG]
		double r=0;

		///cellSize[mm*MAG]

		r = cellSize.x*cellSize.y*cellSize.z;
		///RETURN VAL[mm^3*MAG^3]
		return r;
}

double PTPCore::getLength(const CELL* cell, const int type){
		vecD cellSize = PTP_UTIL.minusVecD(&cell->end,&cell->start);	/// CELLSIZE[mm*MAG]
		double r=0;
		switch(type){
		case X_AXIS:
			r = cellSize.x;
			break;
		case Y_AXIS:
			r = cellSize.y;
			break;
		case Z_AXIS:
			r = cellSize.z;
			break;
		}
		///RETURN VAL[mm*MAG]
		return r;
}

#ifdef USE_SOIL_VISUALIZATION
vecD PTPCore::getOriginalSoilPosition(const vecN n){
		vecD r;
		vecD d;

		vecD temp;
		temp = PTP_UTIL.minusVecD(&PTP_GLOBAL.end,&PTP_GLOBAL.start);
		d.x = temp.x/PTP_GLOBAL.n.x;
		d.y = temp.y/PTP_GLOBAL.n.y;
		d.z = temp.z/PTP_GLOBAL.n.z;

		r.x = PTP_GLOBAL.start.x + (n.x)*d.x;
		r.y = PTP_GLOBAL.start.y + (n.y)*d.y;
		r.z = PTP_GLOBAL.start.z + (n.z)*d.z;

		if(n.x==0) r.x = PTP_GLOBAL.start.x;
		if(n.y==0) r.y = PTP_GLOBAL.start.y;
		if(n.z==0) r.z = PTP_GLOBAL.start.z;
		if(n.x==PTP_GLOBAL.n.x+1) r.x = PTP_GLOBAL.end.x;
		if(n.y==PTP_GLOBAL.n.y+1) r.x = PTP_GLOBAL.end.y;
		if(n.z==PTP_GLOBAL.n.z+1) r.x = PTP_GLOBAL.end.z;

		//local_debug("PARTICLE_GLOBAL[%d][%d][%d]:(%f,%f,%f)\n",n.x,n.y,n.z,r.x,r.y,r.z);

		return r;
}
#endif



#ifdef FAST_CALC
vecN PTPCore::getComputeNeededCellNum(const FOOT* foot, const int type){
		vecN loopStartN=PTP_UTIL.setVecN(0,0,0);
		vecN loopEndN=PTP_UTIL.setVecN(0,0,0);
		vecN bigN = PTP_UTIL.setVecN(0,0,0);
		vecN smallN = PTP_UTIL.setVecN(0,0,0);

		vecN rN=PTP_UTIL.setVecN(0,0,0);

		vecD loopStart= 	getFootAABBBound(foot,START);
		vecD loopEnd =		getFootAABBBound(foot,END);

		//	loopStart.x -= EPSILON;
		//	loopStart.y -= EPSILON;
		//	loopStart.z -= EPSILON;

		//	loopEnd.x += EPSILON;
		//	loopEnd.y += EPSILON;
		//	loopEnd.z += EPSILON;


		loopStartN = judgeWhichCell(&loopStart);
		loopEndN = judgeWhichCell(&loopEnd);

		bigN=PTP_UTIL.getMaxVecN(&loopStartN,&loopEndN);
		smallN=PTP_UTIL.getMinVecN(&loopStartN,&loopEndN);

		loopStartN = smallN;
		loopEndN = bigN;

		loopEndN.x++;
		loopEndN.y++;
		loopEndN.z++;

		if(loopStartN.x<0) loopStartN.x = 0;
		if(loopStartN.y<0) loopStartN.y = 0;
		if(loopStartN.z<0) loopStartN.z = 0;
		if(loopStartN.x>PTP_GLOBAL.n.x) loopStartN.x = PTP_GLOBAL.n.x;
		if(loopStartN.y>PTP_GLOBAL.n.y) loopStartN.y = PTP_GLOBAL.n.y;
		if(loopStartN.z>PTP_GLOBAL.n.z) loopStartN.z = PTP_GLOBAL.n.z;

		if(loopEndN.x<0) loopEndN.x = 0;
		if(loopEndN.y<0) loopEndN.y = 0;
		if(loopEndN.z<0) loopEndN.z = 0;
		if(loopEndN.x>PTP_GLOBAL.n.x) loopEndN.x = PTP_GLOBAL.n.x;
		if(loopEndN.y>PTP_GLOBAL.n.y) loopEndN.y = PTP_GLOBAL.n.y;
		if(loopEndN.z>PTP_GLOBAL.n.z) loopEndN.z = PTP_GLOBAL.n.z;


		switch(type){
		case START:
		case PTP_START:
			rN = loopStartN;
			break;
		case END:
		case PTP_END:
			rN = loopEndN;
			break;
		default:
			local_trace("error");

			break;

		}
		return rN;
}
#endif

double PTPCore::radianShifter(const double* inp, const double* standard){
		return PTP_UTIL.radianMapper(inp)-PTP_UTIL.radianMapper(standard);
}

double PTPCore::radianRestore(const double* inp, const double* standard){
		double r= PTP_UTIL.radianMapper(inp)+PTP_UTIL.radianMapper(standard);
		return PTP_UTIL.radianMapper(&r);
}


double PTPCore::radianShrink(const double* inp){
		double process = PTP_UTIL.radianMapper(inp);

		if(process<PI){
			process /= 2.0;
		}else if(process==PI){
			process = PI/2.0;
		}else if(process>PI){
			process /= 2.0;
			process += PI;
		}else if(process==2*PI){
			process = PI;
		}else{
			local_debug("ERROR");
		}


		return process;
}
	
int PTPCore::initializePTP(const vecN *n,const vecD *s,const vecD *e){	

	register int i,j,k;		
	PTP_GLOBAL.start = PTP_UTIL.setVecD( MAG*s->x, MAG*s->y, MAG*s->z);    /** *soil surface size = start*end */
	PTP_GLOBAL.end = PTP_UTIL.setVecD(MAG*e->x,MAG*e->y,MAG*e->z);
	PTP_GLOBAL.n = PTP_UTIL.setVecN(n->x,n->y,n->z);
	PTP_GLOBAL.cellScale = getCellSize();
	

/** *build cells*/
	for(i=0;i<n->x;i++){
		for(j=0;j<n->y;j++){
			for(k=0;k<n->z;k++){
				CELL_GLOBAL[i][j][k].start = PTP_UTIL.setVecD(
					PTP_GLOBAL.start.x-(PTP_GLOBAL.cellScale.x*0.5f)+ (i*PTP_GLOBAL.cellScale.x), 
					//PTP_GLOBAL.end.y - (PTP_GLOBAL.cellScale.y), 
					PTP_GLOBAL.end.y-(PTP_GLOBAL.cellScale.y*0.5f)- (j*PTP_GLOBAL.cellScale.y),
					PTP_GLOBAL.start.z-(PTP_GLOBAL.cellScale.z*0.5f)+ (k*PTP_GLOBAL.cellScale.z)
					);
				CELL_GLOBAL[i][j][k].end = PTP_UTIL.setVecD(
					PTP_GLOBAL.start.x+(PTP_GLOBAL.cellScale.x*0.5f)+((i)*PTP_GLOBAL.cellScale.x),
					//PTP_GLOBAL.end.y -((j)*PTP_GLOBAL.cellScale.y),
					PTP_GLOBAL.end.y +(PTP_GLOBAL.cellScale.y*0.5f)-((j)*PTP_GLOBAL.cellScale.y),
					PTP_GLOBAL.start.z+(PTP_GLOBAL.cellScale.z*0.5f)+((k)*PTP_GLOBAL.cellScale.z)
					);
				CELL_GLOBAL[i][j][k].center = PTP_UTIL.getMiddlePoint(&CELL_GLOBAL[i][j][k].start,&CELL_GLOBAL[i][j][k].end);

				CELL_GLOBAL[i][j][k].CollisionParticleNum =0;
				CELL_GLOBAL[i][j][k].particleNum = 1;
			
				CELL_GLOBAL[i][j][k].averageLoc = PTP_UTIL.setVecD(0,0,0);
			}
		}
	}

/** build particles*/
	for(i=0;i<n->x;i++){
		for(j=0;j<n->y;j++){
			for(k=0;k<n->z;k++){
#ifdef USE_PROJ_ARRAY
				PROJ_Y_M[i][k] = PTP_GLOBAL.n.y-1;
				PROJ_Y_P[i][k] = 0;
				PROJ_X_M[j][k] = 0;
				PROJ_X_P[j][k] = PTP_GLOBAL.n.x-1;
				PROJ_Z_M[i][j] = 0;
				PROJ_Z_P[i][j] = PTP_GLOBAL.n.z-1;
#endif

				PARTICLE_GLOBAL[i][j][k].loc = getOriginalParticlePosition(PTP_UTIL.setVecN(i,j,k));
				PARTICLE_GLOBAL[i][j][k].cloc = PTP_UTIL.setVecN(i,j,k);
				
				//printf("[%d, %d,%d,] <%f,%f,%f>\n" , i,j,k,PARTICLE_GLOBAL[i][j][k].loc.x, 
				//PARTICLE_GLOBAL[i][j][k].loc.y, PARTICLE_GLOBAL[i][j][k].loc.z);
				
					judgeWhichCell(&PARTICLE_GLOBAL[i][j][k].loc);
			}
		} 
	}
#ifdef USE_SOIL_VISUALIZATION
	/// For Visualization
	for(i=0;i<n->x+1;i++){
		for(j=0;j<n->y+1;j++){
			for(k=0;k<n->z+1;k++){
				SOIL_GLOBAL[i][j][k].loc = getOriginalSoilPosition(setVecN(i,j,k));
			}
		}
	}
#endif

#if (defined USE_VOLUME_DEBUG) || (defined USE_AREA_DEBUG)
	snprintf(filenameCALC,sizeof(filenameCALC),"result_VOLUME.csv");	
	outCALC = fopen(filenameCALC, "w");
	//fprintf(outCALC,"foot_loc_x,foot_loc_y,foot_loc_z,volume,areaX,areaY,areaZ\n");
	fprintf(outCALC,"averageLoc\n");
	fclose(outCALC); 

#endif
	isPTPInitialized = true;
	return 1;
}
vecD PTPCore::particleMovement(const vecD* point,const FOOT* lastFoot){
	vecD E = PTP_UTIL.setVecD(0.0,0.0,0.0);
	vecD R= PTP_UTIL.setVecD(0.0,0.0,0.0);
	vecD movement = PTP_UTIL.normalizeVector(&lastFoot->vel);

	double Rs=0.0;
	R.x = ((point->x/MAG)-lastFoot->loc.x);
#ifdef USE_MOVEMENT_TOP
	R.y = ((point->y/MAG)-(PTP_GLOBAL.end.y+(lastFoot->r/2)));
#else
	R.y = ((point->y/MAG)-(lastFoot->loc.y));
#endif
	R.z = ((point->z/MAG)-lastFoot->loc.z);
	Rs = sqrt(PTP_UTIL.getVecDSizeSQR(&R,ALL));
	E = PTP_UTIL.setVecD( (1./(Rs*Rs*Rs)),  (1./(Rs*Rs*Rs)),  (1./(Rs*Rs*Rs)));
	E = PTP_UTIL.multipleVecD(&E,&R);
	E = PTP_UTIL.normalizeVector(&E);

#ifdef USE_GRAVITY
	E.y -= GRAVITY;		///FIXED
#endif

	movement.x *= PANEL_ZX;
	movement.y *= 0;
	movement.z *= PANEL_ZX;
	//	if(lastFoot->vel.x>0) {Sx = -PANEL_Y; }else if(lastFoot->vel.x<0){ Sx = PANEL_Y; }
	//	if(lastFoot->vel.y>0) {Sy = 0; }
	//	if(lastFoot->vel.z>0) {Sz = -PANEL_Y; }else if(lastFoot->vel.z<0){ Sz = PANEL_Y; }
	E.x -=movement.x;
	E.z -=movement.z;

	//zE = normalizeVector(&E);


	return E;

}
#ifdef USE_SOIL_VISUALIZATION
vecD PTPCore::soilMovement(const vecD* point,const FOOT* foot){
	vecD E = PTP_UTIL.setVecD(0,0,0);
	vecD R= PTP_UTIL.setVecD(0,0,0);

	double Q=0, Q2=0,Sx=0, Sy=0, Sz=0, Rs=0, R2s=0;


	Q = POINT_Q;				//������ ����..
	Q2 = 0;					//�� ���� �Ķ����
	Sy = -Q*1e-5;			//������ Y �̰��� �Ķ����..


	R.x = ((point->x/MAG)-foot->loc.x);
	R.y = ((point->y/MAG)-foot->loc.y+(foot->r ));
	R.z = ((point->z/MAG)-foot->loc.z);
	Rs = sqrt(PTP_UTIL.getVecDSizeSQR(R,ALL));


	//������(���� ����)(����� ����)
	E.x = (Q/(4*PI*Rs*Rs*Rs))*R.x;	
	E.y = (Q/(4*PI*Rs*Rs*Rs))*R.y;
	E.z = (Q/(4*PI*Rs*Rs*Rs))*R.z;

	//�����������(�߷��� ����) 
	///������ �����϶� ������..
	if(foot->vel.x>0) {Sx = -foot->vel.x*SOIL_PANEL_ZX; }else if(foot->vel.x<0){ Sx = foot->vel.x*SOIL_PANEL_ZX; }
	if(foot->vel.z>0) {Sz = -foot->vel.x*SOIL_PANEL_ZX; }else if(foot->vel.z<0){ Sz = foot->vel.x*SOIL_PANEL_ZX; }
	E.x +=(Sx/2);	
	E.y +=(Sy/2);
	E.z +=(Sz/2);

	return E;
}
#endif

bool PTPCore::AABBCollision(CELL* pAABB1, CELL* pAABB2)
{
	if(FLOAT_EQ(pAABB1->end.x,pAABB2->start.x))
		return true;
	if(FLOAT_EQ(pAABB1->end.y,pAABB2->start.y))
		return true;
	if(FLOAT_EQ(pAABB1->end.z,pAABB2->start.z))
		return true;
	if( pAABB1->end.x < pAABB2->start.x || 
		pAABB1->start.x > pAABB2->end.x )
		return false;
	if( pAABB1->end.y < pAABB2->start.y ||
		pAABB1->start.y > pAABB2->end.y )
		return false;
	if( pAABB1->end.z < pAABB2->start.z ||
		pAABB1->start.z > pAABB2->end.z )
		return false;


	return true;
}

int PTPCore::particleWorks(const vecN* particleN,const FOOT* foot, const FOOT* lastFoot, const int pLocData){		///PARTICLE
	double a=0.0, b=0.0, c=0.0;
	double k=0.0;

	PARTICLE* pp = &PARTICLE_GLOBAL[particleN->x][particleN->y][particleN->z];
	vecN lastCLoc = pp->cloc;
	vecN tempCloc = PTP_UTIL.setVecN(-1,-1,-1);
	vecD tempLoc = PTP_UTIL.setVecD(0.f,0.f,0.f);
	vecD ppDirection = PTP_UTIL.setVecD(0.f,0.f,0.f);
	vecD contactDirection = PTP_UTIL.setVecD(0.f,0.f,0.f);
	vecD GDirection = PTP_UTIL.setVecD(0.f,-1.f,0.f);

//	int collisionR=NOT_COLLISION;

	vecD NfV;

#ifdef USE_MOVEMENT_FUNCTION
	NfV = particleMovement(&pp->loc,lastFoot);
//	NfV =PTP_UTIL.setVecD(10.f,0.f,10.f);  //!!
	NfV = PTP_UTIL.normalizeVector(&NfV); //normalizeVector(foot->vel);
	

			ppDirection = PTP_UTIL.minusVecD(&pp->loc,&foot->loc);
			ppDirection = PTP_UTIL.normalizeVector(&ppDirection);
			contactDirection = PTP_UTIL.normalizeVector(&foot->vel);
		//	contactDirection = PTP_UTIL.plusVecD(&contactDirection, &GDirection);
		//	contactDirection = PTP_UTIL.normalizeVector(&contactDirection);
			double cosTheta, collReact;
			cosTheta = PTP_UTIL.getCosTheta(&ppDirection,&contactDirection);
			
//	printf("(%f %f)..(%f %f %f)..(%f, %f, %f)\n", contactDirection.y,cosTheta, pp->loc.x, pp->loc.y, pp->loc.z,foot->vel.x, foot->vel.y, foot->vel.z);		
	
#endif
#ifndef USE_MOVEMENT_FUNCTION
	NfV = PTP_UTIL.normalizeVector(&foot->vel);
#endif
	switch(foot->type){
	case SPHERE:			
	//	a = (NfV.x*NfV.x)+(NfV.y*NfV.y)+(NfV.z*NfV.z);
		a = 1;//(NfV.x*NfV.x)+(NfV.y*NfV.y)+(NfV.z*NfV.z);
		b = 2.0*((pp->loc.x*NfV.x)+(pp->loc.y*NfV.y)+(pp->loc.z*NfV.z)-((foot->loc.x*NfV.x)+(foot->loc.y*NfV.y)+(foot->loc.z*NfV.z)));
		c = (SQR(pp->loc.x)+SQR(foot->loc.x)-(2*pp->loc.x*foot->loc.x))+(SQR(pp->loc.y)+SQR(foot->loc.y)-(2*pp->loc.y*foot->loc.y))+SQR(pp->loc.z)+SQR(foot->loc.z)-(2*pp->loc.z*foot->loc.z)-SQR(foot->r);
		k = PTP_UTIL.quad_eqn(a,b,c);
		
		collReact = SPREAD*(1-cosTheta); 
	//	collReact = 1;
		
		tempLoc.x = pp->loc.x + (k*NfV.x)*collReact;  /** tempLoc is new location vector of particle, but still not updated*/
		tempLoc.y = pp->loc.y + (k*NfV.y)*collReact;  /**..............for test..20141218  */
		tempLoc.z = pp->loc.z + (k*NfV.z)*collReact;  

		break;
	case CYLINDER:
		a = 1;//(NfV.x*NfV.x)+(NfV.y*NfV.y)+(NfV.z*NfV.z);
		b = 2*((pp->loc.x*NfV.x)+(pp->loc.z*NfV.z)-((foot->loc.x*NfV.x)+(foot->loc.z*NfV.z)));
		c = (SQR(pp->loc.x)+SQR(foot->loc.x)-(2*pp->loc.x*foot->loc.x))+SQR(pp->loc.z)+SQR(foot->loc.z)-(2*pp->loc.z*foot->loc.z)-SQR(foot->r);
		k = PTP_UTIL.quad_eqn(a,b,c);
		tempLoc.x = pp->loc.x;   
		tempLoc.y = pp->loc.y;
		tempLoc.z = pp->loc.z;


		if(foot->vel.y>0 && (pLocData==IS_ON_UPPER ||pLocData==IS_IN)){
			tempLoc.y = foot->loc.y + ((foot->l)/2);
		}else if(foot->vel.y<0 && (pLocData==IS_ON_UPPER ||pLocData==IS_IN)){
			tempLoc.y = foot->loc.y - ((foot->l)/2);
		}else if((!FLOAT_EQ(foot->vel.x,0) || !FLOAT_EQ(foot->vel.z,0)) && (pLocData==IS_ON_SIDE ||  pLocData==IS_IN)){

			tempLoc.x = pp->loc.x + (k*NfV.x);
			tempLoc.z = pp->loc.z + (k*NfV.z);
		}
		break;
	case CUBE:
		//TODO:���ǵ� �����

		if(foot->vel.y<0 && (pLocData==IS_IN  || pLocData==IS_ON_UNDER)){
			tempLoc.y = foot->loc.y + foot->r;
		}
	default:

		local_debug("ERROR- FOOT SHAPE");
	}
	if((lastCLoc.x >=0 && lastCLoc.x < PTP_GLOBAL.n.x) &&
		(lastCLoc.y >=0 && lastCLoc.y < PTP_GLOBAL.n.y) &&
		(lastCLoc.z >=0 && lastCLoc.z < PTP_GLOBAL.n.z) ){

			//local_debug("P %d,%d,%d : %d \n",lastCLoc.x,lastCLoc.y,lastCLoc.z,CELL_GLOBAL[lastCLoc.x][lastCLoc.y][lastCLoc.z].particleNum); 
			CELL_GLOBAL[lastCLoc.x][lastCLoc.y][lastCLoc.z].particleNum--;
			/*
			if(CELL_GLOBAL[lastCLoc.x][lastCLoc.y][lastCLoc.z].particleNum==0){
			CELL_GLOBAL[lastCLoc.x][lastCLoc.y][lastCLoc.z].CollisionType=NOT_COLLISION;
			}
			*/
			tempCloc = judgeWhichCell(&tempLoc);   /** find cell ID with collision of Nth particle 'tempLoc'  */
			//local_debug("(%d,%d,%d),[%d,%d,%d]\n",particleN->x,particleN->y,particleN->z,tempCloc.x,tempCloc.y,tempCloc.z);
			if((tempCloc.x>=0 &&tempCloc.x<PTP_GLOBAL.n.x) &&
				(tempCloc.y>=0 && tempCloc.y<PTP_GLOBAL.n.y) &&
				(tempCloc.z>=0 && tempCloc.z<PTP_GLOBAL.n.z) ){
					CELL_GLOBAL[tempCloc.x][tempCloc.y][tempCloc.z].particleNum++;
					if(CELL_GLOBAL[tempCloc.x][tempCloc.y][tempCloc.z].particleNum == 1)    /// particleNum must be higher than 0 because of collision
					{
						CELL_GLOBAL[tempCloc.x][tempCloc.y][tempCloc.z].averageLoc = tempLoc;  //
					} 
					else if(CELL_GLOBAL[tempCloc.x][tempCloc.y][tempCloc.z].particleNum == 2) 
					{
						CELL_GLOBAL[tempCloc.x][tempCloc.y][tempCloc.z].averageLoc = PTP_UTIL.getMiddlePoint(&pp->loc,&tempLoc);  
						
					}
					else if(CELL_GLOBAL[tempCloc.x][tempCloc.y][tempCloc.z].particleNum > 2)
					{
						CELL_GLOBAL[tempCloc.x][tempCloc.y][tempCloc.z].averageLoc = 
						PTP_UTIL.getMiddlePoint(&CELL_GLOBAL[tempCloc.x][tempCloc.y][tempCloc.z].averageLoc,&tempLoc);
					}
     				//local_debug("(%d,%d,%d),[%d,%d,%d], {%d, %f %f}\n",particleN->x,particleN->y,particleN->z,tempCloc.x,tempCloc.y,tempCloc.z,
     				//         CELL_GLOBAL[tempCloc.x][tempCloc.y][tempCloc.z].particleNum, CELL_GLOBAL[tempCloc.x][tempCloc.y][tempCloc.z].averageLoc.y, tempLoc.y);
   	        	     				
					CELL_GLOBAL[tempCloc.x][tempCloc.y][tempCloc.z].CollisionParticleNum++;

					/*
					if(lastCLoc.x<tempCloc.x){
					collisionR |= COLLISION_X_M;
					}else if(lastCLoc.x>tempCloc.x){
					collisionR |= COLLISION_X_P;
					}
					if(lastCLoc.y<tempCloc.y){
					collisionR |= COLLISION_Y_M;
					}else if(lastCLoc.y>tempCloc.y){
					collisionR |= COLLISION_Y_P;
					}
					if(lastCLoc.z<tempCloc.z){
					collisionR |= COLLISION_Z_M;
					}else if(lastCLoc.z>tempCloc.z){
					collisionR |= COLLISION_Z_P;
					}
					//if(foot->vel.x>0) collisionR=collisionR|COLLISION_CASE_1;
					//if(foot->vel.x<0) collisionR=collisionR|COLLISION_CASE_2;
					//if(foot->vel.y>0) collisionR=collisionR|COLLISION_CASE_3;
					//if(foot->vel.y<0) collisionR=collisionR|COLLISION_CASE_4;
					//if(foot->vel.z>0) collisionR=collisionR|COLLISION_CASE_5;
					//if(foot->vel.z<0) collisionR=collisionR|COLLISION_CASE_6;
					CELL_GLOBAL[tempCloc.x][tempCloc.y][tempCloc.z].CollisionType = collisionR;
					*/
			}
	}

	pp->cloc =tempCloc;      /** particle ID/location will be updated */
	pp->loc = tempLoc;

	if(!FLOAT_EQ(lastCLoc.x,tempCloc.x)|| 
		!FLOAT_EQ(lastCLoc.y,tempCloc.y)||
		!FLOAT_EQ(lastCLoc.z,tempCloc.z)
		){
			return COLLISION;
	}else{
		return NOT_COLLISION;
	}

}
#ifdef USE_SOIL_VISUALIZATION
int PTPCore::soilWorks(const vecN* soilN,const FOOT* foot,const FOOT* lastFoot, const int pLocData){		///soil
	////////////////�����丵 �ʿ� 
	static double a, b, c;
	static double k;
	static vecD testPoint;
	SOIL* sp = &SOIL_GLOBAL[soilN->x][soilN->y][soilN->z];
	vecD NfV = normalizeVector(soilMovement(&sp->loc,lastFoot));

	if(foot->type==SPHERE){
		a = 1;//(NfV.x*NfV.x)+(NfV.y*NfV.y)+(NfV.z*NfV.z);
		b = 2*((sp->loc.x*NfV.x)+(sp->loc.y*NfV.y)+(sp->loc.z*NfV.z)-((foot->loc.x*NfV.x)+(foot->loc.y*NfV.y)+(foot->loc.z*NfV.z)));
		c = (SQR(sp->loc.x)+SQR(foot->loc.x)-(2*sp->loc.x*foot->loc.x))+(SQR(sp->loc.y)+SQR(foot->loc.y)-(2*sp->loc.y*foot->loc.y))+SQR(sp->loc.z)+SQR(foot->loc.z)-(2*sp->loc.z*foot->loc.z)-SQR(foot->r);
		k = quad_eqn(a,b,c);

		testPoint.x = sp->loc.x + (k*NfV.x);
		testPoint.y = sp->loc.y + (k*NfV.y);
		testPoint.z = sp->loc.z + (k*NfV.z);

		if(PTP_UTIL.getLengthSQR(sp->loc,testPoint,ALL)<SQR(foot->r))	{			
			sp->loc.x += (k*NfV.x);
			sp->loc.y += (k*NfV.y);
			sp->loc.z += (k*NfV.z);
		}else{
			NfV.x = -NfV.x;
			NfV.y = -NfV.y;
			NfV.z = -NfV.z;
			a = 1;//(NfV.x*NfV.x)+(NfV.y*NfV.y)+(NfV.z*NfV.z);
			b = 2*((sp->loc.x*NfV.x)+(sp->loc.y*NfV.y)+(sp->loc.z*NfV.z)-((foot->loc.x*NfV.x)+(foot->loc.y*NfV.y)+(foot->loc.z*NfV.z)));
			c = (SQR(sp->loc.x)+SQR(foot->loc.x)-(2*sp->loc.x*foot->loc.x))+(SQR(sp->loc.y)+SQR(foot->loc.y)-(2*sp->loc.y*foot->loc.y))+SQR(sp->loc.z)+SQR(foot->loc.z)-(2*sp->loc.z*foot->loc.z)-SQR(foot->r);
			k = quad_eqn(a,b,c);
			sp->loc.x += (k*NfV.x)*foot->vel.x*SOIL_DEGREE;
			sp->loc.y += (k*NfV.y)*foot->vel.y*SOIL_DEGREE_Y;
			sp->loc.z += (k*NfV.z)*foot->vel.z*SOIL_DEGREE;

		}

	}else if(foot->type==CYLINDER){

		a = 1;//(NfV.x*NfV.x)+(NfV.y*NfV.y)+(NfV.z*NfV.z);
		b = 2*((sp->loc.x*NfV.x)+(sp->loc.z*NfV.z)-((foot->loc.x*NfV.x)+(foot->loc.z*NfV.z)));
		c = (SQR(sp->loc.x)+SQR(foot->loc.x)-(2*sp->loc.x*foot->loc.x))+SQR(sp->loc.z)+SQR(foot->loc.z)-(2*sp->loc.z*foot->loc.z)-SQR(foot->r);
		k = quad_eqn(a,b,c);

		if(pLocData==IS_IN){
			sp->loc.x += (k*NfV.x);
			sp->loc.z += (k*NfV.z);
		}
		if(foot->vel.y>0 && (pLocData==IS_ON_UPPER ||pLocData==IS_IN)){
			sp->loc.y = foot->loc.y + ((foot->l)/2);
		}
		else if(foot->vel.y<0 && (pLocData==IS_ON_UPPER ||pLocData==IS_IN)){
			sp->loc.y = foot->loc.y - ((foot->l)/2);
		}
	}else if(foot->type==CUBE){
		//TODO:�̰͵� �ذ��ؾ���
	}else{
		local_debug("ERROR-FOOT SHAPE");
	}	
	return 0;
}
#endif

int PTPCore::particleCollisionDetect(const FOOT* foot, const PARTICLE* particle){
	int r = NOT_COLLISION;
	if(foot->type==SPHERE){
		r = isInASphere(foot,&particle->loc);
	}else if(foot->type==CYLINDER){
		r = isInACylinder(foot,&particle->loc);
	}else if(foot->type==CUBE){
		r = isInACube(foot,&particle->loc);
	}else{
		r = NOT_COLLISION;
		local_trace("Particle Collision Error\n");
	}
	return r;
}
#ifdef USE_SOIL_VISUALIZATION

int PTPCore::soilCollisionDetect(const FOOT* foot, const SOIL* soil){
	int r = NOT;
	if(foot->type==SPHERE){
		r =  isInASphere(foot,&soil->loc);
	}else if(foot->type==CYLINDER){
		r = isInACylinder(foot,&soil->loc);
	}else if(foot->type==CUBE){
		r = isInACube(foot,&soil->loc);
	}else{
		//
	}
	return r;
}
#endif

bool PTPCore::PTPCollisionDetectOne(const FOOT* foot){
	vecD ptpCenter = PTP_UTIL.getMiddlePoint(&PTP_GLOBAL.start,&PTP_GLOBAL.end);
	double rptp = sqrt(PTP_UTIL.getLengthSQR(&CELL_GLOBAL[0][PTP_GLOBAL.n.y-1][0].start,&ptpCenter,ALL));
	double big_r = sqrt(SQR(foot->r)+SQR(foot->l/2));		
	//	printf("tptp:%f r:%f footL(%f,%f,%f) d:%f \n",getLength(ptp.start,PTP_UTIL.getMiddlePoint(ptart,ptp.end)),foot.r,foot.loc.x,foot.loc.y,foot.loc.z,getLength(foot.loc,getMiddlePoint(ptp.start,ptp.end)));
	bool r=false;

	switch(foot->type){
	case SPHERE:
		if((PTP_UTIL.getLengthSQR(&foot->loc,&ptpCenter,ALL)<=SQR2(foot->r,rptp))){
			r=true;
		}
		break;
	case CYLINDER:
		if((PTP_UTIL.getLengthSQR(&foot->loc,&ptpCenter,ALL)<=SQR2(big_r,rptp))){
			r=true;
		}
		break;
	case CUBE: 
		//TODO : �����ؾ���
		if((PTP_UTIL.getLengthSQR(&foot->loc,&ptpCenter,ALL)<=SQR2(big_r,rptp))){
			r=true;
		}	
		break;
	default:
		local_trace("PTPCollision ERROR\n");
	}
	return r;

}

/*
int cellCollisionDetectVertex(const FOOT* foot, const vecN* cellN){

int r=NOT_COLLISION;
vecD testPoint[9];
int rTest[9];

testPoint[0] = CELL_GLOBAL[cellN->x][cellN->y][cellN->z].center;
testPoint[3]= CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start;	///abc
testPoint[7]= CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end;		///def
testPoint[1]= setVecD(CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start.x,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start.y,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end.z);	///abf
testPoint[6]= setVecD(CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start.x,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end.y,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start.z);	///aec
testPoint[8]= setVecD(CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end.x,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start.y,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start.z);	///dbc
testPoint[4]= setVecD(CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start.x,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end.y,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end.z);	///aef
testPoint[2]= setVecD(CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end.x,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end.y,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start.z);	///dec
testPoint[5]= setVecD(CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end.x,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start.y,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end.z);	///dbf

//local_debug("(%f,%f,%f)\n",testPoint[2].x,testPoint[2].y,testPoint[2].z);

//printf("cell->center:(%f,%f,%f) r:%f \n",CELL_GLOBAL[cellN->x][cellN->y][cellN->z].center.x,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].center.y,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].center.z,rcell);
switch(foot->type){
case SPHERE:
rTest[0] = isInASphere(foot,&testPoint[0]);
rTest[1] = isInASphere(foot,&testPoint[1]);
rTest[2] = isInASphere(foot,&testPoint[2]);
rTest[3] = isInASphere(foot,&testPoint[3]);
rTest[4] = isInASphere(foot,&testPoint[4]);
rTest[5] = isInASphere(foot,&testPoint[5]);
rTest[6] = isInASphere(foot,&testPoint[6]);
rTest[7] = isInASphere(foot,&testPoint[7]);
rTest[8] = isInASphere(foot,&testPoint[8]);

break;
case CYLINDER:
rTest[0] = isInACylinder(foot,&testPoint[0]);
rTest[1] = isInACylinder(foot,&testPoint[1]);
rTest[2] = isInACylinder(foot,&testPoint[2]);
rTest[3] = isInACylinder(foot,&testPoint[3]);
rTest[4] = isInACylinder(foot,&testPoint[4]);
rTest[5] = isInACylinder(foot,&testPoint[5]);
rTest[6] = isInACylinder(foot,&testPoint[6]);
rTest[7] = isInACylinder(foot,&testPoint[7]);
rTest[8] = isInACylinder(foot,&testPoint[8]);
break;
case CUBE:
//TODO : �����ؾ���
rTest[0] = isInACube(foot,&testPoint[0]);
rTest[1] = isInACube(foot,&testPoint[1]);
rTest[2] = isInACube(foot,&testPoint[2]);
rTest[3] = isInACube(foot,&testPoint[3]);
rTest[4] = isInACube(foot,&testPoint[4]);
rTest[5] = isInACube(foot,&testPoint[5]);
rTest[6] = isInACube(foot,&testPoint[6]);
rTest[7] = isInACube(foot,&testPoint[7]);
rTest[8] = isInACube(foot,&testPoint[8]);
break;
default:
break;
}

//local_debug("%d %d %d %d %d %d %d %d\n",rTest[0],rTest[1],rTest[2],rTest[3],rTest[4],rTest[5],rTest[6],rTest[7]);
#ifndef PRECISION_MIDDLE
if(rTest[0]!=NOT_COLLISION) r=r|COLLISION_0;
if(rTest[1]!=NOT_COLLISION) r=r|COLLISION_1;
if(rTest[2]!=NOT_COLLISION) r=r|COLLISION_2;
if(rTest[3]!=NOT_COLLISION) r=r|COLLISION_3;
if(rTest[4]!=NOT_COLLISION) r=r|COLLISION_4;
if(rTest[5]!=NOT_COLLISION) r=r|COLLISION_5;
if(rTest[6]!=NOT_COLLISION) r=r|COLLISION_6;
if(rTest[7]!=NOT_COLLISION) r=r|COLLISION_7;
if(rTest[8]!=NOT_COLLISION) r=r|COLLISION_8;
#endif

#ifdef PRECISION_MIDDLE
if(rTest[0]!=NOT_COLLISION) r=r|COLLISION_0;
if(rTest[1]==IS_IN) r=r|COLLISION_1;
if(rTest[2]==IS_IN) r=r|COLLISION_2;
if(rTest[3]==IS_IN) r=r|COLLISION_3;
if(rTest[4]==IS_IN) r=r|COLLISION_4;
if(rTest[5]==IS_IN) r=r|COLLISION_5;
if(rTest[6]==IS_IN) r=r|COLLISION_6;
if(rTest[7]==IS_IN) r=r|COLLISION_7;
if(rTest[8]==IS_IN) r=r|COLLISION_8;
#endif

//CELL_GLOBAL[cellN->x][cellN->y][cellN->z].CellVertex = r;
//local_debug("Vertex[%d][%d][%d] : %x %x\n",cellN->x,cellN->y,cellN->z,r,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].CellVertex);

return r;
}





int CellWorks(const FOOT* foot, const vecN* cellN){
int r_ver=NOT_COLLISION;
int r=NOT_COLLISION;//CELL_GLOBAL[cellN->x][cellN->y][cellN->z].CollisionType;
double rcell;
double line_R;
CELL* cp = &CELL_GLOBAL[cellN->x][cellN->y][cellN->z];
CELL bound;

bound.start = getFootAABBBound(foot,START);
bound.end = getFootAABBBound(foot,END);

if(foot->type==SPHERE) {line_R = foot->r;}
else if(foot->type==CYLINDER) {line_R =sqrt(SQR(foot->r)+SQR(foot->l/2));}

rcell = sqrt(PTP_UTIL.getLengthSQR(&CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start,&CELL_GLOBAL[cellN->x][cellN->y][cellN->z].center,ALL)); ///EXPENSIVE!!!

if(!AABBCollision(cp,&bound)){
r = NOT_COLLISION;
}else if((PTP_UTIL.getLengthSQR(&foot->loc,&CELL_GLOBAL[cellN->x][cellN->y][cellN->z].center,ALL)>SQR2(line_R,rcell))){
r = NOT_COLLISION;
}else{
///In Cells Case
r_ver = cellCollisionDetectVertex(foot,cellN);
cp->CellVertex = r_ver;
if((r_ver&COLLISION_1)==COLLISION_1){	r=r|COLLISION_CASE_20;		}
if((r_ver&COLLISION_2)==COLLISION_2){	r=r|COLLISION_CASE_21;		}
if((r_ver&COLLISION_3)==COLLISION_3){	r=r|COLLISION_CASE_22;		}
if((r_ver&COLLISION_4)==COLLISION_4){	r=r|COLLISION_CASE_23;		}
if((r_ver&COLLISION_5)==COLLISION_5){	r=r|COLLISION_CASE_24;		}
if((r_ver&COLLISION_6)==COLLISION_6){	r=r|COLLISION_CASE_25;		}
if((r_ver&COLLISION_7)==COLLISION_7){	r=r|COLLISION_CASE_26;		}
if((r_ver&COLLISION_8)==COLLISION_8){	r=r|COLLISION_CASE_27;		}

#ifdef PRECISION_MIDDLE
r =r|cellCollisionFeaturePoint(foot,cellN);
//local_debug("(%d,%d,%d)%s",cellN->x,cellN->y,cellN->z,intToBinary(cellCollision));
#endif
#ifdef PRECISION_HIGH
r=r|cellCollisionDetectLine(foot,cellN);
#endif
}
cp->CollisionType=r;
//local_debug("CW[%d][%d][%d]:%x %x\n",cellN->x,cellN->y,cellN->z,cp->CollisionType,cp->CellVertex);
return r;

}



_inline int CubicSphereVertexCollision(const FOOT* foot, const vecN* cellN){	
//TODO : �����
}

#ifdef PRECISION_HIGH

_inline double determinePlanePointLoc(const vecD n, const double d, const vecD p){
return (n.x*p.x) * (n.y*p.y) * (n.z*p.z) - d; 
}

_inline int getIntersectionNumLineSphere(const vecD* LineStart, const vecD* LineEnd, const vecD* SphereCenter, const double* SphereRadius){
vecD minusEndStart= minusVecD(LineEnd,LineStart);

double A =  PTP_UTIL.getVecDSizeSQR(minusEndStart,ALL);
double B = (LineStart->x*minusEndStart.x)+(LineStart->y*minusEndStart.y)+(LineStart->z*minusEndStart.z)
-(minusEndStart.x*SphereCenter->x)-(minusEndStart.y*SphereCenter->y)-(minusEndStart.z*SphereCenter->z);
double C = PTP_UTIL.getVecDSizeSQR(*LineStart,ALL) + PTP_UTIL.getVecDSizeSQR(*SphereCenter,ALL) + 
(LineStart->x*SphereCenter->x)+(LineStart->y*SphereCenter->y)+(LineStart->z*SphereCenter->z) - SQR(*SphereRadius);
double quad_r;
double t0=0, t1=0;
//TODO : CORRECT EQUATION... MAYBE WRONG NOW
quad_r = quad_eqn_new(A,B,C,&t0,&t1);
//local_debug("quad: %d, %f, %f\n",quad_r,t0,t1);
if(quad_r==2){
int r=0;
if(t0>0 && t0<1){
r++;
}
if(t1>0 && t1<1){
r++;
}
return r;
}else{
return 0;
}


}


_inline int CubicSphareLineCollision(const FOOT* foot, const vecN* cellN){	
///�� ������ ����
vecD pointABC=CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start;
vecD pointDEF=CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end;
vecD pointABF = setVecD(pointABC.x,pointABC.y,pointDEF.z);
vecD pointDBC = setVecD(pointDEF.x,pointABC.y,pointABC.z);
vecD pointDBF = setVecD(pointDEF.x,pointABC.y,pointDEF.z);
vecD pointAEC = setVecD(pointABC.x,pointDEF.y,pointABC.z);
vecD pointAEF = setVecD(pointABC.x,pointDEF.y,pointDEF.z);
vecD pointDEC = setVecD(pointDEF.x,pointDEF.y,pointABC.z);

int ray_r[12];
int r_r=NOT_COLLISION;
///RayABC_X
ray_r[0] = getIntersectionNumLineSphere(&pointABC,&pointDBC,&foot->loc,&foot->r);
///RayAEF_X
ray_r[1] = getIntersectionNumLineSphere(&pointAEF,&pointDEF,&foot->loc,&foot->r);
///RayABF_X
ray_r[2] = getIntersectionNumLineSphere(&pointABF,&pointDBF,&foot->loc,&foot->r);

///RayAEC_X
ray_r[3] = getIntersectionNumLineSphere(&pointAEC,&pointDEC,&foot->loc,&foot->r);

///RayABC_Y
ray_r[4] = getIntersectionNumLineSphere(&pointABC,&pointAEC,&foot->loc,&foot->r);

///RayABF_Y
ray_r[5] = getIntersectionNumLineSphere(&pointABF,&pointAEF,&foot->loc,&foot->r);

///RayDBC_Y
ray_r[6] = getIntersectionNumLineSphere(&pointDBC,&pointDEC,&foot->loc,&foot->r);

///RayDBF_Y
ray_r[7] = getIntersectionNumLineSphere(&pointDBF,&pointDEF,&foot->loc,&foot->r);

///RayABF_Z
ray_r[8] = getIntersectionNumLineSphere(&pointABF,&pointABC,&foot->loc,&foot->r);

///RayAEF_Z
ray_r[9] = getIntersectionNumLineSphere(&pointAEF,&pointAEC,&foot->loc,&foot->r);

///RayDBF_Z
ray_r[10] = getIntersectionNumLineSphere(&pointDBF,&pointDBC,&foot->loc,&foot->r);

///RayDEF_Z
ray_r[11] = getIntersectionNumLineSphere(&pointDEF,&pointDEC,&foot->loc,&foot->r);

//local_debug("R %d %d %d %d %d %d %d %d %d %d %d %d \n",ray_r[0],
ray_r[1],
ray_r[2],
ray_r[3],
ray_r[4],
ray_r[5],
ray_r[6],
ray_r[7],
ray_r[8],
ray_r[9],
ray_r[10],
ray_r[11]);

if(ray_r[0]>1){		r_r=r_r|COLLISION_CASE_7;}
if(ray_r[1]>1){		r_r=r_r|COLLISION_CASE_8;}
if(ray_r[2]>1){		r_r=r_r|COLLISION_CASE_9;}
if(ray_r[3]>1){		r_r=r_r|COLLISION_CASE_10;}
if(ray_r[4]>1){		r_r=r_r|COLLISION_CASE_11;}
if(ray_r[5]>1){		r_r=r_r|COLLISION_CASE_12;}
if(ray_r[6]>1){		r_r=r_r|COLLISION_CASE_13;}
if(ray_r[7]>1){		r_r=r_r|COLLISION_CASE_14;}
if(ray_r[8]>1){		r_r=r_r|COLLISION_CASE_15;}
if(ray_r[9]>1){		r_r=r_r|COLLISION_CASE_16;}
if(ray_r[10]>1){	r_r=r_r|COLLISION_CASE_17;}
if(ray_r[11]>1){	r_r=r_r|COLLISION_CASE_18;}
///���� ��ġ�� ���� ó�� ����

return r_r;	
}

int cellCollisionDetectLine(const FOOT* foot, const vecN* cellN){
if(foot->type==SPHERE){
return CubicSphareLineCollision(foot,cellN);
}else{
return NOT_COLLISION;
}
}
#endif



 vecD getFeaturePoint(const FOOT* foot, const vecD point){///EXPENSIVE
vecD dvec = setVecD(0.f,0.f,0.f); 
vecD direction_shape  = setVecD(0.f,0.f,0.f);
vecD direction = minusVecD(&foot->loc,&point);
switch(foot->type){
case SPHERE:
direction_shape = normalizeVector(&direction);
dvec.x = foot->loc.x - (direction_shape.x*foot->r);
dvec.y = foot->loc.y - (direction_shape.y*foot->r);
dvec.z = foot->loc.z - (direction_shape.z*foot->r);
break;
case CYLINDER:
if(PTP_UTIL.getVecDSizeSQR(&direction,Y_AXIS)<=SQR(foot->r)){
direction.y = 0;
direction_shape = normalizeVector(&direction);

dvec.x = foot->loc.x - (direction_shape.x*foot->r);
dvec.z = foot->loc.z - (direction_shape.z*foot->r);
if(direction_shape.y > 0){
dvec.y = foot->loc.y-(foot->l/2);
}else{
dvec.y = foot->loc.y+(foot->l/2);
}

}
break;
default:
local_trace("Feature Point ERROR\n");

}

return dvec;
}

int cellCollisionFeaturePoint(const FOOT* foot, const vecN* cellN){

vecD featurePoint[9];
int r_r=NOT_COLLISION;
int r[9];
register int i=0;

featurePoint[0]= getFeaturePoint(foot,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].center);	///EXPENSIVE
featurePoint[1]= getFeaturePoint(foot,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start);	///abc	
featurePoint[2]= getFeaturePoint(foot,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end);		///cdf
featurePoint[3]= getFeaturePoint(foot,setVecD(CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start.x,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start.y,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end.z));	///abf
featurePoint[4]= getFeaturePoint(foot,setVecD(CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start.x,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end.y,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start.z));	///aec
featurePoint[5]= getFeaturePoint(foot,setVecD(CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end.x,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start.y,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start.z));	///dbc
featurePoint[6]= getFeaturePoint(foot,setVecD(CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start.x,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end.y,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end.z));	///aef
featurePoint[7]= getFeaturePoint(foot,setVecD(CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end.x,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end.y,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start.z));	///dec
featurePoint[8]= getFeaturePoint(foot,setVecD(CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end.x,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start.y,CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end.z));	///dbf

for(i=1;i<9;i++){
r[i] = isInABox(&featurePoint[i],&CELL_GLOBAL[cellN->x][cellN->y][cellN->z].start,&CELL_GLOBAL[cellN->x][cellN->y][cellN->z].end);	
if(r[i]!=NOT_COLLISION){
if(foot->vel.x>0) r_r=r_r|COLLISION_CASE_1;
if(foot->vel.x<0) r_r=r_r|COLLISION_CASE_2;
if(foot->vel.y>0) r_r=r_r|COLLISION_CASE_3;
if(foot->vel.y<0) r_r=r_r|COLLISION_CASE_4;
if(foot->vel.z>0) r_r=r_r|COLLISION_CASE_5;
if(foot->vel.z<0) r_r=r_r|COLLISION_CASE_6;
}
}
return r_r;
}
*/

int PTPCore::ptpWorks(const FOOT* foot,const FOOT* lastFoot){
	register int i,j,k;
	vecN n;
	bool flagPTP;
	int pLocData=0;

	vecN loopStart;
	vecN loopEnd;

	flagPTP = PTPCollisionDetectOne(foot);

#ifdef FAST_CALC
	loopStart = getComputeNeededCellNum(foot,PTP_START);
	loopEnd= getComputeNeededCellNum(foot,PTP_END);
	if((loopStart.x>loopEnd.x) || (loopStart.y>loopEnd.y) || (loopStart.z>loopEnd.z)){
		local_trace("ERROR FAST CALC RANGE\n");
	}
#else
	loopStart = PTP_UTIL.setVecN(0,0,0);
	loopEnd = PTP_GLOBAL.n;
#endif
	//local_debug("1(%f,%f,%f),(%f,%f,%f) \n",lastFoot->loc.x,lastFoot->loc.y,lastFoot->loc.z,lastFoot->vel.x,lastFoot->vel.y,lastFoot->vel.z);
	//local_debug("2(%f,%f,%f),(%f,%f,%f) \n\n",foot->loc.x,foot->loc.y,foot->loc.z,foot->vel.x,foot->vel.y,foot->vel.z);

	if(flagPTP==true){
		for(i=0;i<PTP_GLOBAL.n.x;i++){
			for(j=0;j<PTP_GLOBAL.n.y;j++){
				for(k=0;k<PTP_GLOBAL.n.z;k++){
					CELL_GLOBAL[i][j][k].CollisionParticleNum=0;
				}
			}
		}
		///PARTICLES
		for(i=0;i<PTP_GLOBAL.n.x;i++){
			for(j=0;j<PTP_GLOBAL.n.y;j++){
				for(k=0;k<PTP_GLOBAL.n.z;k++){
					n=PTP_UTIL.setVecN(i,j,k);
					//CellWorks(foot,&n);
					///Move Particles	
					pLocData = particleCollisionDetect(foot,&PARTICLE_GLOBAL[i][j][k]);
					if(pLocData!=NOT_COLLISION){
						particleWorks(&n,foot,lastFoot,pLocData);
						PARTICLE_GLOBAL[i][j][k].isCollision = COLLISION;
					}else{
						PARTICLE_GLOBAL[i][j][k].isCollision = NOT_COLLISION;
					}

#ifdef USE_SOIL_VISUALIZATION
					///Soil Visualization	
					if(soilCollisionDetect(foot,&SOIL_GLOBAL[i][j][k]))
						soilWorks(&n,foot,lastFoot,pLocData);
#endif
				}
			}
		}
		return 1;
	}
	return 0;
}

int PTPCore::getCellProj(const FOOT* foot, const int type, const int mode){
	register int i,j,k;
	vecN loopStart;
	vecN loopEnd;
	unsigned int max_y_m[MAX_X][MAX_Z]={{0}};
	unsigned int max_x_m[MAX_Y][MAX_Z]={{0}};
	unsigned int max_z_m[MAX_X][MAX_Y]={{0}};
	unsigned int max_y_p[MAX_X][MAX_Z]={{0}};
	unsigned int max_x_p[MAX_Y][MAX_Z]={{0}};
	unsigned int max_z_p[MAX_X][MAX_Y]={{0}};

#ifdef FAST_CALC
	if(type==FAST){
		loopStart = getComputeNeededCellNum(foot,START);
		loopEnd= getComputeNeededCellNum(foot,END);
	}else{
		loopStart = PTP_UTIL.setVecN(0,0,0);
		loopEnd = PTP_GLOBAL.n;
	}	
	if((loopStart.x>loopEnd.x) || (loopStart.y>loopEnd.y) || (loopStart.z>loopEnd.z)){
		local_trace("ERROR FAST CALC RANGE\n");
	}
#else
	loopStart = setVecN(0,0,0);
	loopEnd = PTP_GLOBAL.n;
#endif

	for(i=loopStart.x;i<loopEnd.x;i++){
		for(k=loopStart.z;k<loopEnd.z;k++){
			for(j=loopStart.y;j<loopEnd.y;j++){
				if(CELL_GLOBAL[i][j][k].particleNum>=max_x_p[j][k]){
					max_x_p[j][k] = CELL_GLOBAL[i][j][k].particleNum;
					PROJ_X_P[j][k] = i; 
				}
				if(CELL_GLOBAL[i][j][k].particleNum>=max_z_p[i][j]){
					max_z_p[i][j] = CELL_GLOBAL[i][j][k].particleNum;
					PROJ_Z_P[i][j] = k; 
				}
				if(CELL_GLOBAL[i][j][k].particleNum>=max_y_m[i][k]){
					max_y_m[i][k] = CELL_GLOBAL[i][j][k].particleNum;
					PROJ_Y_M[i][k] = j; 
				}
			}
		}
	}


	for(i=loopEnd.x-1;i!=loopStart.x-1;i--){
		for(k=loopEnd.z-1;k!=loopStart.z-1;k--){
			for(j=loopEnd.y-1;j!=loopStart.y-1;j--){
				if(CELL_GLOBAL[i][j][k].particleNum>=max_y_p[i][k]){
					max_y_p[i][k] = CELL_GLOBAL[i][j][k].particleNum;
					PROJ_Y_P[i][k] = j; 
				}
				if(CELL_GLOBAL[i][j][k].particleNum>=max_x_m[j][k]){
					max_x_m[j][k] = CELL_GLOBAL[i][j][k].particleNum;
					PROJ_X_M[j][k] = i; 
				}
				if(CELL_GLOBAL[i][j][k].particleNum>=max_z_m[i][j]){
					max_z_m[i][j] = CELL_GLOBAL[i][j][k].particleNum;
					PROJ_Z_M[i][j] = k; 
				}
			}
		}
	}


	return 0;
}
double PTPCore::shearStress(const double disp, const double pressure){  
	// scalar shear stress in the direction of velocity 
	double sStress;
	sStress = (COHESION + pressure*tan(INT_ANGLE))*(1-exp(-ABS(disp)/DEFORMATION_MODULE));
	return sStress;
}  
bool PTPCore::calcForce(const FOOT* foot, const int type, vecD* forceR, vecD* forceT){
	register int i,j,k;
	vecN loopStart;
	vecN loopEnd;

	vecD forceTheta = PTP_UTIL.setVecD(0.f,0.f,0.f);
	vecD force = PTP_UTIL.setVecD(0.f,0.f,0.f);
	vecD fCell=PTP_UTIL.setVecD(0.f,0.f,0.f);
	vecD fTCell=PTP_UTIL.setVecD(0.f,0.f,0.f);
	double pressure = 0.f;
	vecD normalDirection= PTP_UTIL.setVecD(0.f,0.f,0.f);
	vecD cellSize = PTP_UTIL.setVecD(0.f,0.f,0.f);
	vecD fZCellRaw = PTP_UTIL.setVecD(0.f,0.f,0.f);
//	vecD fZDirection = PTP_UTIL.setVecD(0.f,0.f,0.f);
	double areaX=0,areaY=0,areaZ=0;
	double fZCellSize = 0.f;

#ifdef VEL_DIR_EFFECT
	vecD velDirEffect = PTP_UTIL.setVecD(1.f,1.f,1.f);
#endif
#ifdef USING_FR
	vecD fRCell = PTP_UTIL.setVecD(0.f,0.f,0.f);
	vecD frCell = PTP_UTIL.setVecD(0.f,0.f,0.f);
	double fRCellSize = 0.f;
	double PrV_Size = 0.f;
	double PrR_Size = 0.f;	
	vecD velDirection = PTP_UTIL.setVecD(0.f,0.f,0.f);
	vecD GDirection = PTP_UTIL.setVecD(0.f,0.f,0.f);
	vecD normalCellForce = PTP_UTIL.setVecD(0.f,0.f,0.f);
	vecD shearCellForce = PTP_UTIL.setVecD(0.f,0.f,0.f);
	vecD PrG = PTP_UTIL.setVecD(0.f,0.f,0.f);
	vecD fRDirection = PTP_UTIL.setVecD(0.f,0.f,0.f);
	vecD frDirection = PTP_UTIL.setVecD(0.f,0.f,0.f);
	vecD forceXZ = PTP_UTIL.setVecD(0.f,0.f,0.f);
	double forceXZ_Size = 0.f;
//	vecD start_loc = PTP_UTIL.setVecD(0.f,0.f,0.f);
	vecD lateral_loc = PTP_UTIL.setVecD(0.f,0.f,0.f);
	double 	shearStress_Size;
	
	double frCellSize = 0.f;
#else
	vecD fLCell = PTP_UTIL.setVecD(0.f,0.f,0.f);
	vecD fZCell = PTP_UTIL.setVecD(0.f,0.f,0.f);
#ifdef ACTIVATE_FL
	double fLCellSize = 0.f;
	vecD fLCellRaw = PTP_UTIL.setVecD(0.f,0.f,0.f);
	vecD fLDirection= PTP_UTIL.setVecD(0.f,0.f,0.f);
#endif
#endif

#ifdef USE_LOCAL_DEBUG
	int collisionCellNum = 0;
	int collisionParticleNum = 0;
#endif

#if defined USE_AREA_DEBUG
	vecD areaDEBUG = PTP_UTIL.setVecD(0.f,0.f,0.f);
#endif


#ifdef FAST_CALC
	if(type==FAST){
		loopStart = getComputeNeededCellNum(foot,START);
		loopEnd= getComputeNeededCellNum(foot,END);
	}else{
		loopStart = PTP_UTIL.setVecN(0,0,0);
		loopEnd = PTP_GLOBAL.n;
	}	
	if((loopStart.x>loopEnd.x) || (loopStart.y>loopEnd.y) || (loopStart.z>loopEnd.z)){
		local_trace("ERROR FAST CALC RANGE\n");
	}
#else
	loopStart = PTP_UTIL.setVecN(0,0,0);
	loopEnd = PTP_GLOBAL.n;
#endif
	//printf("Loop (%d,%d,%d) (%d,%d,%d)\n",loopStart.x,loopStart.y,loopStart.z,loopEnd.x,loopEnd.y,loopEnd.z);


#ifdef VEL_DIR_EFFECT
	velDirEffect = PTP_UTIL.normalizeVector(&foot->vel);
	velDirEffect.x = ABS(velDirEffect.x);
	velDirEffect.y = ABS(velDirEffect.y);
	velDirEffect.z = ABS(velDirEffect.z);
#else
	velDirEffect = PTP_UTIL.setVecD(1.0f,1.0f,1.0f);
#endif

//printf("###########\n");

	for(i=loopStart.x;i<loopEnd.x;i++){    /** loopStart and loopEnd are contact area with object  */
		for(k=loopStart.z;k<loopEnd.z;k++){
			for(j=loopStart.y;j<loopEnd.y;j++){
				//					local_debug("CELL[%d][%d][%d] \tPARTICLE %d C%x V%x \n",i,j,k,CELL_GLOBAL[i][j][k].particleNum,CELL_GLOBAL[i][j][k].CollisionType,CELL_GLOBAL[i][j][k].CellVertex);
				/// F = P*A?
				cellSize = PTP_UTIL.setVecD(0.f,0.f,0.f);	/// CELLSIZE[mm*MAG]

				if(CELL_GLOBAL[i][j][k].CollisionParticleNum){
					pressure = lookupTable(&CELL_GLOBAL[i][j][k]);	///[Pa]
					cellSize = PTP_UTIL.minusVecD(&CELL_GLOBAL[i][j][k].end,&CELL_GLOBAL[i][j][k].start);	/// CELLSIZE[mm*MAG]

					areaX = cellSize.y*cellSize.z*MM2M*MM2M;///[m^2*MAG^2]
					areaY = cellSize.x*cellSize.z*MM2M*MM2M;
					areaZ = cellSize.x*cellSize.y*MM2M*MM2M;

					///Get Normal Vector(Normalized)
					normalDirection = PTP_UTIL.minusVecD(&CELL_GLOBAL[i][j][k].center,&foot->loc);
					normalDirection = PTP_UTIL.normalizeVector(&normalDirection);

					fZCellSize = ABS(areaY*pressure);
					vecD moveDirection = PTP_UTIL.setVecD(0.f,0.f,0.f); 
					vecD schearCellForce = PTP_UTIL.setVecD(0.f,0.f,0.f); 
	
					if (PTP_GLOBAL.lateral == 1 ) {
					lateral_loc = PTP_UTIL.setVecD(foot->loc.x,0.f,foot->loc.z);  
					lateral_loc = PTP_UTIL.minusVecD(&lateral_loc,&PTP_GLOBAL.start_loc);
					moveDirection = PTP_UTIL.normalizeVector(&foot->vel);
    				shearStress_Size = areaY*shearStress(PTP_UTIL.sqrtVecDP(&lateral_loc)*MM2M, pressure); 
    				shearCellForce.x = moveDirection.x*shearStress_Size;
    				shearCellForce.y = moveDirection.y*shearStress_Size;
     				shearCellForce.z = moveDirection.z*shearStress_Size;
     				   				
					}		
	//	printf("(%d) cfz=%f sss=%f p=%f a=%f sloc=%f, \n",PTP_GLOBAL.lateral, fZCellSize, shearStress_Size, pressure, areaY, PTP_UTIL.sqrtVecDP(&lateral_loc)*MM2M);
					
					///Get Fr,FR Direction (Normalized)
					switch(foot->type){
					case SPHERE:
						fRDirection.x = normalDirection.x;
						fRDirection.y = normalDirection.y;
						fRDirection.z = normalDirection.z;
						
					GDirection = PTP_UTIL.setVecD(0.f,-1.f,0.f);        //gravity direction
					
					PrG.x = GDirection.x*fZCellSize;
					PrG.y = GDirection.y*fZCellSize;
					PrG.z = GDirection.z*fZCellSize;
					
					PrR_Size = ABS(PTP_UTIL.InnerProduct(&PrG,&fRDirection));		
					//velocity drection pressure					
					
					normalCellForce.x = fRDirection.x*PrR_Size;
					normalCellForce.y = fRDirection.y*PrR_Size;
					normalCellForce.z = fRDirection.z*PrR_Size;
					
						break;
					case CYLINDER:
						fRDirection.x = 0;
						fRDirection.y = normalDirection.y;
						fRDirection.z = 0;

						fRDirection =PTP_UTIL.normalizeVector(&fRDirection);

						frDirection=PTP_UTIL.CrossProduct(&normalDirection, &foot->vel);
						frDirection=PTP_UTIL.CrossProduct(&frDirection,&normalDirection);
						frDirection.y = 0;
						frDirection =PTP_UTIL.normalizeVector(&frDirection);

						break;
					}
	//				printf("(%f %f %f)  (%f %f %f)\n",force.x, force.y, force.z, shearCellForce.x, shearCellForce.y, shearCellForce.z);
					fCell = PTP_UTIL.plusVecD(&normalCellForce, &shearCellForce);
			//		fCell =normalCellForce;
					
				
					force.x += fCell.x;
					force.y += fCell.y;    
					force.z += fCell.z;
					
					forceTheta.x += shearCellForce.x;
					forceTheta.y += shearCellForce.y;    
					forceTheta.z += shearCellForce.z;
					


					///F=P*A[(Pa)*(mm^2*MAG^2)]=[N*1000*MAG^2]
				}
			}
		}
	}


#if (defined USE_AREA_DEBUG)
	outCALC = fopen(filenameCALC, "a+");
	fprintf(outCALC,"%f,%f,%f,%f,%f,%f\n",foot->loc.x/MAG, foot->loc.y/MAG,foot->loc.z/MAG,areaDEBUG.x,areaDEBUG.y,areaDEBUG.z);
	fclose(outCALC);	
#endif

	//local_debug("Cell : %d ,Particle : %d \n",collisionCellNum,collisionParticleNum);
	//		local_debug("Cell : %d-%dOverlap(%d) ,Particle : %d Area(%e,%e,%e)\n",validCellNum,overlapNum,collisionCellNum,collisionParticleNum,areaDebug.x,areaDebug.y,areaDebug.z);

	force.x /= MAG*MAG; //*M2MM*M2MM;
	force.y /= MAG*MAG; //*M2MM*M2MM;
	force.z /= MAG*MAG; //*M2MM*M2MM;

	//printf("%f,%f,%f\n",fR.x,fR.y,fR.z);
	forceT->x =forceTheta.x;
	forceT->y =forceTheta.y;
	forceT->z =forceTheta.z;
	
	forceR->x = force.x;
	forceR->y = force.y;
	forceR->z = force.z;
	
	

	forceXZ = PTP_UTIL.setVecD(forceR->x,0.f,forceR->z);
	forceXZ_Size = PTP_UTIL.sqrtVecDP(&forceXZ);
	if(forceXZ_Size >= 0.1 && PTP_GLOBAL.lateral == 0){
	PTP_GLOBAL.lateral = 1; //on
	PTP_GLOBAL.start_loc = PTP_UTIL.setVecD(foot->loc.x,0.f,foot->loc.z); }
	else if(forceXZ_Size < 0.1 && PTP_GLOBAL.lateral == 1)
	{ PTP_GLOBAL.lateral = 0; } //off
	
//	printf("(contact %d)..forceXZ = %f (%f %f %f)\n",PTP_GLOBAL.lateral,forceXZ_Size,forceR->x,forceR->y,forceR->z);
		
	
	return true;
	///F=-A+k*abs(pow((z-z0/1000),n))
	///RETURN VALUE : FORCE(N)
}

double PTPCore::lookupTable(const CELL* cell){
	double k,n,r,temp,p;

	k = LOOKUP_K;	
	n = LOOKUP_N;

	temp = cell->averageLoc.y*MM2M;  
	if(temp > 0) temp=0;
//	p = k*pow(-1*0.001,n);
	r = k*pow(ABS(temp),n);
	
//	printf("## %f %f \n", r, temp);
	
	return r;
	/// RETURN VALUE : PRESSURE[Pa]
}

int PTPCore::isInABox(const vecD* p,const vecD* start,const vecD* end){
	vecD min = PTP_UTIL.getMinVec(start,end);
	vecD max = PTP_UTIL.getMaxVec(start,end);
	//	local_debug("x(%f<%f<%f) y(%f<%f<%f) z(%f<%f<%f)\n",  min.x,p->x,max.x,min.y,p->y,max.y,min.z,p->z,max.z);
	if(p->x>min.x && p->x<max.x && p->y>min.y && p->y<max.y && p->z>min.z && p->z<max.z){
		return IS_IN;
	}else if(FLOAT_EQ(p->x,min.x) && p->y>=min.y && p->y<=max.y && p->z>=min.z && p->z<=max.z){
		return IS_ON_LEFT;
	}else if(FLOAT_EQ(p->x,max.x) && p->y>=min.y && p->y<=max.y && p->z>=min.z && p->z<=max.z){
		return IS_ON_RIGHT;
	}
	else if(FLOAT_EQ(p->y,min.y) && p->x>=min.x && p->y<=max.x && p->z>=min.z && p->z<=max.z){
		return IS_ON_TOP;
	}else if(FLOAT_EQ(p->y,max.y) && p->x>=min.x && p->y<=max.x && p->z>=min.z && p->z<=max.z){
		return IS_ON_BOTTOM;
	}
	else if(FLOAT_EQ(p->z,min.z) && p->y>=min.y && p->y<=max.y && p->x>=min.x && p->x<=max.x){
		return IS_ON_UPPER;
	}else if(FLOAT_EQ(p->z,max.z) && p->y>=min.y && p->y<=max.y && p->x>=min.x && p->x<=max.x){
		return IS_ON_UNDER;
	}else{
		return NOT_COLLISION;
	}
}

int PTPCore::isInACube(const FOOT* foot, const vecD* point){
	vecD start= PTP_UTIL.setVecD(foot->loc.x-foot->r,foot->loc.y-foot->r,foot->loc.z-foot->r);
	vecD end= PTP_UTIL.setVecD(foot->loc.x+foot->r,foot->loc.y+foot->r,foot->loc.z+foot->r);
	return isInABox(point,&start,&end);
}

int PTPCore::isInASphere(const FOOT* foot, const vecD* point){
	int r = NOT_COLLISION;
	vecD vec = PTP_UTIL.minusVecD(&foot->loc,point);
	if(PTP_UTIL.getVecDSizeSQR(&vec,ALL)<SQR(foot->r)){
		r = IS_IN;
	}else if(FLOAT_EQ(PTP_UTIL.getVecDSizeSQR(&vec,ALL),SQR(foot->r))){
		r = IS_ON;
	}
	return r;
}

int PTPCore::isInACylinder(const FOOT* foot, const vecD* point){
	int r = NOT_COLLISION;
	vecD vec = PTP_UTIL.minusVecD(&foot->loc,point);
	if(PTP_UTIL.getVecDSizeSQR(&vec,Y_AXIS)<=SQR(foot->r)){

		if(FLOAT_EQ(foot->loc.y-(foot->l/2),point->y)){ 
			r = IS_ON_UNDER;
		}
		else if(FLOAT_EQ(foot->loc.y+(foot->l/2),point->y)){
			r = IS_ON_UPPER;
		}else if(((foot->loc.y-(foot->l/2))<point->y) && ((foot->loc.y+(foot->l/2))>point->y)){
			r = IS_IN;
		}
	}else if(FLOAT_EQ(PTP_UTIL.getVecDSizeSQR(&vec,Y_AXIS),SQR(foot->r))){
		if(((foot->loc.y-(foot->l/2))<point->y) &&((foot->loc.y+(foot->l/2))>point->y)){
			r = IS_ON_SIDE;
		}
	}
	//		local_debug("%d,%f,%f,%f,%e,%e\n",r,(foot->loc.y),(foot->l/2),point->y,(foot->loc.y-(foot->l/2)-point->y),(foot->loc.y+(foot->l/2)-point->y));
	//		//local_debug("%f,%f\n ",PTP_UTIL.getVecDSizeSQR(PTP_UTIL.minusVecD(&foot->loc,point),Y_AXIS),SQR(foot->r));
	//		local_debug("CY (%f,%f,%f) (%f,%f,%f), %d \n",foot->loc.x,(foot->loc.y-(foot->l/2)),foot->loc.z,point->x,point->y,point->z,r);
	return r;
}

} //end of namespace soildynamics
