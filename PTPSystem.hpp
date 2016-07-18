#ifndef __PTPSYSTEM_H__
#define __PTPSYSTEM_H__
#include "PTPCore.hpp"
//#include "PTPUtil.hpp"

namespace soildynamics {

class PTPSystem : public PTPCore{
public:
	
	PTPSystem();
    ~PTPSystem();
    void test_class(const vecD i);
    double sysTest;

    void  endPTP();
    char* printPTP();
    char* printCELL(const vecN n);
    char* printPARTICLE(const vecN n);
    bool getPTPForce(FOOT foot, vecD* force, vecD* forceT);
private:

    
};

} //end of namespace soildynamics

#endif // __PTPSYSTEM_H__