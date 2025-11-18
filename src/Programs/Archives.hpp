#ifndef GP__ARCHIVES_HPP_
#define GP__ARCHIVES_HPP_

#include <vector>
#include <unordered_set>
#include "Utils/Misc.hpp"
#include <istream>
#include <ostream>

namespace GP
{
    struct ArchiveEntry
    {
	std::vector<double> m_state;
	std::vector<double> m_trajPoints; //x,y,v, theta, phi, time 
	std::vector<double> m_remainingPathPoints;
	double m_goalX;
	double m_goalY;
	double m_goalRadius;    
    };

    class Archives
    {
    public:
	Archives(void)
	{}

	virtual ~Archives(void)
	{
	    DeleteItems<ArchiveEntry*>(&m_entries);
	}

	virtual void SaveTrajectory(Solution &sol);
	
	virtual void Read(std::istream & in);

	virtual void Print(std::ostream &out);
	

	std::vector<ArchiveEntry*> m_entries;
	
    };
}

    
#endif
