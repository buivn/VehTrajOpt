/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */
#include "Programs/Archives.hpp"
#include "GP/Solution.hpp"

namespace GP
{
    /*struct ArchiveEntry
    {
	std::vector<double> m_state;
	std::vector<double> m_trajPoints;
	std::vector<double> m_remainingPathPoints;
	double m_goalX;
	double m_goalY;
	double m_goalRadius;    
	};*/
	void Archives::SaveTrajectory(Solution &sol) {
	    // auto ae = new ArchiveEntry();
	    // m_entries.push_back(ae);
	    
	    // ae->m_state.resize(sol.m_dimState);
	    // GP::CopyArray<double>(&(ae->m_state[0]), sol.m_dimState, &(sol.m_trajStates[0]));
	    
	    // ae->m_goalX = sol.m_goalX;
	    // ae->m_goalY = sol.m_goalY;
	    // ae->m_goalRadius = sol.m_goalRadius;
	    
	    // ae->m_trajPoints.resize(sol.m_trajStates.size());
	    // GP::CopyArray<double>(&(ae->m_trajPoints[0]), sol.m_trajStates.size(), &(sol.m_trajStates[0]));
		return;
	}
    void Archives::Read(std::istream & in) {
		DeleteItems<ArchiveEntry*>(&m_entries);
		m_entries.clear();

		int n = 0;
		int dim;
		double val;
		
		in >> n;
		for(int i = 0; i < n; ++i)
		{
			auto ae = new ArchiveEntry();
			m_entries.push_back(ae);

			in >> dim;
			for(int j = 0; j < dim; ++j)
			{
				in >> val;
				ae->m_state.push_back(val);
			}
			in >> ae->m_goalX >> ae->m_goalY >> ae->m_goalRadius;

			in >> dim;
			for(int j = 0; j < dim; ++j)
			{
				in >> val;
				ae->m_trajPoints.push_back(val);
			}
		}
		
    }
    

    void Archives::Print(std::ostream &out) {
		out << m_entries.size() << std::endl;
		for(auto & ae : m_entries)
		{
			out << ae->m_state.size() << " ";	    
			for(auto & val : ae->m_state)
				out << val << " ";	    
			out << ae->m_goalX << " " << ae->m_goalY << " " << ae->m_goalRadius << std::endl;

			out << ae->m_trajPoints.size()  << " ";
			for(auto & val : ae->m_trajPoints)
				out << val << " ";
			out << std::endl;
		}	
    }
    
}

    
