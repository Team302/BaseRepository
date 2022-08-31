/*
 * DragonPDP.h
 */

#pragma once

#include <memory>
#include <frc/PowerDistribution.h>

class DragonPDP
{
	public:
		static DragonPDP* GetInstance();

		//=======================================================================================
		// Method:  		CreatePDP
		// Description:		Create a PDP from inputs
		// Returns:         std::shared_ptr<PowerDistributionPanel>
		//=======================================================================================
		frc::PowerDistribution* CreatePDP
		(
			int			canID				// <I> - PDP CAN ID
		);

		frc::PowerDistribution* GetPDP() const;

	private:
		DragonPDP();
		virtual ~DragonPDP() = default;

		static DragonPDP*						m_instance;
		mutable frc::PowerDistribution*			m_pdp;

};


