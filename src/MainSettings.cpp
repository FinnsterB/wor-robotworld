#include "MainSettings.hpp"

namespace Application
{
	/**
	 *
	 */
	MainSettings::MainSettings() : drawOpenSet(true), speed(10), worldNumber(0)
	{
	}
	/**
	 *
	 */
	MainSettings::~MainSettings()
	{
	}
	/**
	 *
	 */
	bool MainSettings::getDrawOpenSet() const
	{
		return drawOpenSet;
	}
	/**
	 *
	 */
	void MainSettings::setDrawOpenSet( bool aDrawOpenSet)
	{
		drawOpenSet = aDrawOpenSet;
	}
	/**
	 *
	 */
	unsigned long MainSettings::getSpeed() const
	{
		return speed;
	}
	/**
	 *
	 */
	void MainSettings::setSpeed( unsigned long aSpeed)
	{
		speed = aSpeed;
	}
	/**
	 *
	 */
	unsigned long MainSettings::getWorldNumber() const
	{
		return worldNumber;
	}
	/**
	 *
	 */
	void MainSettings::setWorldNumber( unsigned long aWorldNumber)
	{
		worldNumber = aWorldNumber;
	}
    void MainSettings::setUseKalmanFilter(bool aUseKalman)
    {
		useKalmanFilter = aUseKalman;
    }
    void MainSettings::setUseParticleFilter(bool aUseParticle)
    {
		useParticleFilter = aUseParticle;
    }
    bool MainSettings::getUseKalmanFilter()
    {
        return useKalmanFilter;
    }

    bool MainSettings::getUseParticleFilter()
    {
        return useParticleFilter;
    }

} /* namespace Application */
