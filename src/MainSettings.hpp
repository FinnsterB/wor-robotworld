#ifndef MAINSETTINGS_HPP_
#define MAINSETTINGS_HPP_

#include "Config.hpp"

namespace Application
{

	/*
	 *
	 */
	class MainSettings
	{
		public:
			/**
			 *
			 */
			MainSettings();
			/**
			 *
			 */
			virtual ~MainSettings();
			/**
			 *
			 */
			bool getDrawOpenSet() const;
			/**
			 *
			 */
			void setDrawOpenSet( bool aDrawOpenSet);
			/**
			 *
			 */
			unsigned long getSpeed() const;
			/**
			 *
			 */
			void setSpeed( unsigned long aSpeed);
			/**
			 *
			 */
			unsigned long getWorldNumber() const;
			/**
			 *
			 */
			void setWorldNumber( unsigned long aWorldNumber);
			/**
			 * 
			*/
			void setUseKalmanFilter(bool aUseKalman);
			/**
			*
			*/
			void setUseParticleFilter(bool aUseParticle);
			/**
			 * 
			*/
			bool getUseKalmanFilter();
			/**
			 * 
			*/
			bool getUseParticleFilter();

		private:
			bool drawOpenSet;
			unsigned long speed;
			unsigned long worldNumber;
			bool useKalmanFilter;
			bool useParticleFilter;
	};

} /* namespace Application */

#endif /* SRC_MAINSETTINGS_HPP_ */
