#ifndef LIDARDISTANCESENSOR_HPP_
#define LIDARDISTANCESENSOR_HPP_

#include "Config.hpp"

#include "AbstractSensor.hpp"
#include "DistancePercept.hpp"

namespace Model
{
	/**
	 * Compile time configurable length of the laser beam
	 */

	class Robot;
	typedef std::shared_ptr<Robot> RobotPtr;

	/**
	 *
	 */
	class LidarDistanceSensor : public AbstractSensor
	{
		public:
			/**
			 *
			 */
			explicit LidarDistanceSensor( Robot& aRobot);
			/**
			 *
			 */
			virtual std::shared_ptr< AbstractStimulus > getStimulus() const override;
			/**
			 *
			 */
			virtual std::shared_ptr< AbstractPercept > getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const override;
			/**
			 *
			 */
			static void setStdDev(double aStdDev) {LidarDistanceSensor::stddev = aStdDev;}
			/**
			 *
			 */
			static double getStdDev(){ return stddev;}
			/**
			 * @name Debug functions
			 */
			//@{
			/**
			 * Returns a 1-line description of the object
			 */
			virtual std::string asString() const override;
			/**
			 * Returns a description of the object with all data of the object usable for debugging
			 */
			virtual std::string asDebugString() const override;
			//@}
		protected:
		private:
			/**
			 * Standard deviation of the odometer per 10 pixels
			 */
			static double stddev;
	};
} // namespace Model
#endif /* LIDARDISTANCESENSOR_HPP_ */
