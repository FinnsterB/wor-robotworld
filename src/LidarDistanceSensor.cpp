#include "LidarDistanceSensor.hpp"
#include "LaserDistanceSensor.hpp" // Purely for laser beam length, TODO this is dumb

#include "Logger.hpp"
#include "Robot.hpp"
#include "RobotWorld.hpp"
#include "Wall.hpp"
#include "Shape2DUtils.hpp"
#include "MathUtils.hpp"

#include <random>

namespace Model
{
	/**
	 *
	 */
	/* static */ double LidarDistanceSensor::stddev = 10.0;
	/**
	 *
	 */
	LidarDistanceSensor::LidarDistanceSensor( Robot& aRobot) :
								AbstractSensor( aRobot)
	{
	}
	/**
	 *
	 */
	std::shared_ptr< AbstractStimulus > LidarDistanceSensor::getStimulus() const
	{
		std::vector<DistanceStimulus> pointCloud;
		Robot* robot = dynamic_cast<Robot*>(agent);
		if(robot)
		{
			std::random_device rd{};
			std::mt19937 gen{rd()};
		    std::normal_distribution<> noise{0,LidarDistanceSensor::stddev};

			double angle = 0.0;


			for (size_t i = 0; i < 180; i++)
			{
				angle += 2.0*3.14/180;
				
				wxPoint intersection{-1,-1};
				std::vector< WallPtr > walls = RobotWorld::getRobotWorld().getWalls();
				for (std::shared_ptr< Wall > wall : walls)
				{
					wxPoint wallPoint1 = wall->getPoint1();
					wxPoint wallPoint2 = wall->getPoint2();
					wxPoint robotLocation = robot->getPosition();
					wxPoint laserEndpoint{static_cast<int>(robotLocation.x + std::cos( angle) * laserBeamLength + noise(gen)) ,
										static_cast<int>(robotLocation.y + std::sin( angle) * laserBeamLength + noise(gen))};

					wxPoint currentIntersection = Utils::Shape2DUtils::getIntersection( wallPoint1, wallPoint2, robotLocation, laserEndpoint);

						if (currentIntersection != wxDefaultPosition)
					{
						if(intersection == wxDefaultPosition)
						{
							intersection = currentIntersection;
						}else if(Utils::Shape2DUtils::distance(robotLocation,currentIntersection) < Utils::Shape2DUtils::distance(robotLocation,intersection))
						{
							intersection = currentIntersection;
						}
					}
					if(intersection != wxDefaultPosition)
					{
						double distance = Utils::Shape2DUtils::distance(robotLocation,intersection);
						pointCloud.push_back(DistanceStimulus(angle, distance));
					}
					else{
						pointCloud.push_back(DistanceStimulus(angle, laserBeamLength));
					}
				}
			}
		}
		std::shared_ptr stimuli = std::make_shared< DistanceStimuli >();
		stimuli->stimuli = pointCloud;
		return stimuli;
	}
	/**
	 *
	 */
	std::shared_ptr< AbstractPercept > LidarDistanceSensor::getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const
	{
		Robot* robot = dynamic_cast< Robot* >( agent);
		if (robot)
		{
			wxPoint robotLocation = robot->getPosition();

			DistanceStimuli* distanceStimuli = dynamic_cast< DistanceStimuli* >( anAbstractStimulus.get());
			if(distanceStimuli)
			{
				if(distanceStimuli->stimuli.empty())
				{
					return std::make_shared<DistancePercepts>();
				}
				
				std::vector<DistancePercept> percepts;
				for(DistanceStimulus d: distanceStimuli->stimuli){
					wxPoint endpoint{	static_cast< int >( robotLocation.x + std::cos( d.angle)*d.distance),
								static_cast< int >( robotLocation.y + std::sin( d.angle)*d.distance)};
					percepts.push_back(DistancePercept(endpoint));
				}

				return std::make_shared<DistancePercepts>( percepts);
			}
		}

		return std::make_shared<DistancePercepts>();
	}
	/**
	 *
	 */
	std::string LidarDistanceSensor::asString() const
	{
		return "LidarDistanceSensor";
	}
	/**
	 *
	 */
	std::string LidarDistanceSensor::asDebugString() const
	{
		return asString();
	}
} // namespace Model
