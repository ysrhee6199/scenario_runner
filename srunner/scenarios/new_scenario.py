import random

import py_trees

import carla
from srunner.scenariomanager.weather_sim import (Weather, OSCWeatherBehavior,MOSCWeatherBehavior)
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      KeepVelocity,
                                                                      StopVehicle,
                                                                      WaypointFollower,
                                                                      ChangeWeather)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               DriveDistance,
                                                                               StandStill,
                                                                               InTriggerDistanceToLocation)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import (get_waypoint_in_distance,get_location_in_distance)


class NewScenario(BasicScenario):
    """
    Some documentation on NewScenario
    :param world is the CARLA world
    :param ego_vehicles is a list of ego vehicles for this scenario
    :param config is the scenario configuration (ScenarioConfiguration)
    :param randomize can be used to select parameters randomly (optional, default=False)
    :param debug_mode can be used to provide more comprehensive console output (optional, default=False)
    :param criteria_enable can be used to disable/enable scenario evaluation based on test criteria (optional, default=True)
    :param timeout is the overall scenario timeout (optional, default=60 seconds)
    """

    # some ego vehicle parameters
    # some parameters for the other vehicles

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Initialize all parameters required for NewScenario
        """
        """
        Setup all relevant parameters and create scenario
        """
        self.timeout = timeout
        self.ego_vehicles = ego_vehicles
        self._map = CarlaDataProvider.get_map()
        self.distance = 100
        self.criteria_enable = False
        self._trigger_dist = 2
        self.trigger_location, _ = get_location_in_distance(self.ego_vehicles[1], self.distance)
        self.new_weather =  Weather(carla.WeatherParameters.HardRainNoon)
        # Call constructor of BasicScenario
        super(NewScenario, self).__init__(
          "NewScenario",
          ego_vehicles,
          config,
          world,
          debug_mode,
          criteria_enable=self.criteria_enable)


    def _create_behavior(self):
        """
        Setup the behavior for NewScenario
        """
        root = py_trees.composites.Parallel("Newscenario", py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        change_weather_behavior = ChangeWeather(weather=self.new_weather)
        osc_weather_behavior = OSCWeatherBehavior()


        sequence = py_trees.composites.Sequence()
        sequence.add_child(change_weather_behavior)
        sequence.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[1], self.trigger_location, self._trigger_dist))

        sequence.add_child(osc_weather_behavior)   
        root.add_child(sequence)
        return root

    def _create_test_criteria(self):
        """
        Setup the evaluation criteria for NewScenario
        """

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()