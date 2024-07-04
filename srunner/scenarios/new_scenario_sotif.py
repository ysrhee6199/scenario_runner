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
                                                                      ChangeWeather,
                                                                      ChangeRoadFriction,
                                                                      UniformAcceleration,
                                                                      AccelerateToVelocity)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               DriveDistance,
                                                                               StandStill,
                                                                               InTriggerDistanceToLocation,
                                                                               VelocityPublisher,
                                                                               BrakePublisher,
                                                                               TimeGapPublisher,
                                                                               TimeOfWaitComparison,
                                                                               SensorPublisher)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import (BasicScenario,ROSBasicScenario)
from srunner.tools.scenario_helper import (get_waypoint_in_distance,get_location_in_distance)

from srunner.scenariomanager.timer import GameTime
class NewScenarioSotif(ROSBasicScenario):
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
        print("Heelo")
        self.world = world
        self.timeout = timeout
        self.ego_vehicles = ego_vehicles
        self._map = CarlaDataProvider.get_map()
        self.criteria_enable = False
        # Call constructor of BasicScenario
        super(NewScenarioSotif, self).__init__(
          "NewScenarioSotif",
          ego_vehicles,
          config,
          world,
          debug_mode,
          criteria_enable=self.criteria_enable)
    def _initialize_actors(self, config):

        # add actors from xml file
        for actor in config.other_actors:
            vehicle = CarlaDataProvider.request_new_actor(actor.model, actor.transform)
            self.other_actors.append(vehicle)
            vehicle.set_simulate_physics(enabled=False)

    def _create_behavior(self):
        """
        Setup the behavior for NewScenario
        """
        root = py_trees.composites.Parallel("Newscenario", py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        #change_weather_behavior = ChangeWeather(weather=self.new_weather)
        #osc_weather_behavior = MOSCWeatherBehavior()
        #change_road_friction = ChangeRoadFriction(0.6)

        sequence = py_trees.composites.Sequence()
        #sequence.add_child(change_weather_behavior)
        sequence.add_child(VelocityPublisher(name="Publish Velocity",target_speed = 0.0))
        sequence.add_child(TimeOfWaitComparison(duration_time = 10))
        sequence.add_child(TimeGapPublisher(name="Publish Timegap", timegap = 0.5))
        sequence.add_child(VelocityPublisher(name="Publish Velocity",target_speed = 20.00))
        sequence.add_child(BrakePublisher(name="Publish Brake",brake=False))
        sequence.add_child(TimeOfWaitComparison(duration_time = 45))
        #sequence.add_child(VelocityPublisher(name="Publish Velocity",target_speed = 25.00))
        sequence.add_child(SensorPublisher(name = "sensor"))
        sequence.add_child(TimeOfWaitComparison(duration_time = 3))
        #sequence.add_child(VelocityPublisher(name="Publish Velocity",target_speed = 19.4))
        sequence.add_child(SensorPublisher(name = "sensor"))
        sequence.add_child(TimeOfWaitComparison(duration_time = 45))
        sequence.add_child(BrakePublisher(name="Publish Brake",brake=True))
        #sequence.add_child(UniformAcceleration(actor = self.other_actors[0], start_velocity = self.other_actors.get_velocity().x**2 + self.other_actors.get_velocity().y**2, target_velocity = 16.67,  start_time = GameTime.get_time()))


        #sequence.add_child(TimeOfWaitComparison(duration_time = 17))
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


