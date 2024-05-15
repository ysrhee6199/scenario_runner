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
                                                                      ChangeRoadFriction)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               DriveDistance,
                                                                               StandStill,
                                                                               InTriggerDistanceToLocation,
                                                                               VelocityPublisher,
                                                                               BrakePublisher,
                                                                               TimeGapPublisher,
                                                                               TimeOfWaitComparison)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import (BasicScenario,ROSBasicScenario)
from srunner.tools.scenario_helper import (get_waypoint_in_distance,get_location_in_distance)


class NewScenario(ROSBasicScenario):
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
        self.distance = 100
        self.criteria_enable = False
        self._trigger_dist = 2
        self.trigger_location, _ = get_location_in_distance(self.ego_vehicles[1], self.distance)
        self.trigger_end_location, _ = get_location_in_distance(self.ego_vehicles[1], 300)
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
        osc_weather_behavior = MOSCWeatherBehavior()
        change_road_friction = ChangeRoadFriction(0.6)

        sequence = py_trees.composites.Sequence()
        sequence.add_child(change_weather_behavior)

        sequence.add_child(TimeGapPublisher(name="Publish Timegap", timegap = 0.5))
        sequence.add_child(VelocityPublisher(name="Publish Velocity",target_speed = 14.0))
        sequence.add_child(BrakePublisher(name="Publish Brake",brake=False))


        sequence.add_child(TimeOfWaitComparison(duration_time = 5))
        

        sequence.add_child(osc_weather_behavior) 
        sequence.add_child(change_road_friction)
        sequence.add_child(SpawnAndDriveVehicle(scenario=self, ego_vehicle=self.ego_vehicles[1], distance_ahead=40))
        sequence.add_child(TimeOfWaitComparison(duration_time = 20))
        
        sequence.add_child(BrakePublisher(name="Publish Brake3",brake=True))
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

    def spawn_vehicle_ahead(self, reference_vehicle, distance):
        """
        Spawns a vehicle at a given distance ahead of a reference vehicle.
        """
        spawn_transform = reference_vehicle.get_transform()
        forward_vector = spawn_transform.get_forward_vector()

        # Calculate the new spawn location
        new_location = spawn_transform.location + forward_vector * distance
        spawn_transform.location = new_location

        # Ensure there is no collision at the spawn location

        blueprint = self.world.get_blueprint_library().filter('vehicle.*')[0]
        first_vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', spawn_transform)
        if first_vehicle:
            self.other_actors.append(first_vehicle)
            return first_vehicle
        else:
            raise RuntimeError("Spawn failed because of collision at spawn position")
        

class SpawnAndDriveVehicle(py_trees.composites.Sequence):
    def __init__(self, scenario, ego_vehicle, distance_ahead, name="SpawnAndDriveVehicle"):
        super(SpawnAndDriveVehicle, self).__init__(name)
        self.scenario = scenario
        self.ego_vehicle = ego_vehicle
        self.distance_ahead = distance_ahead
        self.vehicle = None

        # Add behavior to wait until vehicle is spawned

    def setup(self, timeout):
        return True

    def initialise(self):
        self.vehicle = self.scenario.spawn_vehicle_ahead(self.ego_vehicle, self.distance_ahead)
        # Add behaviors to control the vehicle's speed after it has been spawned
        self.add_child(ChangeVehicleSpeed(self.vehicle, 60 / 3.6, duration=5))  # 60 km/h for 5 seconds
        self.add_child(ChangeVehicleSpeed(self.vehicle, 30 / 3.6, duration=5))  # 30 km/h for 5 seconds
        self.add_child(ChangeVehicleSpeed(self.vehicle, 80 / 3.6, duration=5))  # 80 km/h for 5 seconds
        self.add_child(StopVehicle(self.vehicle, brake_value=1.0))  # Stop the vehicle



class ChangeVehicleSpeed(py_trees.behaviour.Behaviour):
    def __init__(self, vehicle, speed, duration, name="ChangeVehicleSpeed"):
        super(ChangeVehicleSpeed, self).__init__(name)
        self.vehicle = vehicle
        self.speed = speed
        self.duration = duration
        self.start_time = None

    def initialise(self):
        self.start_time = None

    def update(self):
        if self.vehicle is None:
            return py_trees.common.Status.FAILURE

        if self.start_time is None:
            self.start_time = self.vehicle.get_world().get_snapshot().timestamp.elapsed_seconds

        current_time = self.vehicle.get_world().get_snapshot().timestamp.elapsed_seconds
        elapsed_time = current_time - self.start_time

        if elapsed_time < self.duration:
            self.vehicle.set_target_velocity(carla.Vector3D(self.speed, 0, 0))
            return py_trees.common.Status.RUNNING
        else:
            self.vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        if self.vehicle is not None:
            self.vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))