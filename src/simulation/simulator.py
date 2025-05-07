from src.world_entities.environment import Environment
from src.world_entities.base_station import BaseStation
from src.world_entities.drone import Drone

from src.utilities.utilities import (
    PathManager,
    current_date,
    euclidean_distance,
)
from src.utilities.config import SimConfig
from src.drawing import pp_draw

import numpy as np
import time


class Simulator:

    def __init__(self, cfg: SimConfig = SimConfig()):

        self.cur_step = 0
        self.sim_seed = cfg.sim_seed
        self.ts_duration_sec = cfg.sim_ts_duration
        self.sim_duration_ts = cfg.sim_duration
        self.env_width_meters, self.env_height_meters = (
            cfg.env_width,
            cfg.env_height,
        )
        self.n_drones = cfg.n_drones
        self.n_obstacles = cfg.n_obstacles
        self.grid_cell_size = (
            0
            if cfg.n_grid_cells <= 0
            else int(self.env_width_meters / cfg.n_grid_cells)
        )

        # if this coo is not none, then the drones are self driven
        self.drone_coo = cfg.initial_drone_coords
        self.selected_drone = None

        self.drone_speed_meters_sec = cfg.drone_speed
        self.drone_max_battery = cfg.drone_max_energy
        self.drone_max_buffer = cfg.drone_max_buffer_size
        self.drone_com_range_meters = cfg.drone_com_range
        self.drone_sen_range_meters = cfg.drone_sensing_range
        self.drone_radar_range_meters = cfg.drone_radar_radius
        self.bs_com_range_meters = cfg.drone_com_range
        self.bs_coords = cfg.base_station_coords
        self.target_coods = cfg.targets_coords
        self._config = cfg

        # create the world entites
        self.__set_randomness()
        self.__create_world_entities()
        self.__setup_plotting()

    def simulation_duration_sec(self):
        return self.sim_duration_ts * self.ts_duration_sec

    def simulation_name(self):
        return "seed{}-ndrones{}-date{}".format(
            self.sim_seed, self.n_drones, current_date()
        )

    @property
    def free_movement(self) -> bool:
        return self.drone_coo is not None

    def detect_key_pressed(self, key_pressed):
        """Moves the drones freely."""

        if key_pressed in ["a", "A"]:  # decrease angle
            self.selected_drone.angle -= self._config.drone_angle_increment
            self.selected_drone.angle = self.selected_drone.angle % 360

        elif key_pressed in ["d", "D"]:  # increase angle
            self.selected_drone.angle += self._config.drone_angle_increment
            self.selected_drone.angle = self.selected_drone.angle % 360

        elif key_pressed in ["w", "W"]:  # increase speed
            self.selected_drone.speed += self._config.drone_speed_increment

        elif key_pressed in ["s", "S"]:  # decrease speed
            self.selected_drone.speed -= self._config.drone_speed_increment

    def detect_drone_click(self, position):
        """Handles drones selection in the simulation."""
        click_coords_to_map = (
            self.environment.width / self._config.draw_size * position[0],
            self.environment.height
            / self._config.draw_size
            * (self._config.draw_size - position[1]),
        )
        entities_distance = [
            euclidean_distance(drone.coords, click_coords_to_map)
            for drone in self.environment.drones
        ]
        clicked_drone = self.environment.drones[
            np.argmin(entities_distance)
        ]  # potentially clicked drone

        TOLERATED_CLICK_DISTANCE = 40

        closest_drone_coords = clicked_drone.coords
        dron_coords_to_screen = (
            closest_drone_coords[0]
            * self._config.draw_size
            / self.environment.width,
            self._config.draw_size
            - (
                closest_drone_coords[1]
                * self._config.draw_size
                / self.environment.width
            ),
        )

        if (
            euclidean_distance(dron_coords_to_screen, position)
            < TOLERATED_CLICK_DISTANCE
        ):
            # DRONE WAS CLICKED HANDLE NOW
            self.on_drone_click(clicked_drone)

    def on_drone_click(self, clicked_drone):
        """Defines the behaviour following a click on a drone."""
        self.selected_drone = clicked_drone

    def __setup_plotting(self):
        if self._config.plot_sim or self._config.save_plot:
            self.draw_manager = pp_draw.PathPlanningDrawer(
                self.environment, self, borders=True
            )

    def __set_randomness(self):
        """Set the random generators."""
        self.rnd_env = np.random.RandomState(self.sim_seed)
        self.rnd_event = np.random.RandomState(self.sim_seed)

    def __create_world_entities(self):
        """Creates the world entities."""

        if self.drone_coo is None:
            self.path_manager = PathManager(
                self._config.fixed_tours_dir + "RANDOM_missions", self.sim_seed
            )

        self.environment = Environment(
            self.env_width_meters, self.env_height_meters, self
        )

        self.environment.spawn_obstacles()
        self.environment.spawn_targets(self.target_coods)

        base_stations = [
            BaseStation(self.bs_coords, self.bs_com_range_meters, self)
        ]

        drones = [
            Drone(
                identifier=i,
                path=(
                    [self.drone_coo]
                    if self.free_movement
                    else self.path_manager.path(i)
                ),
                bs=base_stations[0],
                angle=0,
                speed=0 if self.free_movement else self.drone_speed_meters_sec,
                com_range=self.drone_com_range_meters,
                sensing_range=self.drone_sen_range_meters,
                radar_range=self.drone_radar_range_meters,
                max_buffer=self.drone_max_buffer,
                max_battery=self.drone_max_battery,
                simulator=self,
            )
            for i in range(self.n_drones)
        ]

        self.selected_drone = drones[0]
        self.environment.add_base_station(base_stations)
        self.environment.add_drones(drones)

    def __plot(self, cur_step):
        """Plot the simulation"""

        if cur_step % self._config.skip_sim_step != 0:
            return

        if self._config.wait_sim_step > 0:
            time.sleep(self._config.wait_sim_step)

        self.draw_manager.grid_plot()
        self.draw_manager.borders_plot()

        for drone in self.environment.drones:
            self.draw_manager.draw_drone(drone, cur_step)

        for base_station in self.environment.base_station:
            self.draw_manager.draw_depot(base_station)

        for event in self.environment.get_valid_events(cur_step):
            self.draw_manager.draw_event(event)

        self.draw_manager.draw_simulation_info(
            cur_step=cur_step, max_steps=self.sim_duration_ts
        )
        self.draw_manager.draw_obstacles()
        self.draw_manager.draw_target(self.target_coods)
        self.draw_manager.update(
            save=self._config.save_plot,
            filename=self.simulation_name() + str(cur_step) + ".png",
        )

    def run(self):
        """The method starts the simulation."""

        for cur_step in range(self.sim_duration_ts):
            self.cur_step = cur_step

            for drone in self.environment.drones:
                self.environment.detect_collision(drone)
                drone.move()

            if self._config.save_plot or self._config.plot_sim:
                self.__plot(cur_step)

    def print_metrics(self):
        pass

    def close(self):
        pass
