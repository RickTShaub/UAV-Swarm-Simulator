"""
This file contains all the constants and parameters of the simulator.
It comes handy when you want to make one shot simulations, making parameters
and constants vary in every simulation. For an extensive experimental campaign
read the header at src.simulator.
"""

# ----------------------------- SIMULATION PARAMS ---------------------------- #

from dataclasses import dataclass, field

# TODO: Get rid of these constants.
ENV_WIDTH: float = 1500.0
"""Width of environment in meters"""

ENV_HEIGHT: float = 1500.0
"""Height of environmentin meters"""


@dataclass
class SimConfig:
    """
    Configuration data for simulation
    """

    sim_seed = 0
    """Seed of this simulation for randomization."""

    sim_duration: int = 3000000
    """Number of steps in the simulation."""

    sim_ts_duration: float = 0.150
    """Duration of a step in seconds."""

    env_width: float = ENV_WIDTH
    """Width of environment in meters"""

    env_height: float = ENV_HEIGHT
    """Height of environmentin meters"""

    n_drones: int = 2
    """Number of drones."""

    n_obstacles: int = 10
    """Number of random obstacles in the map"""

    n_grid_cells: int = 3
    """Number of cells in the grid."""

    initial_drone_coords: list[int] | None = field(
        default_factory=lambda: [ENV_WIDTH / 2, ENV_HEIGHT]
    )
    """
    IMPORTANT: coordinates of the drones at the beginning, it can be `None`. 
    In that case drone will follow fixed tours determined in fixed_tours_dir    
    """

    drone_speed: float = 5.0
    "Initial drone speed in m/s."

    drone_angle: float = 0
    """Angle of the drone in degrees (>= 0.0 <= 360.0)"""
    drone_speed_increment: float = 5.0
    """Speed increment at every key stroke in m/s"""

    drone_angle_increment: float = 45.0
    """Angle in degrees to increment at every key stroke"""

    drone_com_range: float = 100.0
    """meters, communication range of the drones."""

    drone_sensing_range: float = 0.0
    """Meters, the sensing range of the drones."""

    drone_max_buffer_size: int = 0
    """ax number of packets in the buffer of a drone."""

    drone_max_energy: int = 100
    """Max energy of a drone. I'm not sure of the units. Energy implies Wh,
    but the default value implies percent."""

    drone_radar_radius: int = 60
    """ Radar radius in meters"""

    # base station
    base_station_coords: list[int] = field(
        default_factory=lambda: [ENV_WIDTH / 2, 0]
    )
    """Coordinates of the base staion."""

    base_station_com_range: float = 200
    """Meters, communication range of the depot."""

    plot_trajectory_next_target: bool = True

    fixed_tours_dir: str = "data/tours/"
    """The path to the drones tours"""

    handcrafted_path: bool = False

    plot_sim: bool = True
    """Whether to plot or not the simulation
    (set to `False` for faster experiments)"""

    wait_sim_step: float = 0
    """seconds, pauses the rendering for x seconds.
    Value must be greater than or equal to 0"""

    skip_sim_step: int = 5
    """steps, plot the simulation every x steps.
    Value must be greater than 0"""

    draw_size: int = 700
    """size of the drawing window"""

    save_plot: bool = False
    """whether to save the plots of the simulation or not"""

    save_plot_dir: str = "data/plots/"
    """Where to save plots"""

    targets_coords: list[tuple[int, int]] = field(
        default_factory=lambda: [(750, 750)]
    )
    """Target coordinates"""

    def __post_init__(self):
        if self.skip_sim_step <= 0:
            raise ValueError("skip_sim_step must be greater than zero")
        if self.wait_sim_step < 0:
            raise ValueError(
                "wait_sim_step must be greater than or equal to zero"
            )
