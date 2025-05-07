# Simulation interfaces

from abc import ABC, abstractmethod


class ISimulator(ABC):
    """
    Interface for simulators
    """

    @abstractmethod
    def simulation_duration_sec(self) -> float:
        """
        The simulation duration in seconds.

        Returns:
            float: Simulation duration in seconds
        """
        ...

    @abstractmethod
    def simulation_name(self) -> str:
        """
        Name of the simulation

        Returns:
            str: Name of the simulation
        """
        ...

    @property
    @abstractmethod
    def free_movement(self) -> bool:
        """
        Flag that indicates the drone is operator driven.

        Returns:
            bool: `True` if the drone is operator driven, otherwise `False`
        """
        ...

    @abstractmethod
    def run(self) -> None:
        """
        Run the simulation.
        """
        ...

    @abstractmethod
    def print_metrics(self) -> None:
        """
        Print the simulation's metrics
        """
        ...

    @abstractmethod
    def close(self) -> None:
        """
        CLose the simulation.
        """
        ...
