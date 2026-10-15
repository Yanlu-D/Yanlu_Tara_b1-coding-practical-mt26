class PDController:
    """Proportional-derivative controller for submarine depth control.

    The controller calculates a control action from the difference between
    the reference and observed depths. It stores the previous error between
    time steps so that the derivative term can be calculated.
    """

    def __init__(self) -> None:
        self.kp = 0.15
        self.kd = 0.6
        self.previous_error = 0.0

    def control(self, reference: float, observation: float) -> float:
        """Calculate the control action for the current time step.

        Args:
            reference: Desired submarine depth.
            observation: Current observed submarine depth.

        Returns:
            The control action produced by the PD controller.
        """
        error = reference - observation

        proportional = self.kp * error
        derivative = self.kd * (error - self.previous_error)

        control_action = proportional + derivative

        self.previous_error = error

        return control_action

    def reset(self) -> None:
        """Reset the stored error before starting a new simulation."""
        self.previous_error = 0.0