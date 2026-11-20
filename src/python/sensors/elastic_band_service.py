"""Serve /elastic_band/toggle for the Isaac sim's elastic band, as the RoboCasa sims do."""


class ElasticBandService:
    """Answer /elastic_band/toggle (std_srvs/srv/Trigger) on the sensor bridge's ROS node.

    The requests are handled in spin(), which the sim's main loop calls between physics
    steps: the band is toggled on the physics thread, never beside a step.
    """

    def __init__(self, bridge, band):
        """Create the service on bridge's node (a RosSensorBridge) for band (an ElasticBand).

        Raises RuntimeError when the bridge has no ROS node.
        """
        if bridge.node is None:
            raise RuntimeError('the elastic band toggle needs the sensor bridge\'s ROS node')
        from rclpy.executors import SingleThreadedExecutor
        from std_srvs.srv import Trigger
        self._band = band
        self._node = bridge.node
        self._executor = SingleThreadedExecutor(context=bridge.context)
        self._executor.add_node(self._node)
        self._service = self._node.create_service(Trigger, '/elastic_band/toggle', self._on_toggle)

    def _on_toggle(self, request, response):
        """Toggle the band and answer with its new state, as RoboCasa's MeasurementBridge does."""
        response.success = True
        response.message = 'enabled' if self._band.toggle() else 'disabled'
        return response

    def spin(self):
        """Handle the requests that have arrived, without waiting for more."""
        self._executor.spin_once(timeout_sec=0)

    def close(self):
        """Remove the service and the executor; call it before the bridge closes its node."""
        self._executor.remove_node(self._node)
        self._node.destroy_service(self._service)
        self._executor.shutdown()
