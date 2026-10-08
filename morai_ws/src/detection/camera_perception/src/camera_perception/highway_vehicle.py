"""Shared camera vehicle-class selection for driving-situation gates."""


# The detector publishes one normalized ``Car`` class for situation gates.
# Bus/truck detections remain available on the obstacle topic, but must not
# independently switch the route into highway mode.
HIGHWAY_VEHICLE_CLASSES = frozenset(("car",))


def highway_vehicle_detected(labels):
    """Return true only for the normalized car class used by situation gates."""
    normalized = {str(label).strip().lower() for label in labels}
    return bool(normalized.intersection(HIGHWAY_VEHICLE_CLASSES))

