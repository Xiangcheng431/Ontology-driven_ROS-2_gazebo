"""Fresh physical contact evidence, distinct from a position-only check."""

import threading
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts


class SupportContacts:
    def __init__(self, node, cargo_model, topic):
        self.cargo_model = cargo_model
        self.lock = threading.Lock()
        self.pairs = ()
        self.received = 0.0
        self._clock = node.get_clock()
        self.received_clock = None
        self.subscription = node.create_subscription(
            Contacts,
            topic,
            self.receive,
            qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup(),
        )

    def receive(self, message):
        pairs = []
        for contact in message.contacts:
            first = getattr(contact.collision1, "name", contact.collision1)
            second = getattr(contact.collision2, "name", contact.collision2)
            pairs.append((str(first), str(second)))
        with self.lock:
            self.pairs = tuple(pairs)
            self.received = time.monotonic()
            self.received_clock = self._clock.now().nanoseconds

    def _contact_fresh(self, received, received_clock):
        # CASK_CONTACT_SIMULATION_FRESHNESS_V1
        if not received or received_clock is None:
            return False
        clock_age = self._clock.now().nanoseconds - received_clock
        wall_age = time.monotonic() - received
        # Retain the 0.5-second physical-time window. Bound a paused clock
        # or stalled callbacks independently; never retain evidence forever.
        return 0 <= clock_age <= 500000000 and 0 <= wall_age <= 2.0

    def supported(self, model, link):
        with self.lock:
            pairs, received, received_clock = self.pairs, self.received, self.received_clock
        if not self._contact_fresh(received, received_clock):
            return False
        return any(
            (self.cargo_model + "::" in a and model + "::" in b and link in b)
            or (self.cargo_model + "::" in b and model + "::" in a and link in a)
            for a, b in pairs
        )

    def status(self, model, link):
        """Explain absent evidence without substituting position for contact."""
        with self.lock:
            pairs, received, received_clock = self.pairs, self.received, self.received_clock
        if not received:
            return "NO_MESSAGES"
        if not self._contact_fresh(received, received_clock):
            return "STALE"
        if not pairs:
            return "NO_CONTACTS"
        if self.supported(model, link):
            return "SUPPORT_CONTACT"
        first = pairs[0]
        return "OTHER_CONTACTS:" + "|".join(first)
