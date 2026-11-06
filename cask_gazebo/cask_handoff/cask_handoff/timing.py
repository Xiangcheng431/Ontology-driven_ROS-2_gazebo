"""Bounded approach budget and measured receiver progress for cargo handoff."""
import math

# Receiver navigation already allows 600 s. Transfer/contact limits stay local.
RECEIVER_APPROACH_TIMEOUT = 600.0
GROUP_TIMEOUT = 900.0
RECEIVER_PROGRESS_TIMEOUT = 45.0


class ReceiverProgress:
    """Heartbeats alone cannot keep a stationary receiver alive indefinitely."""
    def __init__(self, timeout):
        self.timeout = timeout
        self.anchor = self.actor = self.stage = self.changed = None

    def observe(self, peer, now):
        if peer and peer['stage'] == 'FAILED':
            # Existing coordinator may select a different pre-dock receiver.
            self.anchor = self.actor = self.stage = self.changed = None
            return True
        pose = peer['data'].get('progress_pose') if peer else None
        if (isinstance(pose, (list, tuple)) and len(pose) == 4
                and all(isinstance(v, (int, float)) and math.isfinite(v) for v in pose)):
            actor, stage = peer['actor'], peer['stage']
            progressed = (self.anchor is None or actor != self.actor or stage != self.stage
                          or math.dist(pose[:2], self.anchor[:2]) >= .02
                          or abs(math.atan2(math.sin(pose[2]-self.anchor[2]),
                                            math.cos(pose[2]-self.anchor[2]))) >= .02
                          or abs(pose[3]-self.anchor[3]) >= .01)
            if progressed:
                self.anchor, self.actor, self.stage, self.changed = tuple(pose), actor, stage, now
        # Legacy receivers without progress telemetry retain the absolute budget.
        return self.changed is None or now-self.changed < self.timeout
