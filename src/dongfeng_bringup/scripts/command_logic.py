"""ROS-independent command handling for the 30 cm sandbox roads."""
import math

MAX_LINEAR = 0.25
MAX_ANGULAR = 1.2


def manual_publish_needed(velocity, previous, key_received):
    """An idle terminal must not continuously revoke automatic control."""
    return key_received or velocity != (0.,0.) or previous != (0.,0.)


class TerminalKeys:
    """Suppress escape sequences even when terminal reads split their bytes."""
    def __init__(self):
        self.escape = ''

    def feed(self, raw):
        keys = []
        for c in raw.decode('ascii', errors='ignore'):
            if c == '\x1b':
                self.escape = 'esc'
                keys.append(' ')  # Escape / arrows request a stop.
            elif self.escape == 'esc':
                self.escape = 'sequence' if c in ('[', 'O') else ''
            elif self.escape == 'sequence':
                if '@' <= c <= '~':
                    self.escape = ''
            else:
                keys.append(c)
        return keys


class CommandLease:
    """A velocity remains valid only while fresh commands arrive (wall time)."""
    def __init__(self, timeout=0.4, max_linear=MAX_LINEAR, max_angular=MAX_ANGULAR):
        if not all(math.isfinite(v) and v > 0 for v in (timeout, max_linear, max_angular)):
            raise ValueError('Timeout and limits must be finite positive numbers')
        self.timeout, self.max_linear, self.max_angular = timeout, max_linear, max_angular
        self.last_time = None
        self.velocity = (0.0, 0.0)

    def update(self, linear, angular, now):
        if not all(math.isfinite(v) for v in (linear, angular, now)):
            self.stop()
            return False
        self.velocity = (max(-self.max_linear, min(self.max_linear, linear)),
                         max(-self.max_angular, min(self.max_angular, angular)))
        self.last_time = now
        return True

    def stop(self):
        self.velocity = (0.0, 0.0)
        self.last_time = None

    def sample(self, now):
        if self.last_time is None or not 0 <= now - self.last_time < self.timeout:
            self.stop()
        return self.velocity


class KeyboardCommands:
    # +angular.z turns left. Only planar motion is accepted.
    MOVES = {'w': (1, 0), 'i': (1, 0), 's': (-1, 0), ',': (-1, 0),
             'a': (0, 1), 'j': (0, 1), 'd': (0, -1), 'l': (0, -1),
             'u': (1, 1), 'o': (1, -1), 'm': (-1, -1), '.': (-1, 1)}

    def __init__(self, speed=0.10, turn=0.65, key_timeout=0.65):
        if not (0 < speed <= MAX_LINEAR and 0 < turn <= MAX_ANGULAR):
            raise ValueError('speed must be in (0, 0.25]; turn in (0, 1.2]')
        self.speed, self.turn = speed, turn
        self.lease = CommandLease(key_timeout)

    def key(self, key, now):
        key = key.lower()
        if key in self.MOVES:
            v, w = self.MOVES[key]
            self.lease.update(v * self.speed, w * self.turn, now)
        else:
            # Speed adjustments never revive an expired movement command.
            self.lease.stop()
            if key in ('+', '='):
                self.speed = min(MAX_LINEAR, self.speed + 0.02)
                self.turn = min(MAX_ANGULAR, self.turn + 0.1)
            elif key == '-':
                self.speed = max(0.02, self.speed - 0.02)
                self.turn = max(0.2, self.turn - 0.1)
        return key not in ('q', '\x03', '\x04')

    def sample(self, now):
        return self.lease.sample(now)
