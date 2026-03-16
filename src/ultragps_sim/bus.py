"""Very small in-process publish/subscribe bus to emulate ROS topics."""

from collections import defaultdict
from typing import Any, Callable


class SimpleBus:
    """Minimal topic bus for educational simulation use."""

    def __init__(self) -> None:
        self._subscribers: dict[str, list[Callable[[Any], None]]] = defaultdict(list)
        self._last_message: dict[str, Any] = {}

    def subscribe(self, topic: str, callback: Callable[[Any], None]) -> None:
        self._subscribers[topic].append(callback)

    def publish(self, topic: str, message: Any) -> None:
        self._last_message[topic] = message
        for callback in self._subscribers.get(topic, []):
            callback(message)

    def last_message(self, topic: str) -> Any | None:
        return self._last_message.get(topic)
