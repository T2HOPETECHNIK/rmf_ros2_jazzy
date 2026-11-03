# Negotiation planning admission

The adapter shares one planning admission limit across all its robots and fleets.
ROS parameters set the limits when the adapter starts:

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `negotiation_max_active_services` | 4 | Maximum admitted negotiation planning services |
| `negotiation_max_queued_requests` | 32 | Maximum waiting table requests |

Both values must be positive. For example, pass
`--ros-args -p negotiation_max_active_services:=2 -p negotiation_max_queued_requests:=16`
to an adapter executable that forwards ROS arguments.

Queued requests contain table references, not planning services. Duplicate table
versions are ignored. Superseded tables and completed negotiations are removed
from the queue. Queue overflow and response deadline expiry forfeit the affected
table. Waiting consumes the original response timeout (15 seconds by default);
admission does not restart it. A slot stays occupied until the responder and its
planning jobs are released, including jobs draining after interruption.

These are count limits, not a byte limit. One planning service can still construct
many route alternatives, and the negotiation tree still consumes memory. Lower
limits can reduce peak planning memory at the cost of more forfeits or retries
under congestion. Validate throughput with the site's navigation graph and logs.
