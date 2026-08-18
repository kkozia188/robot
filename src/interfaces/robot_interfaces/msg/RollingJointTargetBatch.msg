# This is a transport ceiling, not the configured controller capacity.
uint16 TRANSPORT_MAX_POINTS=256
uint16 PROTOCOL_MAJOR=1
uint16 PROTOCOL_MINOR=0

uint16 protocol_major
uint16 protocol_minor
unique_identifier_msgs/UUID controller_boot_id
unique_identifier_msgs/UUID session_id
unique_identifier_msgs/UUID client_instance_id
uint64 sequence
uint64 replace_from_ns
RollingJointPoint[<=256] points
