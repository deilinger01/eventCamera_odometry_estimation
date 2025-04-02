Header header     # standard ROS header
uint32 height     # image height
uint32 width      # image width

uint16[] x         # X-coordinates of events
uint16[] y         # Y-coordinates of events
uint8[] polarity  # Polarity of events (0 or 1)
uint64[] timestamp # timestamp of all events
