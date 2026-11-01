int32 UNSEEN=0
int32 OBSERVED=1
int32 COLLECTED=2

int32 id
float64 x  # Position in the map frame [m]
float64 y  # Position in the map frame [m]
float64 z  # Position in the map frame [m]
float64 radius  # Collection radius [m]
int32 gain
int32 status  # One of: UNSEEN, OBSERVED, COLLECTED
