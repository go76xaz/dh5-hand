# One fingertip sensor reading (Hualichuang 3-axis sensor), relative to the
# last sensor calibration (dh5/calibrate_sensors).
#
# Units are not documented by the manufacturer; they look like N*mm for
# mx/my and N for fz. Observed channel mapping:
#   contact on the front (pad) of the fingertip -> mx
#   contact from the side                       -> my (sign gives left/right)
#   push along the finger onto the tip          -> fz (plus some mx)
string id        # "1"-"5": 1 thumb, 2 index, 3 middle, 4 ring, 5 little
float32 mx
float32 my
float32 fz
bool stale       # true if the values have not changed at all for the node's
                 # stale_timeout (the hand stopped updating them), or if the
                 # read failed - then mx/my/fz are NaN
