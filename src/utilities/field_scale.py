"""Scale between the field images and field inches, for every widget that draws on the field.

Every field image in assets/ is FIELD_IMAGE_PX square with the field centre at the
image centre, so a canvas position p (px) is field position (p / FIELD_IMAGE_PX - 0.5)
* FIELD_IMAGE_IN inches. Route files store inches, so changing the scale moves where
routes are drawn on the image, never the routes or their trajectories.
"""

#: Width and height of every field image, pixels
FIELD_IMAGE_PX = 2000

#: Field inches spanned by one image width. Measured 2026-10-09: four tiles on the team's
#: field are 94.75 in (23.6875 in each), and the tile seams in both field images sit
#: 323.36 px apart (fitted over five seams per axis). Was 146.446120719, 0.04% smaller.
#:
#: The image's inner wall faces are 1932 px apart, 141.5 in at this scale, while the
#: firmware's wall resets use 140.5 in (Config::DISTANCE_RESET_WALL_POSITION = 70.25):
#: the image hides less of each edge tile under the wall than the real field does, so
#: no single scale fits both. Tiles were chosen because field elements sit on them; the
#: drawn walls are about 0.5 in outside the walls the resets measure against.
FIELD_IMAGE_IN = FIELD_IMAGE_PX * 23.6875 / 323.36

#: FIELD_IMAGE_IN in feet, the unit the splines and motion profile use
FIELD_IMAGE_FT = FIELD_IMAGE_IN / 12.0
