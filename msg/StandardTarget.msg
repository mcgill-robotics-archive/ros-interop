# This message describes a standard target as in section 7.2.7 of the rules.

# Target latitude and longitude.
float64 latitude
float64 longitude

# Target orientation.
Orientation orientation

# Target shape.
Shape shape

# Target background and foreground colors.
Color background_color
Color alphanumeric_color

# Target alphanumeric message.
string alphanumeric

# Whether this target was detected, localized, and characterized autonomously,
# -- with no human input -- as per section 7.3 of the rules.
# No more than six targets should be marked as autonomous as per the rules.
bool autonomous
