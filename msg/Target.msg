# This message describes a target as in sections 7.2, 7.5 and 7.6 of the rules.
# This message definition is to be shared for all target types.

# Target type.
TargetType type

# Target latitude and longitude (optional).
# Need only be set for standard, QRC and emergent targets.
float64 latitude
float64 longitude

# Target orientation (optional).
# Need only be set for standard and off-axis targets.
Orientation orientation

# Target shape (optional).
# Need only be set for standard and off-axis targets.
Shape shape

# Target background and foreground colors (optional).
# Need only be set for standard and off-axis targets.
Color background_color
Color alphanumeric_color

# Target alphanumeric message (optional).
# Need only be set for standard and off-axis targets.
string alphanumeric

# Target description (optional).
# For QRC target: exact QRC message.
# For emergent target: general description of the emergent target.
# Need only be set for QRC and emergent targets.
string description

# Whether this target was detected, localized, and characterized autonomously,
# -- with no human input -- as per section 7.3 of the rules (optional).
# No more than six targets should be marked as autonomous as per the rules.
# Need only be set for standard targets.
bool autonomous
