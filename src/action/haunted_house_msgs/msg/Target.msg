# Something the ghost can reach for points: a person to haunt, or a piece of candy to eat.

# Unique within its array for the whole game, so a target can be tracked as others disappear
uint32 id
string name
# Exact position in the map frame. A person's is their head, which is where the ghost must reach
geometry_msgs/Point position
# Points scored for reaching it
uint32 points
