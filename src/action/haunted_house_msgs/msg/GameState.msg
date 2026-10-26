# Counting down to the start: the ghost and the people are frozen
uint8 WAITING=0
# The clock is running
uint8 RUNNING=1
# Time is up, or everything was collected: the ghost and the people are frozen again
uint8 FINISHED=2

std_msgs/Header header
uint8 phase
# Seconds until the game starts while WAITING, and seconds left while RUNNING
float64 time_remaining_s
uint32 score
uint32 candies_eaten
uint32 people_haunted
uint32 candies_left
uint32 people_left
# Times the ghost has flown into a wall or furniture. Costs nothing but pride
uint32 bonks
